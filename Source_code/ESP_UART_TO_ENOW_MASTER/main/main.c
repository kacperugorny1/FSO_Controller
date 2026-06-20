#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "nvs_flash.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_now.h"
#include "esp_mac.h"
#include "driver/uart.h"
#include "driver/usb_serial_jtag.h" // Native USB Driver

// -----------------------------------------------------------------------------
// CONFIGURATION
// -----------------------------------------------------------------------------
#define UART_DEVICE        UART_NUM_1 // Connection to the local wired device
#define UART_BAUD_RATE     115200
#define UART_BUF_SIZE      1024
#define ESPNOW_MAX_DELAY   portMAX_DELAY

// GPIO Pins for the Local Device (UART1)
#define UART_DEVICE_TX_PIN 6
#define UART_DEVICE_RX_PIN 7

static const char *TAG = "MASTER_HUB";

// Queues for data streams
static QueueHandle_t uart_device_queue;
static QueueHandle_t espnow_rx_queue;

// The single remote Slave MAC address
uint8_t slave_mac[ESP_NOW_ETH_ALEN] = {0xF0, 0xF5, 0xBD, 0x0B, 0xEC, 0x24};

typedef struct {
    uint8_t data[ESP_NOW_MAX_DATA_LEN];
    int len;
} espnow_rx_packet_t;

// -----------------------------------------------------------------------------
// ESP-NOW CALLBACK (Remote Slave -> Master)
// -----------------------------------------------------------------------------
static void master_espnow_recv_cb(const esp_now_recv_info_t *recv_info, const uint8_t *data, int len) {
    espnow_rx_packet_t packet;
    packet.len = len;
    memcpy(packet.data, data, len);
    
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xQueueSendFromISR(espnow_rx_queue, &packet, &xHigherPriorityTaskWoken);
}

// -----------------------------------------------------------------------------
// RTOS TASKS
// -----------------------------------------------------------------------------

// TASK 1: PC (via USB) -> Master -> (Routes to Slave OR Local Device)
static void pc_usb_router_task(void *pvParameters) {
    uint8_t *dtmp = (uint8_t *)malloc(UART_BUF_SIZE);

    ESP_LOGI(TAG, "PC USB Router Task Started. Listening for 'S:' prefix...");

    while (1) {
        // Block and wait for data from Native USB port (Wait up to 50ms per loop)
        int len = usb_serial_jtag_read_bytes(dtmp, UART_BUF_SIZE, pdMS_TO_TICKS(50));
        
        if (len > 0) {
            // If it has the "S:" prefix, route it to the Remote Slave via ESP-NOW
            if (len >= 2 && strncmp((char *)dtmp, "S:", 2) == 0) {
                int payload_len = len - 2;
                if (payload_len > ESP_NOW_MAX_DATA_LEN) {
                    payload_len = ESP_NOW_MAX_DATA_LEN; // Prevent overflow
                }
                esp_err_t err = esp_now_send(slave_mac, dtmp + 2, payload_len);
                if (err != ESP_OK) ESP_LOGW(TAG, "Slave Send Error: %s", esp_err_to_name(err));
            } 
            // Otherwise, route it directly to the Local Wired Device (UART1)
            else {
                uart_write_bytes(UART_DEVICE, (const char *)dtmp, len);
            }
        }
    }
    free(dtmp);
    vTaskDelete(NULL);
}

// TASK 2: Local Wired Device (UART1) -> Master -> PC (via USB)
static void device_uart_to_pc_task(void *pvParameters) {
    uart_event_t event;
    uint8_t *dtmp = (uint8_t *)malloc(UART_BUF_SIZE);

    while (1) {
        if (xQueueReceive(uart_device_queue, (void *)&event, ESPNOW_MAX_DELAY)) {
            if (event.type == UART_DATA) {
                int len = uart_read_bytes(UART_DEVICE, dtmp, event.size, ESPNOW_MAX_DELAY);
                // Send directly to PC via USB exactly as received (no prefix)
                if (len > 0) {
                    usb_serial_jtag_write_bytes((const char *)dtmp, len, portMAX_DELAY);
                }
                if (strncmp((char *)dtmp, "AC_INIT", 7) == 0) {
                    //Transfer to slave to initialize the AC unit
                    esp_err_t err = esp_now_send(slave_mac, (const uint8_t *)"AC_INIT\r\n", strlen("AC_INIT\r\n"));
                    if (err != ESP_OK) {
                        ESP_LOGW(TAG, "Failed to send AC_INIT to Slave: %s", esp_err_to_name(err));
                    }
                }
                else if (strncmp((char *)dtmp, "AC_DONE", 7) == 0) {
                    //Transfer to slave to indicate AC unit is done
                    esp_err_t err = esp_now_send(slave_mac, (const uint8_t *)"AC_DONE\r\n", strlen("AC_DONE\r\n"));
                    if (err != ESP_OK) {
                        ESP_LOGW(TAG, "Failed to send AC_DONE to Slave: %s", esp_err_to_name(err));
                    }
                }
            }
            else if (event.type == UART_FIFO_OVF || event.type == UART_BUFFER_FULL) {
                uart_flush_input(UART_DEVICE);
                xQueueReset(uart_device_queue);
            }
        }
    }
    free(dtmp);
    vTaskDelete(NULL);
}

// TASK 3: Remote Slave -> Master -> PC (via USB)
static void espnow_to_pc_task(void *pvParameters) {
    espnow_rx_packet_t packet;
    // Buffer to hold the "S:" prefix + the incoming payload
    uint8_t out_buf[ESP_NOW_MAX_DATA_LEN + 2]; 
    out_buf[0] = 'S';
    out_buf[1] = ':';

    while (1) {
        if (xQueueReceive(espnow_rx_queue, &packet, portMAX_DELAY)) {
            // Combine prefix and data so it prints seamlessly on the PC
            memcpy(&out_buf[2], packet.data, packet.len);
            
            // Write to PC via Native USB
            usb_serial_jtag_write_bytes((const char *)out_buf, packet.len + 2, portMAX_DELAY);
            
            if (strncmp((char *)packet.data, "AC_INIT", 7) == 0) {
                // If the Slave sends an AC status update, also forward it to the Local Device
                uart_write_bytes(UART_DEVICE, "\'", 1); // Send translated AC_INIT TO 1 byte message device understands
            }
            else if (strncmp((char *)packet.data, "AC_DONE", 7) == 0) {
                uart_write_bytes(UART_DEVICE, "\"", 1); // Send translated AC_DONE TO 1 byte message device understands
            }
        }
    }
}

// -----------------------------------------------------------------------------
// INITIALIZATION ROUTINES
// -----------------------------------------------------------------------------

static void init_interfaces(void) {
    // 1. Initialize Native USB (Serial/JTAG) for PC
    usb_serial_jtag_driver_config_t usb_config = {
        .rx_buffer_size = UART_BUF_SIZE,
        .tx_buffer_size = UART_BUF_SIZE,
    };
    ESP_ERROR_CHECK(usb_serial_jtag_driver_install(&usb_config));

    // 2. Initialize UART1 for the Local Wired Device
    uart_config_t uart_config = {
        .baud_rate = UART_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    ESP_ERROR_CHECK(uart_driver_install(UART_DEVICE, UART_BUF_SIZE * 2, UART_BUF_SIZE * 2, 20, &uart_device_queue, 0));
    ESP_ERROR_CHECK(uart_param_config(UART_DEVICE, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(UART_DEVICE, UART_DEVICE_TX_PIN, UART_DEVICE_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
}

static void init_wifi_full_range(void) {
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());

    // Enable Long Range (LR) Protocol alongside standard modes
    ESP_ERROR_CHECK(esp_wifi_set_protocol(WIFI_IF_STA, WIFI_PROTOCOL_11B | WIFI_PROTOCOL_11G | WIFI_PROTOCOL_11N | WIFI_PROTOCOL_LR));
    ESP_ERROR_CHECK(esp_wifi_set_channel(13, WIFI_SECOND_CHAN_NONE));
    ESP_ERROR_CHECK(esp_wifi_set_max_tx_power(80));
}

static void init_espnow(void) {
    ESP_ERROR_CHECK(esp_now_init());
    ESP_ERROR_CHECK(esp_now_register_recv_cb(master_espnow_recv_cb));

    // Define the Long-Range Rate Configuration for the ESP32-C6
    esp_now_rate_config_t lr_rate_cfg = {
        .phymode = WIFI_PHY_MODE_LR,
        .rate = WIFI_PHY_RATE_LORA_500K,
        .ersu = false,
        .dcm = false
    };

    // Register the single Slave
    esp_now_peer_info_t peer = {};
    peer.channel = 13; 
    peer.ifidx = WIFI_IF_STA;
    peer.encrypt = false;
    memcpy(peer.peer_addr, slave_mac, ESP_NOW_ETH_ALEN);
    
    ESP_ERROR_CHECK(esp_now_add_peer(&peer));
    
    // Force Slave connection to use Long-Range PHY
    ESP_ERROR_CHECK(esp_now_set_peer_rate_config(peer.peer_addr, &lr_rate_cfg));
}

// -----------------------------------------------------------------------------
// MAIN ENTRY POINT
// -----------------------------------------------------------------------------

void app_main(void) {
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    init_interfaces();
    init_wifi_full_range();
    init_espnow();

    espnow_rx_queue = xQueueCreate(10, sizeof(espnow_rx_packet_t));
    
    // Start all three routing tasks
    xTaskCreate(pc_usb_router_task, "pc_router", 4096, NULL, 5, NULL);
    xTaskCreate(device_uart_to_pc_task, "dev_to_pc", 4096, NULL, 5, NULL);
    xTaskCreate(espnow_to_pc_task, "esp_to_pc", 4096, NULL, 5, NULL);
    
    ESP_LOGI(TAG, "Master Hub Initialization Complete. Routing active.");
}