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

// -----------------------------------------------------------------------------
// CONFIGURATION
// -----------------------------------------------------------------------------
#define UART_NUM           UART_NUM_1
#define UART_BAUD_RATE     115200
#define UART_BUF_SIZE      1024
#define ESPNOW_MAX_DELAY   portMAX_DELAY


// GPIO Pins for the Local Device (UART1)
#define UART_DEVICE_TX_PIN 6
#define UART_DEVICE_RX_PIN 7

static const char *TAG = "SLAVE_NODE";
static QueueHandle_t uart0_queue;

// TODO: Replace with the actual hardware MAC address of your ESP32-C6 Master
uint8_t master_mac[ESP_NOW_ETH_ALEN] = {0xF0, 0xF5, 0xBD, 0x0B, 0xF6, 0xA8};
uint8_t buff[] = "I HEAR YOU MASTER!\n";

// Add this under your uart0_queue declaration
static QueueHandle_t espnow_rx_queue;

// Define a structure to hold the incoming data
typedef struct {
    uint8_t data[ESP_NOW_MAX_DATA_LEN];
    int len;
} espnow_rx_packet_t;

// -----------------------------------------------------------------------------
// ESP-NOW CALLBACK (LIGHTNING FAST NOW)
// -----------------------------------------------------------------------------
static void slave_espnow_recv_cb(const esp_now_recv_info_t *recv_info, const uint8_t *data, int len) {
    espnow_rx_packet_t packet;
    packet.len = len;
    memcpy(packet.data, data, len);

    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xQueueSendFromISR(espnow_rx_queue, &packet, &xHigherPriorityTaskWoken);
}

// -----------------------------------------------------------------------------
// RTOS TASKS
// -----------------------------------------------------------------------------

// NEW TASK: Processes incoming RF data and writes to UART safely
static void espnow_to_uart_task(void *pvParameters) {
    espnow_rx_packet_t packet;
    while (1) {
        // Wait for data to arrive from the RF callback
        if (xQueueReceive(espnow_rx_queue, &packet, portMAX_DELAY)) {
            // Write to UART safely outside the Wi-Fi task
            if (strncmp((char *)packet.data, "AC_INIT", 7) == 0) {
                // If the Slave sends an AC status update, also forward it to the Local Device
                uart_write_bytes(UART_NUM, "\'", 1); // Send translated AC_INIT TO 1 byte message device understands
            }
            else if (strncmp((char *)packet.data, "AC_DONE", 7) == 0) {
                uart_write_bytes(UART_NUM, "\"", 1); // Send translated AC_DONE TO 1 byte message device understands
            }
            else {
                uart_write_bytes(UART_NUM, (const char *)packet.data, packet.len);
            }
        }
    }
}


// Listens to the external device over UART, parses by line, and forwards to Master
static void uart_forwarding_task(void *pvParameters) {
    uart_event_t event;
    
    // Buffer to build a single clean text line
    uint8_t line_buf[ESP_NOW_MAX_DATA_LEN];
    int line_len = 0;
    
    // Temporary buffer to quickly pull raw bytes from the UART FIFO
    uint8_t rx_buf[128]; 

    ESP_LOGI(TAG, "UART Forwarding Task Started. Listening for external data...");

    while (1) {
        // Block until a UART event occurs
        if (xQueueReceive(uart0_queue, (void *)&event, portMAX_DELAY)) {
            if (event.type == UART_DATA) {
                size_t buffered_size;
                uart_get_buffered_data_len(UART_NUM, &buffered_size);
                
                // Drain the UART FIFO completely
                while (buffered_size > 0) {
                    // Read whatever is instantly available (0 delay)
                    int to_read = (buffered_size > sizeof(rx_buf)) ? sizeof(rx_buf) : buffered_size;
                    int rx_len = uart_read_bytes(UART_NUM, rx_buf, to_read, 0);
                    
                    for (int i = 0; i < rx_len; i++) {
                        line_buf[line_len++] = rx_buf[i];
                        
                        // IF we hit a newline (\n) OR the buffer hits the 250-byte ESP-NOW limit:
                        // Fire the packet immediately!
                        if (rx_buf[i] == '\n' || line_len >= ESP_NOW_MAX_DATA_LEN) {
                            esp_err_t err = esp_now_send(master_mac, line_buf, line_len);
                            if (err != ESP_OK) {
                                ESP_LOGW(TAG, "Send to Master failed: %s", esp_err_to_name(err));
                            }
                            
                            // Reset the line length counter for the next sentence
                            line_len = 0; 
                            
                            // Give the Wi-Fi radio 2 milliseconds to breathe so it 
                            // doesn't drop packets if the machine spits out 50 lines at once.
                            vTaskDelay(pdMS_TO_TICKS(2)); 
                        }
                    }
                    uart_get_buffered_data_len(UART_NUM, &buffered_size);
                }
            }
            // Flush the buffer if it gets overloaded
            else if (event.type == UART_FIFO_OVF || event.type == UART_BUFFER_FULL) {
                uart_flush_input(UART_NUM);
                xQueueReset(uart0_queue);
                line_len = 0; // Wipe the broken line
            }
        }
    }
    vTaskDelete(NULL);
}

// -----------------------------------------------------------------------------
// INITIALIZATION ROUTINES
// -----------------------------------------------------------------------------

static void init_uart(void) {
    uart_config_t uart_config = {
        .baud_rate = UART_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    ESP_ERROR_CHECK(uart_driver_install(UART_NUM, UART_BUF_SIZE * 2, UART_BUF_SIZE * 2, 20, &uart0_queue, 0));
    ESP_ERROR_CHECK(uart_param_config(UART_NUM, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(UART_NUM, UART_DEVICE_TX_PIN, UART_DEVICE_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
}

static void init_wifi_full_range(void) {
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());

    // Enable Long Range Protocol
    ESP_ERROR_CHECK(esp_wifi_set_protocol(WIFI_IF_STA, WIFI_PROTOCOL_11B | WIFI_PROTOCOL_11G | WIFI_PROTOCOL_11N | WIFI_PROTOCOL_LR));
    
    // Stay on quiet channel
    ESP_ERROR_CHECK(esp_wifi_set_channel(13, WIFI_SECOND_CHAN_NONE));
    
    // Maximum TX Power
    ESP_ERROR_CHECK(esp_wifi_set_max_tx_power(80));
}

static void init_espnow(void) {
    ESP_ERROR_CHECK(esp_now_init());
    ESP_ERROR_CHECK(esp_now_register_recv_cb(slave_espnow_recv_cb));

    // Register the Master
    esp_now_peer_info_t peer = {};
    peer.channel = 13; 
    peer.ifidx = WIFI_IF_STA;
    peer.encrypt = false;
    memcpy(peer.peer_addr, master_mac, ESP_NOW_ETH_ALEN);
    ESP_ERROR_CHECK(esp_now_add_peer(&peer));

    // Force the connection to the Master to use Long-Range PHY
    esp_now_rate_config_t lr_rate_cfg = {
        .phymode = WIFI_PHY_MODE_LR,
        .rate = WIFI_PHY_RATE_LORA_500K,
        .ersu = false,
        .dcm = false
    };
    ESP_ERROR_CHECK(esp_now_set_peer_rate_config(peer.peer_addr, &lr_rate_cfg));
}
// -----------------------------------------------------------------------------
// MAIN ENTRY POINT
// -----------------------------------------------------------------------------

void app_main(void) {
    // 1. Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // 2. Initialize Hardware & Protocols
    init_uart();
    init_wifi_full_range();
    init_espnow();

    espnow_rx_queue = xQueueCreate(10, sizeof(espnow_rx_packet_t));
    
    // Start both tasks
    xTaskCreate(uart_forwarding_task, "uart_forwarding_task", 4096, NULL, 5, NULL);
    xTaskCreate(espnow_to_uart_task, "espnow_to_uart_task", 4096, NULL, 5, NULL);
    
    ESP_LOGI(TAG, "Slave Node Initialization Complete. Bridging UART to Master...");
}