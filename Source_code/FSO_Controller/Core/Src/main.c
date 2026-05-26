/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define STEPX_Port D2_GPIO_Port
#define STEPX_Pin  D2_Pin
#define STEPY_Port D3_GPIO_Port
#define STEPY_Pin  D3_Pin
#define STEPZ_Port D4_GPIO_Port
#define STEPZ_Pin  D4_Pin

#define DIRX_Port  D5_GPIO_Port
#define DIRX_Pin   D5_Pin
#define DIRY_Port  D6_GPIO_Port
#define DIRY_Pin   D6_Pin
#define DIRZ_Port  D7_GPIO_Port
#define DIRZ_Pin   D7_Pin

#define EN_Port    D8_GPIO_Port
#define EN_Pin     D8_Pin

#define LIMIT_X_Port D9_GPIO_Port
#define LIMIT_X_Pin  D9_Pin
#define LIMIT_Y_Port D10_GPIO_Port
#define LIMIT_Y_Pin  D10_Pin
#define LIMIT_Z_Port D11_GPIO_Port
#define LIMIT_Z_Pin  D11_Pin

#define SFP_ADDR_INFO  (0x50 << 1) // 0xA0
#define SFP_ADDR_DDM   (0x51 << 1) // 0xA2

#define BLACK 1
#if BLACK == 1 //BLACK
#define A_T_D 5748
#define D_T_A (3164 - 300)

#define W_T_X (4000)
#define X_T_W 3376

#define E_T_R 19887

#elif BLACK == 0 //COLOR

#define A_T_D 5758
#define D_T_A 2384

#define W_T_X 4541
#define X_T_W 2798

#define E_T_R 20564

#endif

#define SPIRAL_INCREMENT 25
#define SPIRAL_INCREMENT_BROAD 200

typedef enum{
	to_e,
	home_x,
	to_d,
	home_y,
	to_w,
	home_z,
	idle
} homing_states;

typedef enum{
	GO_D,
	GO_W,
	GO_A,
	GO_X,
	IDLE
} spiral_dir;

typedef enum{
	opt_idle,
	opt_z,
	opt_y,
	opt_x
} opt_states;

struct{
	spiral_dir state;
	int amount;
	int increment;
} spiral;


struct{
	opt_states optimize_state_machine;
	int dir;
	int step_size;
} opt;



homing_states homing_state_machine = idle;
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
// State variables
bool xMoving = false, yMoving = false, zMoving = false;
bool motorsEnabled = true;
uint16_t stepsRemainingX = 0;
uint16_t stepsRemainingY = 0;
uint16_t stepsRemainingZ = 0;
uint16_t activeStepSize = 200;
uint16_t steps_bef_resetY = 0;
uint16_t steps_bef_resetZ = 0;
uint16_t steps_bef_resetX = 0;


char buf[128];

uint8_t ddm_data[128]; // Buffer for DDM data

uint16_t output_pow_tm = 300;
uint32_t timestamp;

uint16_t rx_raw_max_dir = 0;
uint16_t rx_raw_max = 0;
bool first_round = false;
bool second_round = false;
bool finish_on_max = false;
bool fresh_swept = false;

bool axis_homed = false;
// Latch variables
int blockedDirX = -1;
int blockedDirY = -1;
int blockedDirZ = -1;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void Serial_Print(const char* str) {
    HAL_UART_Transmit(&huart2, (uint8_t*)str, strlen(str), HAL_MAX_DELAY);
}

void Serial_Println(const char* str) {
    Serial_Print(str);
    Serial_Print("\r\n");
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
  HAL_GPIO_WritePin(EN_Port, EN_Pin, GPIO_PIN_RESET);
  Serial_Println("--- Dual Axis Limit Controller (Y & Z) ---");
  spiral.amount = 0;
  spiral.state = IDLE;
  bool pause = false;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	output_pow_tm = 300;
	// 1. Check physical switch states (Assumes limits have Pull-Ups configured in CubeMX)
	bool isXPressed = (HAL_GPIO_ReadPin(LIMIT_X_Port, LIMIT_X_Pin) == GPIO_PIN_RESET);
	bool isYPressed = (HAL_GPIO_ReadPin(LIMIT_Y_Port, LIMIT_Y_Pin) == GPIO_PIN_RESET);
	bool isZPressed = (HAL_GPIO_ReadPin(LIMIT_Z_Port, LIMIT_Z_Pin) == GPIO_PIN_RESET);
	if(!pause){


		//Cool down the motors!
		if(stepsRemainingX | stepsRemainingY | stepsRemainingZ)
		  HAL_GPIO_WritePin(EN_Port, EN_Pin, GPIO_PIN_RESET);
		else
			HAL_GPIO_WritePin(EN_Port, EN_Pin, GPIO_PIN_SET);

		// 2. Handle X Latching Logic
		if (isXPressed) {
		  if (xMoving && blockedDirX == -1) {
			sprintf(buf, "X:%d Y:%d Z:%d\r\n", steps_bef_resetX, steps_bef_resetY, steps_bef_resetZ);
			Serial_Print(buf);
			steps_bef_resetY = 0;
			steps_bef_resetX = 0;
			steps_bef_resetZ = 0;
			blockedDirX = HAL_GPIO_ReadPin(DIRX_Port, DIRX_Pin);
			xMoving = false;
			stepsRemainingX = 0;


			//HOMING
			if(homing_state_machine == to_e) { homing_state_machine = home_x; axis_homed = false; }

			Serial_Print("!! X LIMIT HIT !! Blocked: ");
			Serial_Println(blockedDirX == GPIO_PIN_SET ? "HIGH (d)" : "LOW (a)");
		  }
		} else {
		  blockedDirX = -1;
		}
		// 2. Handle Y Latching Logic
		if (isYPressed) {
		  if (yMoving && blockedDirY == -1) {
			sprintf(buf, "X:%d Y:%d Z:%d\r\n", steps_bef_resetX, steps_bef_resetY, steps_bef_resetZ);
			Serial_Print(buf);
			steps_bef_resetY = 0;
			steps_bef_resetX = 0;
			steps_bef_resetZ = 0;
			blockedDirY = HAL_GPIO_ReadPin(DIRY_Port, DIRY_Pin);
			yMoving = false;
			stepsRemainingY = 0;
			//HOMING
			if(homing_state_machine == to_d) { homing_state_machine = home_y; axis_homed = false; }

			//SPIRAL
			if(spiral.state != IDLE) { spiral.state = IDLE; }

			Serial_Print("!! Y LIMIT HIT !! Blocked: ");
			Serial_Println(blockedDirY == GPIO_PIN_SET ? "HIGH (d)" : "LOW (a)");
		  }
		} else {
		  blockedDirY = -1;
		}

		// 3. Handle Z Latching Logic
		if (isZPressed) {
		  if (zMoving && blockedDirZ == -1) {
			sprintf(buf, "X:%d Y:%d Z:%d\r\n", steps_bef_resetX, steps_bef_resetY, steps_bef_resetZ);
			Serial_Print(buf);
			steps_bef_resetY = 0;
			steps_bef_resetX = 0;
			steps_bef_resetZ = 0;
			blockedDirZ = HAL_GPIO_ReadPin(DIRZ_Port, DIRZ_Pin);
			zMoving = false;
			stepsRemainingZ = 0;
			//HOMING
			if(homing_state_machine == to_w) { homing_state_machine = home_z; axis_homed = false; }

			//SPIRAL
			if(spiral.state != IDLE) spiral.state = IDLE;

			Serial_Print("!! Z LIMIT HIT !! Blocked: ");
			Serial_Println(blockedDirZ == GPIO_PIN_SET ? "HIGH (x)" : "LOW (w)");
		  }
		} else {
		  blockedDirZ = -1;
		}

		// --- MOTOR PULSING ---
		// X Safety Check
		if (xMoving) {
		  if (isXPressed && HAL_GPIO_ReadPin(DIRX_Port, DIRX_Pin) == blockedDirX) xMoving = false;
		  else { HAL_GPIO_WritePin(STEPX_Port, STEPX_Pin, GPIO_PIN_SET); ++steps_bef_resetX; }
		}

		// Y Safety Check
		if (yMoving) {
		  if (isYPressed && HAL_GPIO_ReadPin(DIRY_Port, DIRY_Pin) == blockedDirY) yMoving = false;
		  else { HAL_GPIO_WritePin(STEPY_Port, STEPY_Pin, GPIO_PIN_SET); ++steps_bef_resetY; }
		}

		// Z Safety Check
		if (zMoving) {
		  if (isZPressed && HAL_GPIO_ReadPin(DIRZ_Port, DIRZ_Pin) == blockedDirZ) zMoving = false;
		  else { HAL_GPIO_WritePin(STEPZ_Port, STEPZ_Pin, GPIO_PIN_SET);  ++steps_bef_resetZ; }
		}

		// Delay 1ms (equivalent to delayMicroseconds(1000))
		HAL_Delay(2);

		HAL_GPIO_WritePin(STEPX_Port, STEPX_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(STEPY_Port, STEPY_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(STEPZ_Port, STEPZ_Pin, GPIO_PIN_RESET);

		if (stepsRemainingX > 0) {
		  stepsRemainingX--;
		} else {
		  xMoving = false;
		}
		if (stepsRemainingY > 0) {
		  stepsRemainingY--;
		} else {
		  yMoving = false;
		}
		if (stepsRemainingZ > 0) {
		  stepsRemainingZ--;
		} else {
		  zMoving = false;
		}

		//spiral state machine
		if(spiral.state != IDLE){
			output_pow_tm = 50;
			if(spiral.state == GO_D && !stepsRemainingY && !stepsRemainingZ){
				HAL_GPIO_WritePin(DIRY_Port, DIRY_Pin, GPIO_PIN_SET);
				yMoving = true;
				stepsRemainingY = (++spiral.amount) * spiral.increment;
				spiral.state = GO_W;
			}
			else if(spiral.state == GO_W && !stepsRemainingY && !stepsRemainingZ){
				HAL_GPIO_WritePin(DIRZ_Port, DIRZ_Pin, GPIO_PIN_RESET);
				zMoving = true;
				stepsRemainingZ = spiral.amount * spiral.increment;
				spiral.state = GO_A;
			}
			else if(spiral.state == GO_A && !stepsRemainingY && !stepsRemainingZ){
				HAL_GPIO_WritePin(DIRY_Port, DIRY_Pin, GPIO_PIN_RESET);
				yMoving = true;
				stepsRemainingY = (++spiral.amount) * spiral.increment;
				spiral.state = GO_X;

			}
			else if(spiral.state == GO_X && !stepsRemainingY && !stepsRemainingZ){
				HAL_GPIO_WritePin(DIRZ_Port, DIRZ_Pin, GPIO_PIN_SET);
				zMoving = true;
				stepsRemainingZ = spiral.amount * spiral.increment;
				spiral.state = GO_D;

			}

		}

		//state machine for homing
		if(homing_state_machine != idle && !stepsRemainingX && !stepsRemainingY && !stepsRemainingZ){
			switch(homing_state_machine){
			case to_e:
				HAL_GPIO_WritePin(DIRX_Port, DIRX_Pin, GPIO_PIN_SET); xMoving = true; stepsRemainingX = 60000;
				break;

			case home_x:
				if(!axis_homed) { HAL_GPIO_WritePin(DIRX_Port, DIRX_Pin, GPIO_PIN_RESET); xMoving = true; stepsRemainingX = E_T_R; axis_homed = true; }
				else homing_state_machine = idle;
				break;

			case to_d:
				HAL_GPIO_WritePin(DIRY_Port, DIRY_Pin, GPIO_PIN_SET); yMoving = true; stepsRemainingY = 15000;
				break;

			case home_y:
				if(!axis_homed) { HAL_GPIO_WritePin(DIRY_Port, DIRY_Pin, GPIO_PIN_RESET); yMoving = true; stepsRemainingY = D_T_A; axis_homed = true; }
				else homing_state_machine = to_w;
				break;

			case to_w:
				HAL_GPIO_WritePin(DIRZ_Port, DIRZ_Pin, GPIO_PIN_RESET); zMoving = true; stepsRemainingZ = 15000;
				break;

			case home_z:
				if(!axis_homed) { HAL_GPIO_WritePin(DIRZ_Port, DIRZ_Pin, GPIO_PIN_SET); zMoving = true; stepsRemainingZ = W_T_X; axis_homed = true; }
				else homing_state_machine = idle;
				break;
			default: break;


			}
		}

	}


	uint8_t c;
	// --- SERIAL COMMS ---
	// Non-blocking UART receive (timeout = 0)
	if (HAL_UART_Receive(&huart2, &c, 1, 0) == HAL_OK) {

	  switch (c) {

		case 'G':
		  opt.optimize_state_machine = opt_z;
		  opt.step_size = 10;
		  opt.dir = 0;
			rx_raw_max = 0;
			rx_raw_max_dir = 0;
			first_round = false;
			second_round = false;
			finish_on_max = false;
		  break;
		case 'g':
		  opt.optimize_state_machine = opt_z;
		  opt.step_size = 1;
		  opt.dir = 0;
			rx_raw_max = 0;
			rx_raw_max_dir = 0;
			first_round = false;
			finish_on_max = false;
			fresh_swept = false;
		  break;

		case 'h':
		  homing_state_machine = to_d;
		  break;
		case 'k':
		  homing_state_machine = to_e;
		  break;
		case 'j':
		  spiral.state = GO_D;
		  spiral.amount = 0;
		  spiral.increment = SPIRAL_INCREMENT;
		  break;

		case 'J':
		  spiral.state = GO_D;
		  spiral.amount = 0;
		  spiral.increment = SPIRAL_INCREMENT_BROAD;
		  break;
		case 'p':
		  pause = !pause;
		  if(pause == true) HAL_GPIO_WritePin(EN_Port, EN_Pin, GPIO_PIN_SET);
		  else{
				if(stepsRemainingX | stepsRemainingY | stepsRemainingZ)
				  HAL_GPIO_WritePin(EN_Port, EN_Pin, GPIO_PIN_RESET);
				HAL_Delay(5);
		  }

		  break;

		// Z-Axis
		case 'e':
		  if (isXPressed && blockedDirX == GPIO_PIN_SET) Serial_Println("X Blocked!");
		  else { HAL_GPIO_WritePin(DIRX_Port, DIRX_Pin, GPIO_PIN_SET); xMoving = true; stepsRemainingX = activeStepSize; }
		  break;
		case 'r':
		  if (isXPressed && blockedDirX == GPIO_PIN_RESET) Serial_Println("X Blocked!");
		  else { HAL_GPIO_WritePin(DIRX_Port, DIRX_Pin, GPIO_PIN_RESET); xMoving = true; stepsRemainingX = activeStepSize; }
		  break;

		// Y-Axis
		case 'd':
		  if (isYPressed && blockedDirY == GPIO_PIN_SET) Serial_Println("Y Blocked!");
		  else { HAL_GPIO_WritePin(DIRY_Port, DIRY_Pin, GPIO_PIN_SET); yMoving = true; stepsRemainingY = activeStepSize; }
		  break;
		case 'a':
		  if (isYPressed && blockedDirY == GPIO_PIN_RESET) Serial_Println("Y Blocked!");
		  else { HAL_GPIO_WritePin(DIRY_Port, DIRY_Pin, GPIO_PIN_RESET); yMoving = true; stepsRemainingY = activeStepSize; }
		  break;

		// Z-Axis
		case 'x':
		  if (isZPressed && blockedDirZ == GPIO_PIN_SET) Serial_Println("Z Blocked!");
		  else { HAL_GPIO_WritePin(DIRZ_Port, DIRZ_Pin, GPIO_PIN_SET); zMoving = true; stepsRemainingZ = activeStepSize; }
		  break;
		case 'w':
		  if (isZPressed && blockedDirZ == GPIO_PIN_RESET) Serial_Println("Z Blocked!");
		  else { HAL_GPIO_WritePin(DIRZ_Port, DIRZ_Pin, GPIO_PIN_RESET); zMoving = true; stepsRemainingZ = activeStepSize; }
		  break;

		// Controls
		case 's':
		  activeStepSize = (activeStepSize == 200) ? 100 : (activeStepSize == 100) ? 10 : (activeStepSize == 10) ? 1 : 200;
		  snprintf(buf, sizeof(buf), "Step size: %u\r\n", activeStepSize);
		  Serial_Print(buf);
		  break;
		case ' ': xMoving = yMoving = zMoving = false; stepsRemainingX = 0; stepsRemainingY = 0; stepsRemainingZ = 0;
		 spiral.state = IDLE; homing_state_machine = idle;
		 opt.optimize_state_machine = opt_idle;


		snprintf(buf, sizeof(buf), "steps_bef_resetY: %u, steps_bef_resetZ: %u, steps_bef_resetX: %u\r\n", steps_bef_resetY, steps_bef_resetZ, steps_bef_resetX);
		  Serial_Print(buf);
		break;
	  }
	}

	//READOUT
	if(HAL_GetTick() - timestamp > output_pow_tm || (opt.optimize_state_machine != opt_idle && !stepsRemainingX && !stepsRemainingY && !stepsRemainingZ)){

		if (HAL_I2C_Mem_Read(&hi2c1, SFP_ADDR_DDM, 96, I2C_MEMADD_SIZE_8BIT, &ddm_data[96], 22,100) == HAL_OK)
		  {
		    char buf[32];
			// --- RX POWER
			// Offset 104-105. Units are 0.1 uW
			uint16_t rx_raw = ((ddm_data[104] << 8) | ddm_data[105]) - 1;

			// Integer division gives the whole number, modulo gives the remainder
			uint16_t whole_uw = rx_raw / 10;
			uint16_t decimal_uw = rx_raw % 10;

			if(spiral.state != IDLE && rx_raw > 0 && rx_raw < 100) {spiral.state = IDLE; }

			// Outputs: "Power: 123.4 uW"
			snprintf(buf, sizeof(buf), "Power: %u.%u uW\r\n", whole_uw, decimal_uw);
			Serial_Print(buf);

//
//			float rx_uw = rx_raw * 0.0001f;
//
//			if(rx_uw > 0) {
//				rx_power_dbm = 10.0f * log10f(rx_uw);
//			} else {
//				rx_power_dbm = -40.0f; // Sensitivity floor
//			}
//			if(spiral.state != IDLE && rx_power_dbm > -39.99f && rx_power_dbm < -30.00f) {spiral.state = IDLE; }
//			snprintf(buf, sizeof(buf), "Power: %f dbm\r\n", rx_power_dbm);
//			Serial_Print(buf);
			timestamp = HAL_GetTick();

			if(opt.optimize_state_machine != opt_idle){
				if(finish_on_max && rx_raw_max <= rx_raw){
					switch(opt.optimize_state_machine){
					case opt_z: opt.optimize_state_machine = opt_y; break;
					case opt_y: opt.optimize_state_machine = opt_idle; break;
					default: break;
					}
					rx_raw_max = 0;
					rx_raw_max_dir = 0;
					first_round = false;
					second_round = false;
					finish_on_max = false;
					continue; // GO TO NEXT WHILE(1)
				}
				if(rx_raw_max < rx_raw) {rx_raw_max = rx_raw; rx_raw_max_dir = opt.dir; fresh_swept = true;}
				else if(rx_raw == 0 && fresh_swept){
					opt.dir = !opt.dir;
					fresh_swept = false;
					if(!first_round) first_round = true;
					else if(first_round && !second_round) second_round = true;
					else finish_on_max = true;
				}
				else if(rx_raw > 10) fresh_swept = true;

				switch(opt.optimize_state_machine){
					case opt_z: HAL_GPIO_WritePin(DIRZ_Port, DIRZ_Pin, opt.dir); zMoving = true; stepsRemainingZ = opt.step_size; break;
					case opt_y: HAL_GPIO_WritePin(DIRY_Port, DIRY_Pin, opt.dir); yMoving = true; stepsRemainingY = opt.step_size; break;
					default: break;
				}
				HAL_Delay(50);
			}

		  }

	}


    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, D6_Pin|D3_Pin|D5_Pin|D4_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, D7_Pin|D8_Pin|D2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : D11_Pin */
  GPIO_InitStruct.Pin = D11_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(D11_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : D6_Pin D3_Pin D5_Pin D4_Pin */
  GPIO_InitStruct.Pin = D6_Pin|D3_Pin|D5_Pin|D4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : D9_Pin */
  GPIO_InitStruct.Pin = D9_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(D9_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : D7_Pin D8_Pin D2_Pin */
  GPIO_InitStruct.Pin = D7_Pin|D8_Pin|D2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : D10_Pin */
  GPIO_InitStruct.Pin = D10_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(D10_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
