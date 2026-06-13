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


// =========================================================
// Active Compensation Configuration
// =========================================================

// Timing & Iterations
#define AC_TIMEOUT_MS          30000   // Comm timeout and IDLE reset interval (ms)
#define AC_RETRANSMIT_MS       5000   // Comm timeout and IDLE reset interval (ms)
#define AC_RUN_ITERATIONS      30     // Number of step iterations per compensation run
#define AC_RUN_HISTORY	       1     // Number of step iterations per compensation run
#define AC_TICKS_PER_STEP      9     // Super loop cycles to wait between motor evaluations
#define AC_RUN_DIVIDER         5     // Super loop cycles to wait between motor evaluations

// Math & Thresholds
#define AC_THRESH_NUM          4      // Threshold multiplier
#define AC_THRESH_DEN          5      // Threshold divisor-> 4/5 = 80%
#define AC_EMA_PERIOD          32      // Moving average period/weighting factor, ~ 30 samples SMA


#define ADC_REFRESH_TIME    15     // Default output power time interval

typedef enum{
	TO_E,
	HOME_X,
	TO_D,
	HOME_Y,
	TO_W,
	HOME_Z,
	HO_IDLE
} homing_states;

typedef enum{
	GO_D,
	GO_W,
	GO_A,
	GO_X,
	IDLE
} spiral_dir;

typedef enum{
	OPT_IDLE,
	OPT_Z,
	OPT_Y,
	OPT_X
} opt_states;

//AC - active compensation
typedef enum{
	AC_OFF, // NO ac at all
	AC_IDLE, // ac turn on, monitor power and see what happens, if power drops below treshold, change to ac_rdy_to_run and send to esp to signal
	AC_RUN, // run ac algorithm
	AC_WAIT_TO_SAVE, //ac wait for another device to save the pavg
	AC_PAVG_MEAS //measure pavg
} ac_states;

struct ac_struct{
	ac_states state;
	uint8_t steps;
	uint8_t pavg_num;
	uint8_t SMA_num;
	uint8_t EMA_num;
	uint32_t mov_avg;
	int pmax;
	uint8_t iter;
	bool dir;
	bool dirz, diry;
	bool me_init;
	uint32_t mov_avg_arr[AC_RUN_HISTORY];
	uint8_t dir_change_count[2];
} ac;

struct spiral_struct{
	spiral_dir state;
	int amount;
	int increment;
} spiral;


struct opt_struct{
	opt_states optimize_state_machine;
	int dir;
	int step_size;
} opt;



homing_states homing_state_machine = HO_IDLE;
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
// State variables

bool pause = false;
bool xMoving = false, yMoving = false, zMoving = false;
bool motorsEnabled = true;
uint16_t stepsRemainingX = 0;
uint16_t stepsRemainingY = 0;
uint16_t stepsRemainingZ = 0;
uint16_t activeStepSize = 200;
uint16_t steps_bef_resetY = 0;
uint16_t steps_bef_resetZ = 0;
uint16_t steps_bef_resetX = 0;
bool isXPressed;
bool isYPressed;
bool isZPressed;


char buf[128];

uint8_t ddm_data[128]; // Buffer for DDM data
uint16_t rx_raw;

uint16_t output_pow_tm = 300;
uint32_t timestamp;

uint16_t rx_raw_max_dir = 0;
uint16_t rx_raw_max = 0;
bool end_on_max = false;
bool histeresis_hit = false;
bool first_sweep = false;
bool second_sweep = false;

bool skip_end_sw = false;
bool axis_homed = false;
// Latch variables
int blockedDirX = -1;
int blockedDirY = -1;
int blockedDirZ = -1;

bool ddm_benchmark = false;
uint16_t ddm_benchmark_i = 0;
uint32_t ddm_benchmark_timestamp = 0;
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
void delay_half_ms(void) {
    // 8,000 iterations * ~4 cycles/iteration = ~32,000 cycles (0.5ms at 64MHz)
    for (volatile uint32_t i = 0; i < 4000; i++) {
        __asm("NOP"); // No Operation: wastes 1 cycle to keep timing predictable
    }
}
void Serial_Print(const char* str) {
    HAL_UART_Transmit(&huart2, (uint8_t*)str, strlen(str), HAL_MAX_DELAY);
}

void Serial_Println(const char* str) {
    Serial_Print(str);
    Serial_Print("\r\n");
}

void Restart_States(void){
	//OPTIMIZE RESTART
	opt.optimize_state_machine = OPT_IDLE;
	opt.dir = 0;
	rx_raw_max = 0;
	rx_raw_max_dir = 0;
	end_on_max = false;
	histeresis_hit = false;
	first_sweep = false;
	second_sweep = false;

	//SPIRAL
	spiral.state = IDLE;
	spiral.amount = 0;
	spiral.increment = 0;

	homing_state_machine = HO_IDLE;
	skip_end_sw = false;

	//MOVEMENT

	stepsRemainingX = 0;
	stepsRemainingY = 0;
	stepsRemainingZ = 0;

	steps_bef_resetY = 0;
	steps_bef_resetX = 0;
	steps_bef_resetZ = 0;

	//AC RESTART
	ac.state = AC_OFF;
	ac.mov_avg = 0;
	ac.pmax = 0;

	output_pow_tm = 300;

}

void Handle_Movement(void){
	// 1. Check physical switch states (Assumes limits have Pull-Ups configured in CubeMX)
	if(!skip_end_sw){
		isXPressed = (HAL_GPIO_ReadPin(LIMIT_X_Port, LIMIT_X_Pin) == GPIO_PIN_RESET);
		isYPressed = (HAL_GPIO_ReadPin(LIMIT_Y_Port, LIMIT_Y_Pin) == GPIO_PIN_RESET);
		isZPressed = (HAL_GPIO_ReadPin(LIMIT_Z_Port, LIMIT_Z_Pin) == GPIO_PIN_RESET);
	}
	else{
		isXPressed = false;
		isYPressed = false;
		isZPressed = false;

	}
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
		blockedDirX = HAL_GPIO_ReadPin(DIRX_Port, DIRX_Pin);
		xMoving = false;
		stepsRemainingX = 0;
		//HOMING
		if(homing_state_machine == TO_E) { homing_state_machine = HOME_X; axis_homed = false; }
		else Restart_States();

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
		blockedDirY = HAL_GPIO_ReadPin(DIRY_Port, DIRY_Pin);
		yMoving = false;
		stepsRemainingY = 0;
		//HOMING
		if(homing_state_machine == TO_D) { homing_state_machine = HOME_Y; axis_homed = false; }
		else Restart_States();

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
		blockedDirZ = HAL_GPIO_ReadPin(DIRZ_Port, DIRZ_Pin);
		zMoving = false;
		stepsRemainingZ = 0;
		//HOMING
		if(homing_state_machine == TO_W) { homing_state_machine = HOME_Z; axis_homed = false; }
		else Restart_States();

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

//	HAL_Delay(1);
	delay_half_ms();

	//Wait for vibrations to stop
	if(ac.state == AC_RUN && !stepsRemainingY && !stepsRemainingZ){
		HAL_Delay(50);
	}

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
}

void Spiral_State_Machine(void){
	//spiral state machine
	if(spiral.state == IDLE) return;
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

void Homing_State_Machine(void){
	//state machine for homing
	if(!(homing_state_machine != HO_IDLE && !stepsRemainingX && !stepsRemainingY && !stepsRemainingZ)) return;
	switch(homing_state_machine){
	case TO_E:
		HAL_GPIO_WritePin(DIRX_Port, DIRX_Pin, GPIO_PIN_SET); xMoving = true; stepsRemainingX = 60000; skip_end_sw = false;
		break;

	case HOME_X:
		if(!axis_homed) { HAL_GPIO_WritePin(DIRX_Port, DIRX_Pin, GPIO_PIN_RESET); xMoving = true; stepsRemainingX = E_T_R; axis_homed = true; skip_end_sw = true; }
		else { Restart_States(); homing_state_machine = HO_IDLE; }
		break;

	case TO_D:
		HAL_GPIO_WritePin(DIRY_Port, DIRY_Pin, GPIO_PIN_SET); yMoving = true; stepsRemainingY = 15000; skip_end_sw = false;
		break;

	case HOME_Y:
		if(!axis_homed) { HAL_GPIO_WritePin(DIRY_Port, DIRY_Pin, GPIO_PIN_RESET); yMoving = true; stepsRemainingY = D_T_A; axis_homed = true; skip_end_sw = true;}
		else homing_state_machine = TO_W;
		break;

	case TO_W:
		HAL_GPIO_WritePin(DIRZ_Port, DIRZ_Pin, GPIO_PIN_RESET); zMoving = true; stepsRemainingZ = 15000; skip_end_sw = false;
		break;

	case HOME_Z:
		if(!axis_homed) { HAL_GPIO_WritePin(DIRZ_Port, DIRZ_Pin, GPIO_PIN_SET); zMoving = true; stepsRemainingZ = W_T_X; axis_homed = true; skip_end_sw = true; }
		else { Restart_States(); homing_state_machine = HO_IDLE; }
		break;
	default: break;


	}

}

void Serial_Comms(void){
	uint8_t c;
	// --- SERIAL COMMS ---
	// Non-blocking UART receive (timeout = 0)
	if (HAL_UART_Receive(&huart2, &c, 1, 0) != HAL_OK) return;

	switch (c) {
		case 'g':
		  Restart_States();
		  opt.optimize_state_machine = OPT_Z;
		  opt.step_size = 10;
		  output_pow_tm = ADC_REFRESH_TIME;
		  break;

		case 'b':
			output_pow_tm = 2;
			ddm_benchmark_i = 0;
			ddm_benchmark = true;
			ddm_benchmark_timestamp = HAL_GetTick();
			HAL_GPIO_WritePin(DIRY_Port, DIRY_Pin, GPIO_PIN_SET); yMoving = true; stepsRemainingY = 200;

			break;

		case 'c':
		  if(ac.state == AC_OFF){
			  Serial_Println("Active compensation ON!");
			  Restart_States();
			  ac.state = AC_PAVG_MEAS;
			  ac.SMA_num = ac.pavg_num;
			  output_pow_tm = ADC_REFRESH_TIME;
		  }
		  else {
			  Serial_Println("Active compensation OFF!");
			  Restart_States();
			  ac.state = AC_OFF;
		  }
		  break;
		case '\'':
			if(ac.state == AC_IDLE) {
			  Serial_Println("Receive AC_INIT, START RUN");
			  ac.me_init = false;
			  ac.dir_change_count[0] = 0; ac.dir_change_count[1] = 0;
			  memset(ac.mov_avg_arr, 0, sizeof(ac.mov_avg_arr));
			  ac.state = AC_RUN;
			  ac.iter = AC_RUN_ITERATIONS;
			  ac.EMA_num = 0;
			}

		  break;
		case '"':
		  if(ac.state == AC_IDLE){
			  Serial_Println("Receive AC_DONE, START RUN");
			  ac.dir_change_count[0] = 0; ac.dir_change_count[1] = 0;
			  memset(ac.mov_avg_arr, 0, sizeof(ac.mov_avg_arr));
			  ac.state = AC_RUN;
			  ac.iter = AC_RUN_ITERATIONS;
			  ac.EMA_num = 0;
		  }
		  else if(ac.state == AC_WAIT_TO_SAVE){
			  Serial_Println("Receive AC_DONE, PROCESS AVG");
			  ac.state = AC_PAVG_MEAS;
			  ac.SMA_num = ac.pavg_num;
			  ac.mov_avg = 0;
		  }
		  break;

		case 'h':
		  homing_state_machine = TO_D;
		  break;
		case 'k':
		  homing_state_machine = TO_E;
		  break;
		case 'j':

		  Restart_States();
		  output_pow_tm = ADC_REFRESH_TIME;
		  spiral.state = GO_D;
		  spiral.increment = SPIRAL_INCREMENT;
		  break;

		case 'J':
		  Restart_States();
		  output_pow_tm = ADC_REFRESH_TIME;
		  spiral.state = GO_D;
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
		case ' ':
		 snprintf(buf, sizeof(buf), "steps_bef_resetY: %u, steps_bef_resetZ: %u, steps_bef_resetX: %u\r\n", steps_bef_resetY, steps_bef_resetZ, steps_bef_resetX);
		 Serial_Print(buf);
		 Restart_States();
		 break;
	}
}



void I2C_Readout(void){
	//READOUT
	static uint16_t ddm_benchmark_time[256];
	static uint16_t ddm_benchmark_power[256];

	if (HAL_I2C_Mem_Read(&hi2c1, SFP_ADDR_DDM, 96, I2C_MEMADD_SIZE_8BIT, &ddm_data[96], 22,100) != HAL_OK) return;
	char buf[32];
	// --- RX POWER
	// Offset 104-105. Units are 0.1 uW
	rx_raw = ((ddm_data[104] << 8) | ddm_data[105]) - 1;

	// Integer division gives the whole number, modulo gives the remainder
	uint16_t whole_uw = rx_raw / 10;
	uint16_t decimal_uw = rx_raw % 10;

	if(spiral.state != IDLE && rx_raw > 0 && rx_raw < 100) {Restart_States();}

	timestamp = HAL_GetTick();
	// Outputs: "Power: 123.4 uW"
	if(ac.state == AC_OFF && !ddm_benchmark){
		snprintf(buf, sizeof(buf), "T:%lu, Power: %u.%u uW\r\n",timestamp , whole_uw, decimal_uw);
		Serial_Print(buf);
	}
	else if(ddm_benchmark){
		ddm_benchmark_time[ddm_benchmark_i] = timestamp - ddm_benchmark_timestamp;
		ddm_benchmark_power[ddm_benchmark_i++] = rx_raw;
		ddm_benchmark_i = ddm_benchmark_i % 256;
		if(!stepsRemainingY){
			//PRINT IN LOOP
			snprintf(buf, sizeof(buf), "Benchmark ok, i: %u", ddm_benchmark_i);
			Serial_Println(buf);
			for(int i = 0; i < ddm_benchmark_i; ++i){
				snprintf(buf, sizeof(buf), "%u:%u;\r\n", ddm_benchmark_time[i], ddm_benchmark_power[i]);
				Serial_Print(buf);
			}

			ddm_benchmark = false;
			ddm_benchmark_i = 0;
			output_pow_tm = 0xFFFF;
		}
	}

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

}
void Optimize(void){
	if(opt.optimize_state_machine == OPT_IDLE) return;
	if(rx_raw >= rx_raw_max){
		rx_raw_max = rx_raw;
		rx_raw_max_dir = opt.dir;
		if(end_on_max){
			stepsRemainingZ = 0; stepsRemainingY = 0;
			switch(opt.optimize_state_machine){
				case OPT_Z: opt.optimize_state_machine = OPT_Y; break;
				case OPT_Y: opt.optimize_state_machine = OPT_IDLE; output_pow_tm = 300; break;
				default: break;
			}
			rx_raw_max = 0;
			rx_raw_max_dir = 0;
			end_on_max = false;
			histeresis_hit = false;
			first_sweep = false;
			second_sweep = false;
			end_on_max = false;
			opt.step_size = 10;
			return;
		}
	}

	if((rx_raw < rx_raw_max / 4 || rx_raw == 0) && histeresis_hit){
		opt.dir = !opt.dir;
		if(end_on_max && opt.step_size != 1) {if((opt.step_size -= 2) < 1) opt.step_size = 1;}
		else if(end_on_max) rx_raw_max -= 25;
		end_on_max = second_sweep;
		second_sweep = first_sweep;
		first_sweep = true;
		histeresis_hit = false;
	}
	//WHen we skip histeresis point and go below it down to 0 is catastrophic, things just broke up :/
	else if(rx_raw > rx_raw_max / 2) histeresis_hit = true;

	switch(opt.optimize_state_machine){
		case OPT_Z: HAL_GPIO_WritePin(DIRZ_Port, DIRZ_Pin, opt.dir); zMoving = true; stepsRemainingZ = opt.step_size;  break;
		case OPT_Y: HAL_GPIO_WritePin(DIRY_Port, DIRY_Pin, opt.dir); yMoving = true; stepsRemainingY = opt.step_size; break;
		default: break;
	}
}

void Active_Compensation(void){

	char buf[64];
	static uint8_t print_check;
	if(ac.state == AC_OFF) return;
	static uint32_t ac_timestamp;
//	static uint16_t start_level, max_level;

	// whole procedure must be pretty quick, to make it work real time
	switch(ac.state){
	case AC_IDLE:
		//Measure Pavg, needs to ignore 0's or very low as it is some accidential collision
		//If Pavg < tresh*Pmax then communicate a will of compensation

		if(rx_raw == 0xFFFF) break; // eliminate big error
		if(rx_raw == 0x0000) break; // eliminate big error
		ac.mov_avg = rx_raw / AC_EMA_PERIOD + ac.mov_avg * (AC_EMA_PERIOD - 1) / AC_EMA_PERIOD;
		if((++print_check % 100) == 0){
			print_check = 0;
			snprintf(buf, sizeof(buf), "T:%lu Pavg = %lu.%lu uW \r\n", HAL_GetTick(), ac.mov_avg / 10, ac.mov_avg % 10);
			Serial_Print(buf);
		}
		if(HAL_GetTick() - ac_timestamp > AC_RETRANSMIT_MS && ac.mov_avg <= ac.pmax * AC_THRESH_NUM / AC_THRESH_DEN){
			Serial_Println("AC_INIT");
			snprintf(buf, sizeof(buf), "T:%lu Power dropped, Pavg = %lu.%lu uW \r\n", HAL_GetTick(), ac.mov_avg / 10, ac.mov_avg % 10);
			Serial_Print(buf);

			ac_timestamp = HAL_GetTick();
			ac.me_init = true;
		}
		else if(HAL_GetTick() - ac_timestamp > AC_RETRANSMIT_MS && ac.mov_avg >= ac.pmax * (AC_THRESH_DEN * 2 - AC_THRESH_NUM) / AC_THRESH_DEN){
			Serial_Println("AC_INIT");
			snprintf(buf, sizeof(buf), "T:%lu Power increased, Pavg = %lu.%lu uW \r\n", HAL_GetTick(), ac.mov_avg / 10, ac.mov_avg % 10);
			Serial_Print(buf);

			ac_timestamp = HAL_GetTick();
			ac.me_init = true;
		}

		break;


   	case AC_RUN:
		//Receive signal from Serialcomms, AC_INIT/AC_DONE FROM AC_IDLE
   		bool cond_swap_dir = true;
		ac.mov_avg = ac.mov_avg / AC_RUN_DIVIDER + rx_raw * (AC_RUN_DIVIDER - 1) / AC_RUN_DIVIDER;
		if(ac.EMA_num == 0){
			if(ac.dir){
				//Check for down pattern or flat pattern
				for(int i = AC_RUN_HISTORY - 1; i > 0; --i)
					if(ac.mov_avg_arr[i] < ac.mov_avg_arr[i - 1]) cond_swap_dir = false;
				if(ac.mov_avg_arr[0] < ac.mov_avg) cond_swap_dir = false;
				if(cond_swap_dir) { ac.dirz = !ac.dirz; ++ac.dir_change_count[ac.dir];}
				HAL_GPIO_WritePin(DIRZ_Port, DIRZ_Pin, ac.dirz); zMoving = true;
				if(cond_swap_dir) stepsRemainingZ = (AC_RUN_HISTORY + 1) * ac.steps;
				else stepsRemainingZ = ac.steps;
			}
			else if (!ac.dir){
				//Check for down pattern or flat pattern
				for(int i = AC_RUN_HISTORY - 1; i > 0; --i)
					if(ac.mov_avg_arr[i] < ac.mov_avg_arr[i - 1]) cond_swap_dir = false;
				if(ac.mov_avg_arr[0] < ac.mov_avg) cond_swap_dir = false;
				if(cond_swap_dir) {ac.diry = !ac.diry; ++ac.dir_change_count[ac.dir];}
				HAL_GPIO_WritePin(DIRY_Port, DIRY_Pin, ac.diry); yMoving = true;
				if(cond_swap_dir) stepsRemainingY = (AC_RUN_HISTORY + 1) * ac.steps;
				else stepsRemainingY = ac.steps;
			}
			--ac.iter;
			//Change Z/Y if done
			if(ac.dir_change_count[ac.dir] == 2) {
				ac.dir = !ac.dir;
				memset(ac.mov_avg_arr, 0, sizeof(ac.mov_avg_arr));
			}
			//If both done signal finish
			if(ac.dir_change_count[0] == 2 && ac.dir_change_count[1] == 2) ac.iter = 0;
			//Save current result
			for(int i = AC_RUN_HISTORY - 1; i > 0; --i){
				ac.mov_avg_arr[i] = ac.mov_avg_arr[i - 1];
			}
			ac.mov_avg_arr[0] = ac.mov_avg;
		}
		ac.EMA_num = (ac.EMA_num + 1) % AC_TICKS_PER_STEP;
		//Finish
		if(!ac.iter){
			ac.state = AC_WAIT_TO_SAVE;
			ac_timestamp = HAL_GetTick();
			Serial_Println("AC_DONE");
		}
		break;


	case AC_WAIT_TO_SAVE:
		//wait for 2nd to finish and save, AC_DONE INDICATES
		if(ac.me_init) { // if i begin i measure
			ac.state = AC_PAVG_MEAS;
			ac.SMA_num = ac.pavg_num;
			ac.mov_avg = 0;
		}
		if(HAL_GetTick() - ac_timestamp > AC_TIMEOUT_MS)
		{
			Serial_Println("NO COMMS, State = PAVG_MEAS");
			ac.state = AC_PAVG_MEAS;
			ac.SMA_num = ac.pavg_num;
			ac.mov_avg = 0;
		}
		break;

	case AC_PAVG_MEAS:
		//measure, save as max and go idle
		if(rx_raw == 0xFFFF) break;
		ac.mov_avg += rx_raw;
		if(!(--ac.SMA_num))
		{

			ac.pmax = ac.mov_avg / ac.pavg_num;
			ac.mov_avg = ac.pmax;
			ac.me_init = false;

			snprintf(buf, sizeof(buf), "T:%lu Pmax = %u.%u uW \r\n", HAL_GetTick(), ac.pmax / 10, ac.pmax % 10);
			Serial_Print(buf);
			ac.state = AC_IDLE;
		}
		break;

	default: break;
	}
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
  Restart_States();
  ac.steps = 10;
  ac.dirz = 0;
  ac.diry = 0;
  ac.pavg_num = 20;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

	if(!pause){
		Handle_Movement();
		Spiral_State_Machine();
		Homing_State_Machine();
	}

	Serial_Comms();

	if(HAL_GetTick() - timestamp > output_pow_tm){
		I2C_Readout();
		if(opt.optimize_state_machine != OPT_IDLE && !stepsRemainingX && !stepsRemainingY && !stepsRemainingZ) Optimize();
		if(ac.state != AC_OFF) Active_Compensation();
	}





    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }

  /*
   	case AC_RUN:
		//Receive signal from Serialcomms, AC_INIT/AC_DONE FROM AC_IDLE
		//Check if start measure this axis
		ac.mov_avg = (ac.mov_avg + rx_raw) / 2;
		if(ac.mov_avg_arr[0] == 0) {
			start_level = ac.mov_avg;
			max_level = ac.mov_avg;
		}
		if(ac.mov_avg > max_level) max_level = ac.mov_avg;

		if(ac.EMA_num == 0){
			if(ac.dir_change_count[ac.dir] == 2 && ac.mov_avg >= max_level * AC_THRESH_NUM / AC_THRESH_DEN){
				++ac.dir_change_count[ac.dir]; // FINISH
				Serial_Println("Axis found");
			}
			else if(ac.dir){
				if(ac.mov_avg < start_level / 2) {
					ac.dirz = !ac.dirz;
					++ac.dir_change_count[ac.dir];
					stepsRemainingZ = (5) * ac.steps;
					Serial_Println("Change dirz");
				}
				else stepsRemainingZ = ac.steps;
				HAL_GPIO_WritePin(DIRZ_Port, DIRZ_Pin, ac.dirz); zMoving = true;
			}
			else if (!ac.dir){
				if(ac.mov_avg < start_level / 2) {
					ac.diry = !ac.diry;
					++ac.dir_change_count[ac.dir];
					stepsRemainingY = (5) * ac.steps;
					Serial_Println("Change diry");
				}
				else stepsRemainingY = ac.steps;
				HAL_GPIO_WritePin(DIRY_Port, DIRY_Pin, ac.diry); zMoving = true;
			}
			--ac.iter;
			//Change Z/Y if done
			if(ac.dir_change_count[ac.dir] == 3) {
				ac.dir = !ac.dir;
				memset(ac.mov_avg_arr, 0, sizeof(ac.mov_avg_arr));
			}
			//If both done signal finish
			if(ac.dir_change_count[0] == 3 && ac.dir_change_count[1] == 3) ac.iter = 0;
			//Save current result
			for(int i = AC_RUN_HISTORY - 1; i > 0; --i){
				ac.mov_avg_arr[i] = ac.mov_avg_arr[i - 1];
			}
			ac.mov_avg_arr[0] = ac.mov_avg;
		}
		ac.EMA_num = (ac.EMA_num + 1) % AC_TICKS_PER_STEP;
		//Finish
		if(!ac.iter){
			ac.state = AC_WAIT_TO_SAVE;
			ac_timestamp = HAL_GetTick();
			Serial_Println("AC_DONE");
		}
		break;

   */
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
  hi2c1.Init.ClockSpeed = 400000;
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
