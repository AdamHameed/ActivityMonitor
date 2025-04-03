/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include <string.h>  
#include <stdlib.h> 
#include <math.h> 
#include <stdio.h>
#include "step_counter.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE BEGIN PD */
#define PMODACL2_CS_PIN GPIO_PIN_4
#define PMODACL2_CS_PORT GPIOA

// PmodACL2 Register Addresses
#define PMODACL2_PART_ID         0x02  // Device ID
#define PMODACL2_X_DATA          0x08  // X-axis data
#define PMODACL2_Y_DATA          0x09  // Y-axis data
#define PMODACL2_Z_DATA          0x0A  // Z-axis data
#define PMODACL2_STATUS          0x0B  // Status register
#define PMODACL2_XDATA_L         0x0E  // X-axis data low byte
#define PMODACL2_XDATA_H         0x0F  // X-axis data high byte
#define PMODACL2_YDATA_L         0x10  // Y-axis data low byte
#define PMODACL2_YDATA_H         0x11  // Y-axis data high byte
#define PMODACL2_ZDATA_L         0x12  // Z-axis data low byte
#define PMODACL2_ZDATA_H         0x13  // Z-axis data high byte
#define PMODACL2_POWER_CTL       0x2D  // Power control register

// breathing rate mode constants
#define SAMPLE_INTERVAL 15  // for breathing rate mode: measure breathing rate every 15 seconds
#define SAMPLING_RATE 2     // 2 samples per second
#define MAX_SAMPLES (SAMPLE_INTERVAL * SAMPLING_RATE)

volatile current_state_t previousState = STATE_IDLE;
volatile current_state_t currentState = STATE_IDLE;
uint32_t lastActivityTime = 0;

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart3;

PCD_HandleTypeDef hpcd_USB_OTG_FS;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_USB_OTG_FS_PCD_Init(void);
/* USER CODE BEGIN PFP */
void PmodACL2_CS_Low(void);
void PmodACL2_CS_High(void);
void PmodACL2_WriteRegister(uint8_t reg, uint8_t data);
void PmodACL2_Init(void);
float estimate_breathing_rate(float *vertical_accel, int length);
void reset_velocity();
void estimate_velocity();
void calibrate_offset();
void play_balancing_game();

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// Chip Select Control Functions
StepCounterConfig step_config = {
    .min_step_threshold = 1500,
    .max_step_threshold = 5000,
    .window_size = 5,
    .step_delay_ms = 300,
    .frequency_cutoff_hz = 5.0f,
    .sample_rate_hz = 50
};

void PmodACL2_CS_Low() {
    HAL_GPIO_WritePin(PMODACL2_CS_PORT, PMODACL2_CS_PIN, GPIO_PIN_RESET);
}

void PmodACL2_CS_High() {
    HAL_GPIO_WritePin(PMODACL2_CS_PORT, PMODACL2_CS_PIN, GPIO_PIN_SET);
}

void print_msg(char * msg) {
  HAL_UART_Transmit(&huart3, (uint8_t *)msg, strlen(msg), 100);
}

void ReadFIFO(uint8_t *axis, int16_t *data) {
    uint8_t txData = 0x0D;
    uint8_t rxData[3] = {0};

    PmodACL2_CS_Low();

    if (HAL_SPI_Transmit(&hspi1, &txData, 1, HAL_MAX_DELAY) != HAL_OK) {
        print_msg("SPI Transmit Error\n");
        PmodACL2_CS_High();
        return;
    }

    // Receive axis identifier and data (2 bytes)
    if (HAL_SPI_Receive(&hspi1, rxData, 3, HAL_MAX_DELAY) != HAL_OK) {
        print_msg("SPI Receive Error\n");
        PmodACL2_CS_High(); // Deactivate CS
        return;
    }

    PmodACL2_CS_High(); // Deactivate CS

    *axis = rxData[0]; // Axis identifier (b15 and b14)
    *data = (int16_t)((rxData[1] << 8) | rxData[2]); // Combine LSB and MSB
}
void ConfigureFIFO() {
    // FIFO Register is 0x28
    // Set FIFO mode to "Stream", store X, Y, Z data
    uint8_t fifoConfig = 0x80; 
    PmodACL2_WriteRegister(0x28, fifoConfig);
    print_msg("FIFO buffer configured.\n");
}

void PmodACL2_WriteRegister(uint8_t reg, uint8_t data) {
    uint8_t txData[3] = {0x0A, reg, data};

    PmodACL2_CS_Low();
    HAL_SPI_Transmit(&hspi1, txData, 3, HAL_MAX_DELAY);
    PmodACL2_CS_High();
}

void PmodACL2_Init() {
    PmodACL2_WriteRegister(PMODACL2_POWER_CTL, 0x02);
    HAL_Delay(100);
}
void PmodACL2_ReadRegister(uint8_t reg, uint8_t *data, uint8_t length) {
    uint8_t txData[2] = {0x0B, reg};  // Read command and register address
    uint8_t rxData[2 + length];  // Buffer for received data

    PmodACL2_CS_Low();
    HAL_SPI_TransmitReceive(&hspi1, txData, rxData, 2 + length, HAL_MAX_DELAY);
    PmodACL2_CS_High();

    // Copy received data to output buffer
    for (uint8_t i = 0; i < length; i++) {
        data[i] = rxData[i + 2];
    }
}

void PmodACL2_CheckStatus() {
    uint8_t status[1] = {0};
    char message[100];
    PmodACL2_ReadRegister(0x0B, status, 1);  // Read STATUS register
//    sprintf(message, "STATUS: %d \n", status[0]&0x01);
//    print_msg(message);
}

/**
  * @brief  The application entry point.
  * @retval int
  */


// PmodACL2 registers
#define READ_COMMAND 0x0B  // read from data registers
#define Y_AXIS_ADDRESS 0x0A  // Address of Y-axis data register

// Function to read Y-axis data

#define DEVID_AD 0x00      // Device ID register (expected: 0xAD)
#define DEVID_MST 0x01     // Device ID for MEMS (expected: 0x1D)

void PmodACL2_TestPowerCtl() {
    uint8_t powerCtl[1] = {0};
    char message[100];

    // Read the POWER_CTL register
    PmodACL2_ReadRegister(0x2D, powerCtl, 1);  // Read POWER_CTL register

    // Format the message
    sprintf(message, "POWER_CTL: 0x%02X\n", powerCtl[0]);

    // Print the message
    print_msg(message);
}

void PmodACL2_ConfigureShakeInterrupt() {
    PmodACL2_WriteRegister(0x20, 0xFA); 
    PmodACL2_WriteRegister(0x21, 0x00);
	
    // inactivity threshold to 150 mg
    PmodACL2_WriteRegister(0x23, 0x96); 
	
    // inactivity timer to 30 samples
    PmodACL2_WriteRegister(0x25, 0x1E);
	
    // activity/inactivity detection
		PmodACL2_WriteRegister(0x27, 0x3F); 
	
    // connect the awake signal to INT1 pin
    PmodACL2_WriteRegister(0x2A, 0x40);
}


void PmodACL2_ResetFIFO() {
    PmodACL2_WriteRegister(0x28, 0x00);  // FIFO_CTRL register, disable FIFO
    HAL_Delay(10);
}

void PmodACL2_get_acceleration(float *xAccel, float *yAccel, float *zAccel) {
    uint8_t xyzData[3] = {0};
    PmodACL2_ReadRegister(0x08, xyzData, 3);

    *xAccel = ((int8_t)xyzData[0]) * 0.015625f;
    *yAccel = ((int8_t)xyzData[1]) * 0.015625f;
    *zAccel = ((int8_t)xyzData[2]) * 0.015625f;
}

void PmodACL2_step_8_bit_values() {
    // step counting
    static uint16_t stepCount = 0;
    static uint16_t possibleSteps = 0;
    static float dynamicThreshold = 0.0f;
    static const float sensitivity = 0.5f;
    static float history[4] = {0};
    static int historyIndex = 0;
    static int state = 0;
    static float maxVal = 0.0f;
    static float minVal = 0.0f;
    static uint16_t ticksSinceMax = 0;
    static uint16_t consecutiveSteps = 0;

    // calorie estimation
    static float totalCalories = 0.0f;
    static float lastUpdateTime = 0.0f;
    const float MET_WALKING = 3.5f;  // metabolic equivalent for walking
    const float USER_WEIGHT_KG = 60.0f;

    int8_t xData = 0, yData = 0, zData = 0;
    uint8_t status[1] = {0};
    char message[100];
    PmodACL2_ReadRegister(0x0B, status, 1); 

    if (status[0] & 0x01) {
        uint8_t xyzData[3] = {0};
        PmodACL2_ReadRegister(0x08, xyzData, 3);

        int8_t xData = (int8_t)xyzData[0];
        int8_t yData = (int8_t)xyzData[1];
        int8_t zData = (int8_t)xyzData[2];

        float sumAbs = fabsf((float)xData) + fabsf((float)yData) + fabsf((float)zData);

        history[historyIndex] = sumAbs;
        historyIndex = (historyIndex + 1) % 4;
        float filtered = 0.0f;
        for(int i=0; i<4; i++) filtered += history[i];
        filtered /= 4.0f;

        // Peak detection
        if(state == 0) {
            if(filtered > maxVal) {
                maxVal = filtered;
                ticksSinceMax = 0;
            }
            ticksSinceMax++;
            if(ticksSinceMax > 5) {
                state = 1;
                minVal = filtered;
            }
        } else {
            if(filtered < minVal) minVal = filtered;
            if(++ticksSinceMax > 20) {
                float peakDiff = maxVal - minVal;
                float avgPeak = (maxVal + minVal) / 2.0f;
                if(peakDiff > sensitivity) {
                    if(fabsf(avgPeak - dynamicThreshold) > sensitivity * 0.5f) {
                        dynamicThreshold = (dynamicThreshold + avgPeak) / 2.0f;
                    }
                    if(maxVal > dynamicThreshold + (sensitivity * 0.5f) &&
                       minVal < dynamicThreshold - (sensitivity * 0.5f)) {
                        possibleSteps++;
                        if(possibleSteps >= 10) {
                            consecutiveSteps++;
                            stepCount++;
                            
                            // calorie calculation
                            // - 1 step ˜ = 0.75 meters
                            // - calories = MET * weight_kg * time_hours
                            // 100 steps ˜~= 0.5 calories
                            totalCalories += USER_WEIGHT_KG * 0.0005f;
                            
                            sprintf(message, "Steps: %d | Calories: %.2f\n", stepCount, totalCalories);
                            print_msg(message);
                        }
                    } else {
                        possibleSteps = 0;
                    }
                }
                maxVal = filtered;
                minVal = filtered;
                ticksSinceMax = 0;
                state = 0;
            }
        }
    }
}

void freefall(){
	static int step_count = 0;
    uint8_t status[1] = {0};
    char message[100];
    PmodACL2_ReadRegister(0x0B, status, 1);
    

    if (status[0] & 0x01) {
      float xAcceleration;
      float yAcceleration;
      float zAcceleration;
			PmodACL2_get_acceleration(&xAcceleration, &yAcceleration, &zAcceleration);
      
      float magSquared = xAcceleration*xAcceleration + yAcceleration*yAcceleration + zAcceleration*zAcceleration;
      
      const float threshold_squared = 0.1f;
      
      // Check if magnitude squared is below threshold
      if (magSquared < threshold_squared) {
        HAL_GPIO_WritePin(LD1_GPIO_Port, LD1_Pin, GPIO_PIN_SET);
        print_msg("Free fall detected!\n");
        step_count++;
        //sprintf(message, "%d\n", step_count);
        //print_msg(message);
        //HAL_Delay(100);
      } else {
        HAL_GPIO_WritePin(LD1_GPIO_Port, LD1_Pin, GPIO_PIN_RESET);
      }

      char message[100];
//      sprintf(message, "Accel magnitude: %.3f g\n", sqrtf(magSquared));
//      print_msg(message);
    }
  
}


void CheckActivityTimeout(void) {
    static float lastFilteredValue = 0;
    const uint32_t inactivityTimeout = 10000;
    uint8_t xyzData[3] = {0};
    PmodACL2_ReadRegister(0x08, xyzData, 3);

    int8_t xValue = (int8_t)xyzData[0];
    int8_t yValue = (int8_t)xyzData[1];
    int8_t zValue = (int8_t)xyzData[2];

    // Convert to real-world acceleration (g)
    float xAcceleration = (float)xValue * 0.015625f;
    float yAcceleration = (float)yValue * 0.015625f;
    float zAcceleration = (float)zValue * 0.015625f;

    float magSquared = xAcceleration*xAcceleration + yAcceleration*yAcceleration + zAcceleration*zAcceleration;
    float currentFiltered =  magSquared;

    if(fabsf(currentFiltered - lastFilteredValue) > 1) {  // Activity detected
        lastActivityTime = HAL_GetTick();
        lastFilteredValue = currentFiltered;
        HAL_GPIO_WritePin(LD1_GPIO_Port, LD1_Pin, 0);
    }
    else if(HAL_GetTick() - lastActivityTime > inactivityTimeout) {
			// GO TO SLEEP
			reset_velocity();
    	currentState = STATE_IDLE;
    }
}


void enter_sleep_mode(void) {
    HAL_SPI_DeInit(&hspi1);
    HAL_UART_DeInit(&huart3);
    __HAL_RCC_SPI1_CLK_DISABLE();
    __HAL_RCC_USART3_CLK_DISABLE();
    
    HAL_PWR_EnableWakeUpPin(PWR_WAKEUP_PIN2);
    __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_2);
    __HAL_PWR_CLEAR_FLAG(PWR_FLAG_WU);
	
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);

    // disable LED GPIOs
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_0);
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_1);

    __HAL_RCC_GPIOB_CLK_DISABLE();
    __HAL_RCC_SPI1_CLK_DISABLE();
    __HAL_RCC_USART3_CLK_DISABLE();
	
		HAL_SuspendTick();
    HAL_PWR_EnterSLEEPMode(PWR_MAINREGULATOR_ON, PWR_SLEEPENTRY_WFI); // blocks here
    HAL_ResumeTick();
		
    // wakeup
    SystemClock_Config();
    MX_GPIO_Init();
    MX_SPI1_Init();
    MX_USART3_UART_Init();
    PmodACL2_Init();
    PmodACL2_ConfigureShakeInterrupt();
    
    __HAL_PWR_CLEAR_FLAG(PWR_FLAG_WU);
		currentState = STATE_PASSIVE; 
		lastActivityTime = HAL_GetTick(); 
}


void breathing_mode() {
    static float acc_z[MAX_SAMPLES];
    static int sample_index = 0;
    static int isCalibrating = 1;
    static uint32_t lastSampleTime = 0;
    char message[100];

    uint32_t currentTime = HAL_GetTick();

    if (currentTime - lastSampleTime >= 500) {  
        lastSampleTime = currentTime;

        float xAccel, yAccel, zAccel;
        PmodACL2_get_acceleration(&xAccel, &yAccel, &zAccel);
				
				// populate our sample buffer
        acc_z[sample_index] = zAccel;
        sample_index++;

				// while we're sampling, print a message for more clarity
        if (isCalibrating) {
            print_msg("Calibrating breathing rate....\n");
            isCalibrating = 0;
        }

				// once we get enough samples, we pass it into our estimation function
				// which smoothes the data for denoising and counts peaks
        if (sample_index >= MAX_SAMPLES) {
            float bpm = estimate_breathing_rate(acc_z, MAX_SAMPLES);
            sprintf(message, "Breathing Rate: %.2f BPM\n", bpm);
            print_msg(message);
            isCalibrating = 1;
            sample_index = 0;
        }
    }
}


volatile uint8_t accelInterruptFlag = 0;

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    if (GPIO_Pin == GPIO_PIN_2) {
        accelInterruptFlag = 1;
        HAL_GPIO_TogglePin(LD1_GPIO_Port, LD1_Pin);
        //print_msg("INT1 detected\n");
    }
}

void print_state_message(current_state_t state) {
    switch (state) {
        case STATE_PASSIVE:
            print_msg("[MODE] Passive\n");
						print_msg("[+] Step counting, fall detection, calorie tracking enabled\n");
            break;
        case STATE_IDLE:
            print_msg("[MODE] Idle\n");
            break;
				case STATE_BALANCE:
						print_msg("[MODE] Balancing Game\n");
						break;
				case STATE_VELOCITY:
						print_msg("[MODE] Velocity Measurement\n");
						break;
				case STATE_BREATH:
						print_msg("[MODE] Breathing Rate Measurement\n");
						break;
        default:
            print_msg("[MODE] Unknown State\n");
            break;
    }
}


int main(void)
{

  /* USER CODE BEGIN 1 */
  int8_t x, y, z;
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
  MX_SPI1_Init();
  MX_USART3_UART_Init();
  MX_USB_OTG_FS_PCD_Init();
  PmodACL2_Init();
  PmodACL2_ResetFIFO();
	
  /* USER CODE BEGIN 2 */
  // Initialize the PmodACL2
//  PmodACL2_Init();

//  PmodACL2_CS_Low();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
//  ConfigureFIFO();
/* In main() (before while-loop) */
	HAL_NVIC_SetPriority(EXTI2_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(EXTI2_IRQn);
	PmodACL2_ConfigureShakeInterrupt();
	calibrate_offset();
  uint8_t axis;
  int16_t data;

  while (1)
  {
		
		if (currentState != previousState) {
				print_state_message(currentState);
        previousState = currentState;  
    }
		
      switch(currentState) {
					case STATE_IDLE:
						enter_sleep_mode();
						// executes here after wakeup
						break;
					case STATE_PASSIVE:
        	  PmodACL2_step_8_bit_values();
						freefall();
						CheckActivityTimeout();
             break;
					case STATE_BREATH:
						breathing_mode();
						//CheckActivityTimeout();
						break;
					case STATE_BALANCE:
						//print_msg("Balance game starting in 5 seconds...\n");
						//HAL_Delay(5000);
						play_balancing_game();
						//CheckActivityTimeout();
						break;
					case STATE_VELOCITY:
						estimate_velocity();
						CheckActivityTimeout();
      }
    // Delay for readability
	    HAL_Delay(50);
  }
  /* USER CODE END 3 */
}
/* USER CODE END 0 */


/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  RCC_OscInitStruct.PLL.PLLR = 2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief USB_OTG_FS Initialization Function
  * @param None
  * @retval None
  */
static void MX_USB_OTG_FS_PCD_Init(void)
{

  /* USER CODE BEGIN USB_OTG_FS_Init 0 */

  /* USER CODE END USB_OTG_FS_Init 0 */

  /* USER CODE BEGIN USB_OTG_FS_Init 1 */

  /* USER CODE END USB_OTG_FS_Init 1 */
  hpcd_USB_OTG_FS.Instance = USB_OTG_FS;
  hpcd_USB_OTG_FS.Init.dev_endpoints = 6;
  hpcd_USB_OTG_FS.Init.speed = PCD_SPEED_FULL;
  hpcd_USB_OTG_FS.Init.dma_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.phy_itface = PCD_PHY_EMBEDDED;
  hpcd_USB_OTG_FS.Init.Sof_enable = ENABLE;
  hpcd_USB_OTG_FS.Init.low_power_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.lpm_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.vbus_sensing_enable = ENABLE;
  hpcd_USB_OTG_FS.Init.use_dedicated_ep1 = DISABLE;
  if (HAL_PCD_Init(&hpcd_USB_OTG_FS) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USB_OTG_FS_Init 2 */

  /* USER CODE END USB_OTG_FS_Init 2 */

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LD1_Pin|LD3_Pin|LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(USB_PowerSwitchOn_GPIO_Port, USB_PowerSwitchOn_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : USER_Btn_Pin */
  GPIO_InitStruct.Pin = USER_Btn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USER_Btn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PF2 */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pin : PA4 */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : LD1_Pin LD3_Pin LD2_Pin */
  GPIO_InitStruct.Pin = LD1_Pin|LD3_Pin|LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_PowerSwitchOn_Pin */
  GPIO_InitStruct.Pin = USB_PowerSwitchOn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(USB_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_OverCurrent_Pin */
  GPIO_InitStruct.Pin = USB_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USB_OverCurrent_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

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
