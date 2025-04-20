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

// globals for velocity calculation
float velocity_x = 0.0f;
float velocity_y = 0.0f;
float velocity_z = 0.0f;
static float prev_filtered_x = 0, prev_filtered_y = 0, prev_filtered_z = 0;

float offsetX = 0.0f;
float offsetY = 0.0f;
float offsetZ = 0.0f;

void PmodACL2_WriteRegister(uint8_t reg, uint8_t data);
void PmodACL2_ReadRegister(uint8_t reg, uint8_t *data, uint8_t length);
void print_msg(char * msg);

void reset_velocity() {
	velocity_x = 0.0f;
  velocity_y = 0.0f;
  velocity_z = 0.0f;
}

//  estimate velocity
static float prev_accelX = 0, prev_accelY = 0, prev_accelZ = 0;
static float still_timer = 0;

// we need to run this at the start when the monitor is at rest
// because otherwise our raw accelerations will be wrong
void calibrate_offset() {
		uint8_t status[1] = {0};
		PmodACL2_ReadRegister(0x0B, status, 1);
   
    if (status[0] & 0x01) {
			static float accelData[3]; 
			uint8_t xyzData[3] = {0};
			PmodACL2_ReadRegister(0x08, xyzData, 3);
			
			offsetX = (float)((int8_t)xyzData[0]);
			offsetY = (float)((int8_t)xyzData[1]);
			offsetZ = (float)((int8_t)xyzData[2]);
	}
}


void estimate_velocity() {
		static uint32_t lastPrintTime = 0;
    const uint32_t PRINT_INTERVAL = 1000; 
    static uint32_t lastUpdateTime = 0;
    uint32_t currentTime = HAL_GetTick();
    float dt = (currentTime - lastUpdateTime) / 1000.0f; 
    lastUpdateTime = currentTime;

    if (dt <= 0 || dt > 1.0f) return; // normalize extreme values

    uint8_t status[1] = {0};
    const float alpha = 0.2f;    // high-pass,
    const float threshold = 0.08f; // threshold value for deadzone
    const float STILL_TIME_THRESHOLD = 0.07f;
    const float STILLNESS_THRESHOLD = 0.05f; 

    PmodACL2_ReadRegister(0x0B, status, 1);

    if (status[0] & 0x01) {
        uint8_t xyzData[3] = {0};
        char message[100];

        // Read raw data
        PmodACL2_ReadRegister(0x08, xyzData, 3);
				
        float accelX = ((float)((int8_t)xyzData[0]) - offsetX) * 0.015625f;
        float accelY = ((float)((int8_t)xyzData[1]) - offsetY) * 0.015625f;
        float accelZ = ((float)((int8_t)xyzData[2]) - offsetZ) * 0.015625f;

        float filteredX = alpha * prev_filtered_x + (1 - alpha) * accelX;
        float filteredY = alpha * prev_filtered_y + (1 - alpha) * accelY;
        float filteredZ = alpha * prev_filtered_z + (1 - alpha) * accelZ;

        float accelMagnitude = sqrt(filteredX * filteredX + filteredY * filteredY);

        // update velocity if above threshold
        if (accelMagnitude > threshold) {
            velocity_x += filteredX * dt;
            velocity_y += filteredY * dt;
        } else {
            reset_velocity();
        }

        // device is still
        float accel_change = fabs(filteredX - prev_filtered_x) + fabs(filteredY - prev_filtered_y);

        if (fabs(filteredX) < threshold) filteredX = 0;
        if (fabs(filteredY) < threshold) filteredY = 0;
        if (fabs(filteredZ - 1.0f) < threshold) filteredZ = 1.0f;

        velocity_z += filteredZ * dt;

        if (accel_change < STILLNESS_THRESHOLD) {
            still_timer += dt;
            if (still_timer > STILL_TIME_THRESHOLD) {
                calibrate_offset();
                reset_velocity();
                still_timer = 0;
            }
        } else {
            still_timer = 0;
        }

        prev_filtered_x = filteredX;
        prev_filtered_y = filteredY;
        prev_filtered_z = filteredZ;
				
				float speed_kmh = sqrt(velocity_x * velocity_x + velocity_y * velocity_y) * 3.6f;
				float speed_mph = speed_kmh / 1.609f;
				if (HAL_GetTick() - lastPrintTime >= PRINT_INTERVAL) {
            lastPrintTime = HAL_GetTick(); 
						sprintf(message, "Speed: %.2f km/h (%.2f mph)\n", speed_kmh, speed_mph);
						print_msg(message);
				}
    }
}
