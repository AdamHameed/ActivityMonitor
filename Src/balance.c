#include "main.h"

#include <string.h>  
#include <stdlib.h> 
#include <math.h> 
#include <stdio.h>
#include <stdbool.h>
#include "stm32f4xx_hal.h"  

#define TILT_THRESHOLD 0.3f 
#define GAME_OVER_THRESHOLD 0.4 // How long tilt is allowed in seconds before losing game
#define UPDATE_INTERVAL 100   // Game loop length in ms

void PmodACL2_get_acceleration(float *x, float *y, float *z);
void print_msg(char * msg);

void play_balancing_game() {
    static uint32_t lastUpdateTime = 0;
    static float tiltDuration = 0.0f;
    static bool gameOver = false;
    static bool gameStarted = false;
    static bool inCountdown = true;
    static uint32_t countdownStartTime = 0;
    const uint32_t COUNTDOWN_TIME = 3000;
		static uint32_t lastWarningTime = 0;
		const uint32_t WARNING_INTERVAL = 70; 

    if (!gameStarted || gameOver) {
        print_msg("Balance training game starting in 3 seconds... Hold as still as possible!\n");
        
        // reset all state variables
        gameStarted = true;
        gameOver = false;
        inCountdown = true;
        tiltDuration = 0.0f;
        countdownStartTime = HAL_GetTick();  
    }

    // countdown
    if (inCountdown) {
        uint32_t elapsedTime = HAL_GetTick() - countdownStartTime;
        if (elapsedTime >= COUNTDOWN_TIME) {
            print_msg("Balance training game started!\n");
            inCountdown = false;  
        }
        return;
    }

    if (gameOver) return;

    if (HAL_GetTick() - lastUpdateTime >= UPDATE_INTERVAL) {
        lastUpdateTime = HAL_GetTick();

        float xAccel, yAccel, zAccel;
        PmodACL2_get_acceleration(&xAccel, &yAccel, &zAccel);

        float tilt = xAccel * xAccel + yAccel * yAccel;

       if (tilt > TILT_THRESHOLD) {
					tiltDuration += (float)UPDATE_INTERVAL / 1000.0f;
					if (HAL_GetTick() - lastWarningTime >= WARNING_INTERVAL) {
							print_msg("Balance yourself! Tilt detected!\n");
							lastWarningTime = HAL_GetTick();
					}
			} else {
					tiltDuration = 0.0f;
			}

        if (tiltDuration >= GAME_OVER_THRESHOLD) {
            gameOver = true;
            print_msg("You lost balance, game over!\n");
						HAL_Delay(3000);
        }
				
    }
}


