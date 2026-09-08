#ifndef WM8960_H
#define WM8960_H
#include "stm32h7xx_hal.h"
#include <stdbool.h>

HAL_StatusTypeDef WM8960_Init(I2C_HandleTypeDef *hi2c);
bool WM8960_IsReady(I2C_HandleTypeDef *hi2c);

#endif
