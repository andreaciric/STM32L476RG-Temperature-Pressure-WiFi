/*
 * wifi.c
 *
 *  Created on: Sep 22, 2024
 *      Author: AndreaCiric
 */

/* Includes ------------------------------------------------------------------*/
#include "wifi.h"
#include <string.h>
#include <stdio.h>

/* External Variables --------------------------------------------------------*/
extern UART_HandleTypeDef huart3;  // Adjust if using a different UART instance

/* Private Functions ---------------------------------------------------------*/

static HAL_StatusTypeDef WIFI_Send_Command(const char* cmd, uint32_t delay_ms)
{
    HAL_StatusTypeDef status;

    status = HAL_UART_Transmit(&huart3, (uint8_t*)cmd, strlen(cmd), HAL_MAX_DELAY);
    if (status != HAL_OK)
    {
        return status;
    }

    HAL_Delay(delay_ms);
    return HAL_OK;
}

/* Function Implementations --------------------------------------------------*/

HAL_StatusTypeDef WIFI_Setup(void)
{
    HAL_StatusTypeDef status;

    // Set WiFi mode
    status = WIFI_Send_Command(WIFI_CMD1, 2000);
    if (status != HAL_OK) return status;

    // Connect to WiFi
    status = WIFI_Send_Command(WIFI_CMD2, 5000);
    if (status != HAL_OK) return status;

    // Configure MQTT user
    status = WIFI_Send_Command(WIFI_CMD3, 2000);
    if (status != HAL_OK) return status;

    // Connect to MQTT broker
    status = WIFI_Send_Command(WIFI_CMD4, 5000);
    if (status != HAL_OK) return status;

    return HAL_OK;
}

HAL_StatusTypeDef WIFI_Send_Data(float temperature, float pressure)
{
    HAL_StatusTypeDef status;
    char tx_buffer[100] = {0};

    // Format the MQTT publish command with temperature and pressure
    snprintf(tx_buffer, sizeof(tx_buffer), WIFI_MQTT_PUB_FORMAT, (int)temperature, (int)pressure);

    // Send the MQTT publish command
    status = HAL_UART_Transmit(&huart3, (uint8_t*)tx_buffer, strlen(tx_buffer), HAL_MAX_DELAY);
    if (status != HAL_OK) return status;

    HAL_Delay(5000);

    return HAL_OK;
}

