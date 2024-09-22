/*
 * bmp388.c
 *
 *  Created on: Sep 22, 2024
 *      Author: AndreaCiric
 */

/* Includes ------------------------------------------------------------------*/
#include "bmp388.h"
#include <math.h>

/* External Variables --------------------------------------------------------*/
extern SPI_HandleTypeDef hspi3;
extern GPIO_TypeDef* BMP388_CS_GPIO_Port;
extern uint16_t BMP388_CS_Pin;

#ifndef BMP388_CS_GPIO_Port
#define BMP388_CS_GPIO_Port GPIOD
#endif

#ifndef BMP388_CS_Pin
#define BMP388_CS_Pin GPIO_PIN_2
#endif

/* Global Variables ----------------------------------------------------------*/
BMP388_Calib_Data calib_data = {0};

/* Private Functions ---------------------------------------------------------*/

static void BMP388_Select(void)
{
    HAL_GPIO_WritePin(BMP388_CS_GPIO_Port, BMP388_CS_Pin, GPIO_PIN_RESET);
}

static void BMP388_Deselect(void)
{
    HAL_GPIO_WritePin(BMP388_CS_GPIO_Port, BMP388_CS_Pin, GPIO_PIN_SET);
}

/* Function Implementations --------------------------------------------------*/

HAL_StatusTypeDef BMP388_Read_Reg(uint8_t addr, uint8_t *value)
{
    HAL_StatusTypeDef status;
    uint8_t tx_data[2] = {0};
    uint8_t rx_data[2] = {0};

    tx_data[0] = addr | BMP388_READ_MASK;
    tx_data[1] = 0x00;

    BMP388_Select();

    status = HAL_SPI_Transmit(&hspi3, tx_data, 2, HAL_MAX_DELAY);
    if (status != HAL_OK)
    {
        BMP388_Deselect();
        return status;
    }

    status = HAL_SPI_Receive(&hspi3, rx_data, 2, HAL_MAX_DELAY);
    BMP388_Deselect();

    if (status == HAL_OK)
    {
        *value = rx_data[0];
    }

    return status;
}

HAL_StatusTypeDef BMP388_Read_Regs(uint8_t addr, uint8_t *data, uint8_t len)
{
    HAL_StatusTypeDef status;
    uint8_t tx_data[2] = {0};

    tx_data[0] = addr | BMP388_READ_MASK;
    tx_data[1] = 0x00;

    BMP388_Select();

    status = HAL_SPI_Transmit(&hspi3, tx_data, 2, HAL_MAX_DELAY);
    if (status != HAL_OK)
    {
        BMP388_Deselect();
        return status;
    }

    status = HAL_SPI_Receive(&hspi3, data, len, HAL_MAX_DELAY);
    BMP388_Deselect();

    return status;
}

HAL_StatusTypeDef BMP388_Write_Reg(uint8_t addr, uint8_t data)
{
    HAL_StatusTypeDef status;
    uint8_t tx_data[2] = {0};

    tx_data[0] = addr & BMP388_WRITE_MASK;
    tx_data[1] = data;

    BMP388_Select();
    status = HAL_SPI_Transmit(&hspi3, tx_data, 2, HAL_MAX_DELAY);
    if (status != HAL_OK) return status;
    BMP388_Deselect();

    return status;
}

HAL_StatusTypeDef BMP388_Read_Calibration_Data(void)
{
    HAL_StatusTypeDef status;
    uint8_t calib_buffer[21] = {0};

    status = BMP388_Read_Regs(BMP388_CALIB_DATA_START, calib_buffer, 21);
    if (status != HAL_OK) return status;

    uint16_t NVM_PAR_T1 = (uint16_t)((calib_buffer[1] << 8) | calib_buffer[0]);
    uint16_t NVM_PAR_T2 = (uint16_t)((calib_buffer[3] << 8) | calib_buffer[2]);
    int8_t  NVM_PAR_T3 = (int8_t)calib_buffer[4];

    int16_t NVM_PAR_P1 = (int16_t)((calib_buffer[6] << 8) | calib_buffer[5]);
    int16_t NVM_PAR_P2 = (int16_t)((calib_buffer[8] << 8) | calib_buffer[7]);
    int8_t  NVM_PAR_P3 = (int8_t)calib_buffer[9];
    int8_t  NVM_PAR_P4 = (int8_t)calib_buffer[10];
    uint16_t NVM_PAR_P5 = (uint16_t)((calib_buffer[12] << 8) | calib_buffer[11]);
    uint16_t NVM_PAR_P6 = (uint16_t)((calib_buffer[14] << 8) | calib_buffer[13]);
    int8_t  NVM_PAR_P7 = (int8_t)calib_buffer[15];
    int8_t  NVM_PAR_P8 = (int8_t)calib_buffer[16];
    int16_t NVM_PAR_P9 = (int16_t)((calib_buffer[18] << 8) | calib_buffer[17]);
    int8_t  NVM_PAR_P10 = (int8_t)calib_buffer[19];
    int8_t  NVM_PAR_P11 = (int8_t)calib_buffer[20];

    calib_data.PAR_T1 = ((float)NVM_PAR_T1) / powf(2.0f, -8.0f);
    calib_data.PAR_T2 = ((float)NVM_PAR_T2) / powf(2.0f, 30.0f);
    calib_data.PAR_T3 = ((float)NVM_PAR_T3) / powf(2.0f, 48.0f);

    calib_data.PAR_P1 = (((float)NVM_PAR_P1) - powf(2.0f, 14.0f)) / powf(2.0f, 20.0f);
    calib_data.PAR_P2 = (((float)NVM_PAR_P2) - powf(2.0f, 14.0f)) / powf(2.0f, 29.0f);
    calib_data.PAR_P3 = ((float)NVM_PAR_P3) / powf(2.0f, 32.0f);
    calib_data.PAR_P4 = ((float)NVM_PAR_P4) / powf(2.0f, 37.0f);
    calib_data.PAR_P5 = ((float)NVM_PAR_P5) / powf(2.0f, -3.0f);
    calib_data.PAR_P6 = ((float)NVM_PAR_P6) / powf(2.0f, 6.0f);
    calib_data.PAR_P7 = ((float)NVM_PAR_P7) / powf(2.0f, 8.0f);
    calib_data.PAR_P8 = ((float)NVM_PAR_P8) / powf(2.0f, 15.0f);
    calib_data.PAR_P9 = ((float)NVM_PAR_P9) / powf(2.0f, 48.0f);
    calib_data.PAR_P10 = ((float)NVM_PAR_P10) / powf(2.0f, 48.0f);
    calib_data.PAR_P11 = ((float)NVM_PAR_P11) / powf(2.0f, 65.0f);

    return status;
}

HAL_StatusTypeDef BMP388_Init(void)
{
    HAL_StatusTypeDef status;

    // Perform soft reset
    status = BMP388_Write_Reg(BMP388_CMD_REG, BMP388_SOFT_RESET_CMD);
    if (status != HAL_OK) return status;
    HAL_Delay(10);

    // Verify CHIP ID
    uint8_t chip_id = 0;
    status = BMP388_Read_Reg(BMP388_CHIP_ID_REG, &chip_id);
    while (status == HAL_OK && chip_id != BMP388_CHIP_ID)
    {
        HAL_Delay(10);
        status = BMP388_Read_Reg(BMP388_CHIP_ID_REG, &chip_id);
    }

    status = BMP388_Read_Calibration_Data();
    if (status != HAL_OK) return status;

    //uint8_t pwr_ctrl_readback = 0;
    //status = BMP388_Read_Reg(BMP388_PWR_CTRL_REG, &pwr_ctrl_readback);
    status = BMP388_Write_Reg(BMP388_PWR_CTRL_REG, BMP388_PWR_CTRL);
    if (status != HAL_OK) return status;
    //status = BMP388_Read_Reg(BMP388_PWR_CTRL_REG, &pwr_ctrl_readback);

    //uint8_t osr_readback = 0;
    //status = BMP388_Read_Reg(BMP388_OSR_REG, &osr_readback);
    status = BMP388_Write_Reg(BMP388_OSR_REG, BMP388_OSR);
    if (status != HAL_OK) return status;
    //status = BMP388_Read_Reg(BMP388_OSR_REG, &osr_readback);
    //status = BMP388_Read_Reg(BMP388_PWR_CTRL_REG, &pwr_ctrl_readback);

    return status;
}

uint32_t BMP388_Read_Raw_Temperature(void)
{
    uint8_t temp_data[3] = {0};
    uint32_t raw_temp = 0;

    HAL_StatusTypeDef status = BMP388_Read_Regs(BMP388_TEMP_REG, temp_data, 3);
    if (status != HAL_OK) return 0;

    raw_temp = ((uint32_t)temp_data[2] << 16) | ((uint32_t)temp_data[1] << 8) | temp_data[0];

    return raw_temp;
}

uint32_t BMP388_Read_Raw_Pressure(void)
{
    uint8_t press_data[3] = {0};
    uint32_t raw_press = 0;

    HAL_StatusTypeDef status = BMP388_Read_Regs(BMP388_PRESS_REG, press_data, 3);
    if (status != HAL_OK) return 0;

    raw_press = ((uint32_t)press_data[2] << 16) | ((uint32_t)press_data[1] << 8) | press_data[0];

    return raw_press;
}

float BMP388_Compensate_Temperature(uint32_t raw_temp)
{
    float partial_data1 = (float)(raw_temp - calib_data.PAR_T1);
    float partial_data2 = (float)(calib_data.PAR_T2 * partial_data1);

    calib_data.t_lin = partial_data2 + (partial_data1 * partial_data1) * calib_data.PAR_T3;

    return calib_data.t_lin;
}

float BMP388_Compensate_Pressure(uint32_t raw_press)
{
    float t_lin = calib_data.t_lin;

    float partial_data1 = calib_data.PAR_P6 * t_lin;
    float partial_data2 = calib_data.PAR_P7 * t_lin * t_lin;
    float partial_data3 = calib_data.PAR_P8 * t_lin * t_lin * t_lin;
    float partial_out1 = calib_data.PAR_P5 + partial_data1 + partial_data2 + partial_data3;

    partial_data1 = calib_data.PAR_P2 * t_lin;
    partial_data2 = calib_data.PAR_P3 * t_lin * t_lin;
    partial_data3 = calib_data.PAR_P4 * t_lin * t_lin * t_lin;
    float partial_out2 = ((float)raw_press) * (calib_data.PAR_P1 + partial_data1 + partial_data2 + partial_data3);

    partial_data1 = ((float)raw_press) * ((float)raw_press);
    partial_data2 = calib_data.PAR_P9 + calib_data.PAR_P10 * t_lin;
    partial_data3 = partial_data1 * partial_data2;
    float partial_data4 = partial_data3 + ((float)raw_press) * ((float)raw_press) * ((float)raw_press) * calib_data.PAR_P11;

    float pressure = partial_out1 + partial_out2 + partial_data4;

    return pressure;
}

