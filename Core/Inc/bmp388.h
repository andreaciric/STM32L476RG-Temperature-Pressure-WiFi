/*
 * bmp388.h
 *
 *  Created on: Sep 22, 2024
 *      Author: AndreaCiric
 */

#ifndef INC_BMP388_H_
#define INC_BMP388_H_

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stdint.h"
#include "main.h"

/* Type Definitions ----------------------------------------------------------*/

typedef struct {
    float PAR_T1;
    float PAR_T2;
    float PAR_T3;
    float PAR_P1;
    float PAR_P2;
    float PAR_P3;
    float PAR_P4;
    float PAR_P5;
    float PAR_P6;
    float PAR_P7;
    float PAR_P8;
    float PAR_P9;
    float PAR_P10;
    float PAR_P11;
    float t_lin;
} BMP388_Calib_Data;

/* Macro Definitions ---------------------------------------------------------*/
#define BMP388_CHIP_ID_REG        0x00
#define BMP388_CALIB_DATA_START   0x31
#define BMP388_PWR_CTRL_REG       0x1B
#define BMP388_OSR_REG            0x1C
#define BMP388_CMD_REG            0x7E
#define BMP388_PRESS_REG          0x04
#define BMP388_TEMP_REG           0x07

#define BMP388_CHIP_ID            0x50
#define BMP388_SOFT_RESET_CMD     0xB6
#define BMP388_PWR_CTRL			  0x33
#define BMP388_OSR				  0x00

#define BMP388_READ_MASK          0x80
#define BMP388_WRITE_MASK         0x7F

/* Function Prototypes ------------------------------------------------------- */
HAL_StatusTypeDef BMP388_Init(void);
HAL_StatusTypeDef BMP388_Read_Calibration_Data(void);
HAL_StatusTypeDef BMP388_Read_Reg(uint8_t addr, uint8_t *value);
HAL_StatusTypeDef BMP388_Read_Regs(uint8_t addr, uint8_t *data, uint8_t len);
HAL_StatusTypeDef BMP388_Write_Reg(uint8_t addr, uint8_t data);
uint32_t BMP388_Read_Raw_Temperature(void);
uint32_t BMP388_Read_Raw_Pressure(void);
float BMP388_Compensate_Temperature(uint32_t raw_temp);
float BMP388_Compensate_Pressure(uint32_t raw_press);

/* External Variables --------------------------------------------------------*/
extern BMP388_Calib_Data calib_data;

#ifdef __cplusplus
}
#endif

#endif /* INC_BMP388_H_ */
