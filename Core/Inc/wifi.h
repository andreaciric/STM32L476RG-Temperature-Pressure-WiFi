/*
 * wifi.h
 *
 *  Created on: Sep 22, 2024
 *      Author: AndreaCiric
 */

#ifndef INC_WIFI_H_
#define INC_WIFI_H_

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stdint.h"
#include "main.h"

/* Macro Definitions ---------------------------------------------------------*/
#define WIFI_CMD1	"AT+CWMODE=3\r\n"  // Set WiFi mode
#define WIFI_CMD2	"AT+CWJAP=\"Aurora Borealis\",\"hicsuntdragones\"\r\n"  // Connect to WiFi
#define WIFI_CMD3	"AT+MQTTUSERCFG=0,1,\"clientId-bmx49KmLGA\",\"irs2\",\"123456\",0,0,\"\"\r\n"  // MQTT user configuration
#define WIFI_CMD4	"AT+MQTTCONN=0,\"mqtt-dashboard.com\",1883,1\r\n"  // MQTT broker connection

#define WIFI_MQTT_PUB_FORMAT "AT+MQTTPUB=0,\"irs2project\",\"Temperature: %d Pressure: %d\",1,0\r\n"

/* Function Prototypes ------------------------------------------------------- */
HAL_StatusTypeDef WIFI_Setup(void);
HAL_StatusTypeDef WIFI_Send_Data(float temperature, float pressure);

#ifdef __cplusplus
}
#endif

#endif /* INC_WIFI_H_ */
