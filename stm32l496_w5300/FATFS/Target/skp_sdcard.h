#ifndef __SDCARD_H
#define __SDCARD_H

#ifdef __cplusplus
extern "C" {
#endif
#include <stdbool.h>
#include "main.h"
#include "ff.h"
#include "general_sdcard.h"
#include "toString.h"
/**
 * @brief 
 * 
 */
typedef struct
{
	char WiFi_SSID[32];
	char WiFi_PW[32];
	uint8_t wificonfig;
	uint8_t mqttconfig1;
	uint8_t mqttconfig2;
	uint8_t sslconfig1;
	uint8_t sslconfig2;	
}STATIC_INFO;

void testFunc(void);
	
void storeStaticInfoToStruct(STATIC_INFO *staticInfo, char** infoVariable);
void structToStr(STATIC_INFO *staticInfo, char* writeBuffer, char** pCurrent);


#endif
