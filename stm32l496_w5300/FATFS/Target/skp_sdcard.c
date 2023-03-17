#include "skp_sdcard.h"
#include "main.h"
#include <stdio.h>
#include <string.h>


/*********************************************************/
/*                          TEST Function                                  */
/*********************************************************/

void testFunc(void)
{
	FATFS fs;
	FILINFO finfo;


  char *staticInfoFileName = "static4.txt";
  char *staticInfoStr;
  char writeBuffer[69]={0,};
  char *pCurrent;

  char *firstCertName = "91375cab42-private.pem.key";
  char *firstCert;

  char *secondCertName = "testtest.key";
  char *secondCert;


  STATIC_INFO staticInfo = {"SSID1", "PW1", 112, 1, 2, 3, 4};

	/* Mount SD Card */
	mountSD(&fs);

	printf("------------------------------------------\r\n");
  printf("		   SPI SD Card INFO File Read/Write Test \r\n");
  printf("------------------------------------------\r\n");
	

	
	int result = existFile(staticInfoFileName, &finfo);
	if(result != FR_OK){
		printf("Do not Exist.\r\n");
		structToStr(&staticInfo, writeBuffer, &pCurrent);
		writeInfoToSD(staticInfoFileName, writeBuffer);
	}
	else{
		printf("Do Exist.\r\n");
		readInfoFromSD(staticInfoFileName,  &staticInfoStr);
		storeStaticInfoToStruct(&staticInfo, &staticInfoStr);
	}
	
	printf("[Static Info Print]\r\n");
	printf("SSID: %s\r\n", staticInfo.WiFi_SSID);
	printf("PW: %s\r\n", staticInfo.WiFi_PW);
	printf("wificonfig: %d\r\n", staticInfo.wificonfig);
	printf("mqttconfig1: %d\r\n", staticInfo.mqttconfig1);
	printf("mqttconfig2: %d\r\n", staticInfo.mqttconfig2);
	printf("sslconfig1: %d\r\n", staticInfo.sslconfig1);
	printf("sslconfig2: %d\r\n", staticInfo.sslconfig2);
	
	
  printf("------------------------------------------\r\n");
  printf("		   SPI SD Card CERT File Read Test \r\n");
  printf("------------------------------------------\r\n");

	/* Read Certification from SD Card */
	result = existFile(staticInfoFileName, &finfo);
	if(result != FR_OK){
		printf("Do not Exist.\r\n");
	}
	else{
		printf("Do Exist.\r\n");
		readInfoFromSD(firstCertName,  &firstCert);
	  printf("[firstCert]\r\n%s\r\n", firstCert);
	}
	
	result = existFile(staticInfoFileName, &finfo);
	if(result != FR_OK){
		printf("Do not Exist.\r\n");
	}
	else{
		printf("Do Exist.\r\n");
		readInfoFromSD(secondCertName, &secondCert);
	  printf("[secondCertName]\r\n%s\r\n", secondCert);
	}
	
	/* Free Cert  */
  free(firstCert);
	free(secondCertName);
	

  /* Unmount SDCARD */
  unmountSD();
}

/*********************************************************/
/*                          SKP Static Info Parsing                                  */
/*********************************************************/

void structToStr(STATIC_INFO *staticInfo, char* writeBuffer, char** pCurrent)
{
  *pCurrent = writeBuffer;	
	addStrValToString(staticInfo->WiFi_SSID, writeBuffer, pCurrent);
	addStrValToString(staticInfo->WiFi_PW, writeBuffer, pCurrent);
	addIntValToString(&staticInfo->wificonfig, writeBuffer, pCurrent);
	addIntValToString(&staticInfo->mqttconfig1, writeBuffer, pCurrent);
	addIntValToString(&staticInfo->mqttconfig2, writeBuffer, pCurrent);
	addIntValToString(&staticInfo->sslconfig1, writeBuffer, pCurrent);
	addIntValToString(&staticInfo->sslconfig2, writeBuffer, pCurrent);
}



void storeStaticInfoToStruct(STATIC_INFO *staticInfo, char** infoVariable)
{
	
	//printf("infoVariable is : \r\n%s\n", *infoVariable);
	
	char *ptr = strtok(*infoVariable, "\n"); 
	strcpy(staticInfo->WiFi_SSID, ptr);
	//printf("%s\n", ptr);          // �ڸ� ���ڿ� ���
	
  ptr = strtok(NULL, "\n");      
	strcpy(staticInfo->WiFi_PW, ptr); 
	
	ptr = strtok(NULL, "\n");     
	staticInfo->wificonfig = atoi(ptr);
	
	ptr = strtok(NULL, "\n");     
	staticInfo->mqttconfig1 = atoi(ptr);
	
	ptr = strtok(NULL, "\n");    
	staticInfo->mqttconfig2= atoi(ptr);
	
	ptr = strtok(NULL, "\n");      
	staticInfo->sslconfig1= atoi(ptr);
	
	ptr = strtok(NULL, "\n");      
	staticInfo->sslconfig2= atoi(ptr);
	
}	
