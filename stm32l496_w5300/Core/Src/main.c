/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "fatfs.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "socket.h"
#include "wizchip_conf.h"
#include "w5300.h"
#include "ftpd.h"
#include "fatfs_sd.h"
#include "skp_sdcard.h"

#include "ff.h"
#include "general_sdcard.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

#define _USE_W5300_OPTIMIZE				1

#define W5300_BANK_ADDR                 ((uint32_t)0x60000000)//((uint32_t)0x64000000)
#define _W5300_DATA(p)                  *(__IO uint16_t *)(W5300_BANK_ADDR + (p<<1))//(*(volatile unsigned short*) (W5300_BANK_ADDR + (p<<1)))


#define RESET_W5300_GPIO_Port			GPIOC
#define RESET_W5300_Pin					GPIO_PIN_9

#define true					1
#define false					0

#define SOCK_TCPS       0
#define SOCK_UDPS       1
#define PORT_TCPS		5000
#define PORT_UDPS       3000

#define HSPI_SDCARD		 	&hspi1
#define	SD_CS_PORT			GPIOA
#define SD_CS_PIN			GPIO_PIN_4

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef hlpuart1;

RTC_HandleTypeDef hrtc;

SPI_HandleTypeDef hspi1;

SRAM_HandleTypeDef hsram1;

/* USER CODE BEGIN PV */
//uint8_t gDATABUF[DATA_BUF_SIZE];
uint8_t gFTPBUF[_MAX_SS];
uint8_t wiznet_memsize[2][8] = {{8,8,8,8,8,8,8,8}, {8,8,8,8,8,8,8,8}};

#define ETH_MAX_BUF_SIZE		2048

uint8_t ethBuf0[ETH_MAX_BUF_SIZE];


wiz_NetInfo gWIZNETINFO = {
		.mac = {0x00, 0x08, 0xdc, 0x11, 0x22, 0x33},
		.ip = {192, 168, 15, 111},
		.sn = {255, 255, 255, 0},
		.gw = {192, 168, 15, 1},
		.dns = {8, 8, 8, 8},
		.dhcp = NETINFO_STATIC
};


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_LPUART1_UART_Init(void);
static void MX_FMC_Init(void);
static void MX_RTC_Init(void);
static void MX_SPI1_Init(void);
/* USER CODE BEGIN PFP */
uint8_t rxData[2];

int _write(int fd, char *str, int len)
{
	for(int i=0; i<len; i++)
	{
		HAL_UART_Transmit(&hlpuart1, (uint8_t *)&str[i], 1, 0xFFFF);
	}
	return len;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {

    /*
        This will be called once data is received successfully,
        via interrupts.
    */

     /*
       loop back received data
     */
     HAL_UART_Receive_IT(&hlpuart1, rxData, 1);
     HAL_UART_Transmit(&hlpuart1, rxData, 1, 1000);
}
void Reset_W5300()
{
	HAL_GPIO_WritePin(RESET_W5300_GPIO_Port, RESET_W5300_Pin, GPIO_PIN_RESET);
	HAL_Delay(50);
	HAL_GPIO_WritePin(RESET_W5300_GPIO_Port, RESET_W5300_Pin, GPIO_PIN_SET);
	HAL_Delay(500);
}

void W5300_write(uint32_t addr, iodata_t wd)
{
	_W5300_DATA(addr) = wd;
}

iodata_t W5300_read(uint32_t addr)
{
	return _W5300_DATA(addr);
}
void W5300CsEnable(void)
{

}
void W5300CsDisable(void)
{

}
void print_network_information(void)
{
    wizchip_getnetinfo(&gWIZNETINFO);
    printf("Mac address: %02x:%02x:%02x:%02x:%02x:%02x\n\r",gWIZNETINFO.mac[0],gWIZNETINFO.mac[1],gWIZNETINFO.mac[2],gWIZNETINFO.mac[3],gWIZNETINFO.mac[4],gWIZNETINFO.mac[5]);
    printf("IP address : %d.%d.%d.%d\n\r",gWIZNETINFO.ip[0],gWIZNETINFO.ip[1],gWIZNETINFO.ip[2],gWIZNETINFO.ip[3]);
    printf("SM Mask    : %d.%d.%d.%d\n\r",gWIZNETINFO.sn[0],gWIZNETINFO.sn[1],gWIZNETINFO.sn[2],gWIZNETINFO.sn[3]);
    printf("Gate way   : %d.%d.%d.%d\n\r",gWIZNETINFO.gw[0],gWIZNETINFO.gw[1],gWIZNETINFO.gw[2],gWIZNETINFO.gw[3]);
    printf("DNS Server : %d.%d.%d.%d\n\r",gWIZNETINFO.dns[0],gWIZNETINFO.dns[1],gWIZNETINFO.dns[2],gWIZNETINFO.dns[3]);
}

void _InitW5300(void)
{
	unsigned int tmpaddr[4];

	Reset_W5300();
	reg_wizchip_bus_cbfunc(W5300_read, W5300_write);
	reg_wizchip_cs_cbfunc(W5300CsEnable, W5300CsDisable);
	printf("getMR() = %04X\r\n", getMR());

	if (ctlwizchip(CW_INIT_WIZCHIP, (void*)wiznet_memsize) == -1)
	{
		printf("W5300 memory initialization failed\r\n");
	}

	ctlnetwork(CN_SET_NETINFO, (void *)&gWIZNETINFO);
	print_network_information();
}

int test_SD_Card(void)
{
  FATFS fs;
	FILINFO finfo;
  mountSD(&fs);

  printf("----------------------------------------------\r\n");
  printf("	 SPI SD Card INFO File Read/Write Test \r\n");
  printf("----------------------------------------------\r\n");

  char filename[] = "test.txt";
  char testbuffer[] = "test 1234567890 -> result\r\ntest";
  char *readbuffer = NULL;
  int result = existFile(filename, &finfo);
  if(result != FR_OK){
		printf("Do not Exist[%s][%d].\r\n", filename, result);
		if(writeInfoToSD(filename, testbuffer) == false)
    {
      printf(" file Write Fail \r\n");
      return -1;
    }
      
    if(readInfoFromSD(filename, &readbuffer) == false)
    {
      printf(" file Read Fail \r\n");
      return -2;
    }
    if(readbuffer == NULL)
      printf("read Error \r\n");
    else
    {
      printf("read : %s\r\n", readbuffer);
      free(readbuffer);
    }
	}
	else{
		printf("Do Exist[%s].\r\n", filename);
		if(readInfoFromSD(filename, &readbuffer) == false)
    {
      printf(" file Read Fail \r\n");
      return -2;
    }
    if(readbuffer == NULL)
      printf("read Error \r\n");
    else
    {
      printf("read : %s\r\n", readbuffer);
      free(readbuffer);
    }
      
	}
  return 0;
}
int test_SD_Card1(void)
{
  FATFS fs;
  FATFS *pfs;
  FIL fil;
  FRESULT fres;
  DWORD fre_clust;
  uint32_t totalSpace, freeSpace;
  char buffer[100];
  HAL_Delay(500);
  /* Mount SD Card */
	if(f_mount(&fs, "", 0) != FR_OK)
		printf("ERROR f:%s, L:%d\r\n", __FILE__, __LINE__);

	/* Open file to write */
	if(f_open(&fil, "first.txt", FA_OPEN_ALWAYS | FA_READ | FA_WRITE) != FR_OK)
		printf("ERROR f:%s, L:%d\r\n", __FILE__, __LINE__);

	/* Check freeSpace space */
	if(f_getfree("", &fre_clust, &pfs) != FR_OK)
		printf("ERROR f:%s, L:%d\r\n", __FILE__, __LINE__);

	totalSpace = (uint32_t)((pfs->n_fatent - 2) * pfs->csize * 0.5);
	freeSpace = (uint32_t)(fre_clust * pfs->csize * 0.5);

	/* free space is less than 1kb */
	if(freeSpace < 1)
		printf("ERROR f:%s, L:%d\r\n", __FILE__, __LINE__);

	/* Writing text */
	f_puts("STM32 SD Card I/O Example via SPI\n", &fil);
	f_puts("Save the world!!!", &fil);

	/* Close file */
	if(f_close(&fil) != FR_OK)
		printf("ERROR f:%s, L:%d\r\n", __FILE__, __LINE__);

	/* Open file to read */
	if(f_open(&fil, "first.txt", FA_READ) != FR_OK)
		printf("ERROR f:%s, L:%d\r\n", __FILE__, __LINE__);

	while(f_gets(buffer, sizeof(buffer), &fil))
	{
		/* SWV output */
		printf("%s", buffer);
		fflush(stdout);
	}

	/* Close file */
	if(f_close(&fil) != FR_OK)
		printf("ERROR f:%s, L:%d\r\n", __FILE__, __LINE__);

	/* Unmount SDCARD */
	if(f_mount(NULL, "", 1) != FR_OK)
		printf("ERROR f:%s, L:%d\r\n", __FILE__, __LINE__);
}
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  char ret = 0;
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
  MX_LPUART1_UART_Init();
  MX_FMC_Init();
  MX_RTC_Init();
  MX_SPI1_Init();
  MX_FATFS_Init();
  /* USER CODE BEGIN 2 */
  //printf("Hello! W5300 BUS loopback System \r\n");
  HAL_UART_Receive_IT(&hlpuart1, rxData, 1);

  _InitW5300();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  printf("=======================================\r\n");
  printf(">> W5300 based FTP Server Example\r\n");
  printf("=======================================\r\n");
  ftpd_init(gWIZNETINFO.ip);
  //testFunc();
  //test_SD_Card();
  //test_SD_Card1();
  while (1)
  {
	 //loopback_tcps(0, ethBuf0, 3000);
	if((ret = ftpd_run(gFTPBUF)) < 0)
	{
		porintf(" FTP server Error %d \r\n", ret);
	}

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 80;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV4;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief LPUART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_LPUART1_UART_Init(void)
{

  /* USER CODE BEGIN LPUART1_Init 0 */

  /* USER CODE END LPUART1_Init 0 */

  /* USER CODE BEGIN LPUART1_Init 1 */

  /* USER CODE END LPUART1_Init 1 */
  hlpuart1.Instance = LPUART1;
  hlpuart1.Init.BaudRate = 115200;
  hlpuart1.Init.WordLength = UART_WORDLENGTH_8B;
  hlpuart1.Init.StopBits = UART_STOPBITS_1;
  hlpuart1.Init.Parity = UART_PARITY_NONE;
  hlpuart1.Init.Mode = UART_MODE_TX_RX;
  hlpuart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  hlpuart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  hlpuart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&hlpuart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN LPUART1_Init 2 */

  /* USER CODE END LPUART1_Init 2 */

}

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */

  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutRemap = RTC_OUTPUT_REMAP_NONE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

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
  hspi1.Init.DataSize = SPI_DATASIZE_4BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/* FMC initialization function */
static void MX_FMC_Init(void)
{

  /* USER CODE BEGIN FMC_Init 0 */

  /* USER CODE END FMC_Init 0 */

  FMC_NORSRAM_TimingTypeDef Timing = {0};

  /* USER CODE BEGIN FMC_Init 1 */

  /* USER CODE END FMC_Init 1 */

  /** Perform the SRAM1 memory initialization sequence
  */
  hsram1.Instance = FMC_NORSRAM_DEVICE;
  hsram1.Extended = FMC_NORSRAM_EXTENDED_DEVICE;
  /* hsram1.Init */
  hsram1.Init.NSBank = FMC_NORSRAM_BANK1;
  hsram1.Init.DataAddressMux = FMC_DATA_ADDRESS_MUX_DISABLE;
  hsram1.Init.MemoryType = FMC_MEMORY_TYPE_SRAM;
  hsram1.Init.MemoryDataWidth = FMC_NORSRAM_MEM_BUS_WIDTH_16;
  hsram1.Init.BurstAccessMode = FMC_BURST_ACCESS_MODE_DISABLE;
  hsram1.Init.WaitSignalPolarity = FMC_WAIT_SIGNAL_POLARITY_LOW;
  hsram1.Init.WaitSignalActive = FMC_WAIT_TIMING_BEFORE_WS;
  hsram1.Init.WriteOperation = FMC_WRITE_OPERATION_ENABLE;
  hsram1.Init.WaitSignal = FMC_WAIT_SIGNAL_DISABLE;
  hsram1.Init.ExtendedMode = FMC_EXTENDED_MODE_DISABLE;
  hsram1.Init.AsynchronousWait = FMC_ASYNCHRONOUS_WAIT_DISABLE;
  hsram1.Init.WriteBurst = FMC_WRITE_BURST_DISABLE;
  hsram1.Init.ContinuousClock = FMC_CONTINUOUS_CLOCK_SYNC_ONLY;
  hsram1.Init.WriteFifo = FMC_WRITE_FIFO_ENABLE;
  hsram1.Init.PageSize = FMC_PAGE_SIZE_NONE;
  /* Timing */
  Timing.AddressSetupTime = 15;
  Timing.AddressHoldTime = 15;
  Timing.DataSetupTime = 255;
  Timing.BusTurnAroundDuration = 15;
  Timing.CLKDivision = 16;
  Timing.DataLatency = 17;
  Timing.AccessMode = FMC_ACCESS_MODE_A;
  /* ExtTiming */

  if (HAL_SRAM_Init(&hsram1, &Timing, NULL) != HAL_OK)
  {
    Error_Handler( );
  }

  /* USER CODE BEGIN FMC_Init 2 */

  /* USER CODE END FMC_Init 2 */
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
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  HAL_PWREx_EnableVddIO2();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LD3_Pin|LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(USB_PowerSwitchOn_GPIO_Port, USB_PowerSwitchOn_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pins : PE2 PE3 PE4 PE5 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PA4 */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : LD3_Pin LD2_Pin */
  GPIO_InitStruct.Pin = LD3_Pin|LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_OverCurrent_Pin */
  GPIO_InitStruct.Pin = USB_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USB_OverCurrent_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_PowerSwitchOn_Pin */
  GPIO_InitStruct.Pin = USB_PowerSwitchOn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(USB_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PC9 */
  GPIO_InitStruct.Pin = GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void SELECT(void)
{
	HAL_GPIO_WritePin(SD_CS_PORT, SD_CS_PIN, GPIO_PIN_RESET);
	HAL_Delay(1);
}

/* slave deselect */
void DESELECT(void)
{
	HAL_GPIO_WritePin(SD_CS_PORT, SD_CS_PIN, GPIO_PIN_SET);
	HAL_Delay(1);
}

/* SPI transmit a byte */
void SPI_TxByte(uint8_t data)
{
	//uint8_t dummy = 0xff;
	while(!__HAL_SPI_GET_FLAG(HSPI_SDCARD, SPI_FLAG_TXE));
	HAL_SPI_Transmit(HSPI_SDCARD, &data, 1, SPI_TIMEOUT);
	//HAL_SPI_TransmitReceive(HSPI_SDCARD, &data, &dummy, 1, SPI_TIMEOUT);
}

/* SPI transmit buffer */
void SPI_TxBuffer(uint8_t *buffer, uint16_t len)
{
	//uint8_t dummy = 0xff;
	while(!__HAL_SPI_GET_FLAG(HSPI_SDCARD, SPI_FLAG_TXE));
	HAL_SPI_Transmit(HSPI_SDCARD, buffer, len, SPI_TIMEOUT);
	//HAL_SPI_TransmitReceive(HSPI_SDCARD, buffer, &dummy, len, SPI_TIMEOUT);
}

/* SPI receive a byte */
uint8_t SPI_RxByte(void)
{
	uint8_t dummy= 0xFF, data = 0x00;
	//dummy = 0xFF;

	while(!__HAL_SPI_GET_FLAG(HSPI_SDCARD, SPI_FLAG_TXE));
	//while(!__HAL_SPI_GET_FLAG(HSPI_SDCARD, SPI_FLAG_RXNE));
	HAL_SPI_TransmitReceive(HSPI_SDCARD, &dummy, &data, 1, SPI_TIMEOUT);

	return data;
}

/* SPI receive a byte via pointer */
void SPI_RxBytePtr(uint8_t *buff) 
{
	*buff = SPI_RxByte();
}
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
