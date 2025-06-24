/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "string.h"
#include "math.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "tm_stm32f4_hd44780.h"
#include "keypad.h"
#include "stm32h7xx_hal_conf.h"
#include "main.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
#if defined ( __ICCARM__ ) /*!< IAR Compiler */
#pragma location=0x30000000
ETH_DMADescTypeDef  DMARxDscrTab[ETH_RX_DESC_CNT]; /* Ethernet Rx DMA Descriptors */
#pragma location=0x30000200
ETH_DMADescTypeDef  DMATxDscrTab[ETH_TX_DESC_CNT]; /* Ethernet Tx DMA Descriptors */

#elif defined ( __CC_ARM )  /* MDK ARM Compiler */

__attribute__((at(0x30000000))) ETH_DMADescTypeDef  DMARxDscrTab[ETH_RX_DESC_CNT]; /* Ethernet Rx DMA Descriptors */
__attribute__((at(0x30000200))) ETH_DMADescTypeDef  DMATxDscrTab[ETH_TX_DESC_CNT]; /* Ethernet Tx DMA Descriptors */

#elif defined ( __GNUC__ ) /* GNU Compiler */

ETH_DMADescTypeDef DMARxDscrTab[ETH_RX_DESC_CNT] __attribute__((section(".RxDecripSection"))); /* Ethernet Rx DMA Descriptors */
ETH_DMADescTypeDef DMATxDscrTab[ETH_TX_DESC_CNT] __attribute__((section(".TxDecripSection")));   /* Ethernet Tx DMA Descriptors */
#endif

ETH_TxPacketConfig TxConfig;

DAC_HandleTypeDef hdac1;
DMA_HandleTypeDef hdma_dac1_ch1;

ETH_HandleTypeDef heth;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim6;

UART_HandleTypeDef huart3;

PCD_HandleTypeDef hpcd_USB_OTG_FS;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_USB_OTG_FS_PCD_Init(void);
static void MX_ETH_Init(void);
static void MX_DAC1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM6_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

#define NS       128
#define TIM2CLK  100000000
#define MIN_SIGNAL 100
#define MAX_SIGNAL 2000
int F_SIGNAL;
uint32_t TIM2_Ticks = TIM2CLK / (NS * MIN_SIGNAL);

uint32_t sine_val[NS];
uint32_t triangle_val[NS];
uint32_t square_wave_val[NS];
uint32_t sawtooth_wave_val[NS];

uint32_t counter = 0; // Ορισμός αρχικής τιμής του counter
char frequency[10]; // Ορισμός του αλφαριθμητικού για την αποθήκευση της τιμής
char cnt_val[10]; // Αλφαριθμητικό για την αποθήκευση της τιμής του TIM4->CNT
char function[10];
uint16_t sineWave[128];


uint32_t sine_wave[NS] = {
		  2010,  2108,  2205,  2302,  2398,  2494,  2588,
		  2680,  2772,  2861,  2948,  3033,  3116,  3195,
		  3272,  3346,  3417,  3484,  3548,  3608,  3665,
		  3717,  3765,  3809,  3849,  3884,  3914,  3940,
		  3962,  3978,  3990,  3998,  4000,  3998,  3990,
		  3978,  3962,  3940,  3914,  3884,  3849,  3809,
		  3765,  3717,  3665,  3608,  3548,  3484,  3417,
		  3346,  3272,  3195,  3116,  3033,  2948,  2861,
		  2772,  2680,  2588,  2494,  2398,  2302,  2205,
		  2108,  2010,  1912,  1815,  1718,  1622,  1526,
		  1432,  1340,  1248,  1159,  1072,   987,   904,
		   825,   748,   674,   603,   536,   472,   412,
		   355,   303,   255,   211,   171,   136,   106,
		    80,    58,    42,    30,    22,    20,    22,
		    30,    42,    58,    80,   106,   136,   171,
		   211,   255,   303,   355,   412,   472,   536,
		   603,   674,   748,   825,   904,   987,  1072,
		  1159,  1248,  1340,  1432,  1526,  1622,  1718,
		  1815,  1912 };


uint32_t triangle_wave[NS] = {
		61, 122, 183, 244, 305, 366, 427, 488, 548, 609,
		670, 731, 792, 853, 914, 975, 1036, 1097, 1158, 1219,
		1280, 1341, 1402, 1463, 1523, 1584, 1645, 1706, 1767, 1828,
		1889, 1950, 2011, 2072, 2133, 2194, 2255, 2316, 2377, 2438,
		2498, 2559, 2620, 2681, 2742, 2803, 2864, 2925, 2986, 3047,
		3108, 3169, 3230, 3291, 3352, 3413, 3473, 3534, 3595, 3656,
		3717, 3778, 3839, 3900, 3839, 3778, 3717, 3656, 3595, 3534,
		3473, 3413, 3352, 3291, 3230, 3169, 3108, 3047, 2986, 2925,
		2864, 2803, 2742, 2681, 2620, 2559, 2498, 2438, 2377, 2316,
		2255, 2194, 2133, 2072, 2011, 1950, 1889, 1828, 1767, 1706,
		1645, 1584, 1523, 1463, 1402, 1341, 1280, 1219, 1158, 1097,
		1036, 975, 914, 853, 792, 731, 670, 609, 548, 488,
		427, 366, 305, 244, 183, 122, 61, 0};


uint32_t square_wave[NS] = {
	     0,     0,     0,     0,     0,     0,     0,
	     0,     0,     0,     0,     0,     0,     0,
	     0,     0,     0,     0,     0,     0,     0,
	     0,     0,     0,     0,     0,     0,     0,
	     0,     0,     0,     0,  4000,  4000,  4000,
	  4000,  4000,  4000,  4000,  4000,  4000,  4000,
	  4000,  4000,  4000,  4000,  4000,  4000,  4000,
	  4000,  4000,  4000,  4000,  4000,  4000,  4000,
	  4000,  4000,  4000,  4000,  4000,  4000,  4000,
	  4000,  4000,  4000,  4000,  4000,  4000,  4000,
	  4000,  4000,  4000,  4000,  4000,  4000,  4000,
	  4000,  4000,  4000,  4000,  4000,  4000,  4000,
	  4000,  4000,  4000,  4000,  4000,  4000,  4000,
	  4000,  4000,  4000,  4000,  4000,     0,     0,
	     0,     0,     0,     0,     0,     0,     0,
	     0,     0,     0,     0,     0,     0,     0,
	     0,     0,     0,     0,     0,     0,     0,
	     0,     0,     0,     0,     0,     0,     0,
	     0,     0 };


uint32_t sawtooth_wave[NS] = {
	     0,    30,    59,    89,   119,   148,   178,
	   208,   238,   267,   297,   327,   356,   386,
	   416,   445,   475,   505,   534,   564,   594,
	   623,   653,   683,   713,   742,   772,   802,
	   831,   861,   891,   920,   950,   980,  1009,
	  1039,  1069,  1098,  1128,  1158,  1188,  1217,
	  1247,  1277,  1306,  1336,  1366,  1395,  1425,
	  1455,  1484,  1514,  1544,  1573,  1603,  1633,
	  1663,  1692,  1722,  1752,  1781,  1811,  1841,
	  1870,  1900,  1930,  1959,  1989,  2019,  2048,
	  2078,  2108,  2137,  2167,  2197,  2227,  2256,
	  2286,  2316,  2345,  2375,  2405,  2434,  2464,
	  2494,  2523,  2553,  2583,  2612,  2642,  2672,
	  2702,  2731,  2761,  2791,  2820,  2850,  2880,
	  2909,  2939,  2969,  2998,  3028,  3058,  3087,
	  3117,  3147,  3177,  3206,  3236,  3266,  3295,
	  3325,  3355,  3384,  3414,  3444,  3473,  3503,
	  3533,  3562,  3592,  3622,  3652,  3681,  3711,
	  3741,  3770 };


void update_counter_and_signal() {
    counter = TIM4->CNT;

    if (counter >= 1900) {
        	__HAL_TIM_SET_COUNTER(&htim4, 0);
            counter = 0;
            //counter = TIM4->CNT;
            TIM4->CCR1 = counter;
        }

    F_SIGNAL = MIN_SIGNAL + counter;
    if (F_SIGNAL < MIN_SIGNAL || F_SIGNAL > MAX_SIGNAL) {
        counter = 0;
        F_SIGNAL = MIN_SIGNAL + counter;
    }

    TIM2_Ticks = TIM2CLK / (NS * F_SIGNAL);

	HAL_Delay(500);
    TIM2-> ARR = TIM2_Ticks - 1;

}


void Serial_Send(int freq)
{
    char buffer2[256];
    snprintf(buffer2, sizeof(buffer2), "Frequency: %4d. Select: '1' for sine wave, '2' for triangle wave, '3' for square wave, '4' for sawtooth wave.\r", freq);
    HAL_UART_Transmit(&huart3, (uint32_t*)buffer2, strlen(buffer2), HAL_MAX_DELAY);

    }


char Serial_Receive(void) {
    char received_char = '\0';
    HAL_StatusTypeDef status;

    // Receive data from UART
    status = HAL_UART_Receive(&huart3, (uint8_t*)&received_char, 1, 1000);
    if (status != HAL_OK) {
        return '\0';
    }

    return received_char;
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

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
  MX_DMA_Init();
  MX_USART3_UART_Init();
  MX_USB_OTG_FS_PCD_Init();
  MX_ETH_Init();
  MX_DAC1_Init();
  MX_TIM2_Init();
  MX_TIM1_Init();
  MX_TIM4_Init();
  MX_TIM6_Init();
  /* USER CODE BEGIN 2 */

  TM_HD44780_Init(16, 2);

  HAL_TIM_Base_Start(&htim2);

  HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  /** Amplitude Modulation Settings */

  //HAL_TIM_Base_Start(&htim6);

	while (1)
	{

	  char tempCharStore = KeypadGetKey();
      update_counter_and_signal() ;
      TM_HD44780_Puts(0, 0, "Sel. function: ");

      //sprintf(cnt_val, "%04lu", TIM4->CNT); // %lu για unsigned long int
      //Μετατροπή της τιμής του F_SIGNAL σε αλφαριθμητικό
      //Εμφάνιση της τιμής
      sprintf(frequency, "%04u", F_SIGNAL);
      TM_HD44780_Puts(0, 1, "Frequency: ");
      TM_HD44780_Puts(12, 1, frequency);

      Serial_Send(F_SIGNAL);

      char received_fun = Serial_Receive();
      sprintf(function, "%c", received_fun);
      TM_HD44780_Puts(15, 0, function);


  	switch (received_fun) {

  	case '1':
      	HAL_DAC_Stop_DMA(&hdac1, DAC1_CHANNEL_1);
  		HAL_DAC_Start_DMA(&hdac1, DAC1_CHANNEL_1, (uint32_t*)sine_wave, NS, DAC_ALIGN_12B_R);

  		break;

  	case '2':
      	HAL_DAC_Stop_DMA(&hdac1, DAC1_CHANNEL_1);
  		HAL_DAC_Start_DMA(&hdac1, DAC1_CHANNEL_1, (uint32_t*)triangle_wave, NS, DAC_ALIGN_12B_R);

          break;

  	case '3':
      	HAL_DAC_Stop_DMA(&hdac1, DAC1_CHANNEL_1);
  		HAL_DAC_Start_DMA(&hdac1, DAC1_CHANNEL_1, (uint32_t*)square_wave, NS, DAC_ALIGN_12B_R);

  		break;

  	case '4':
      	HAL_DAC_Stop_DMA(&hdac1, DAC1_CHANNEL_1);
  		HAL_DAC_Start_DMA(&hdac1, DAC1_CHANNEL_1, (uint32_t*)sawtooth_wave, NS, DAC_ALIGN_12B_R);

  		break;

  	default:

  		break;

  	}


      if (tempCharStore != KEYPAD_NO_PRESSED) {

      	TM_HD44780_Puts(15, 0, &tempCharStore);

      	// Stop the DAC DMA before starting a new waveform
      	HAL_DAC_Stop_DMA(&hdac1, DAC1_CHANNEL_1);


      	switch (tempCharStore) {

      	case '1':

      		HAL_DAC_Start_DMA(&hdac1, DAC1_CHANNEL_1, (uint32_t*)sine_wave, NS, DAC_ALIGN_12B_R);

      		break;

      	case '2':

      		HAL_DAC_Start_DMA(&hdac1, DAC1_CHANNEL_1, (uint32_t*)triangle_wave, NS, DAC_ALIGN_12B_R);

              break;

      	case '3':

      		HAL_DAC_Start_DMA(&hdac1, DAC1_CHANNEL_1, (uint32_t*)square_wave, NS, DAC_ALIGN_12B_R);

      		break;

      	case '4':

      		HAL_DAC_Start_DMA(&hdac1, DAC1_CHANNEL_1, (uint32_t*)sawtooth_wave, NS, DAC_ALIGN_12B_R);

      		break;

      	default:

      		break;

      	}
      }
      /* USER CODE BEGIN 3 */

	/* USER CODE END 3 */
	}

}


/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 50;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief DAC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_DAC1_Init(void)
{

  /* USER CODE BEGIN DAC1_Init 0 */

  /* USER CODE END DAC1_Init 0 */

  DAC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN DAC1_Init 1 */

  /* USER CODE END DAC1_Init 1 */

  /** DAC Initialization
  */
  hdac1.Instance = DAC1;
  if (HAL_DAC_Init(&hdac1) != HAL_OK)
  {
    Error_Handler();
  }

  /** DAC channel OUT1 config
  */
  sConfig.DAC_SampleAndHold = DAC_SAMPLEANDHOLD_DISABLE;
  sConfig.DAC_Trigger = DAC_TRIGGER_T2_TRGO;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  sConfig.DAC_ConnectOnChipPeripheral = DAC_CHIPCONNECT_DISABLE;
  sConfig.DAC_UserTrimming = DAC_TRIMMING_FACTORY;
  if (HAL_DAC_ConfigChannel(&hdac1, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC1_Init 2 */

  /* USER CODE END DAC1_Init 2 */

}

/**
  * @brief ETH Initialization Function
  * @param None
  * @retval None
  */
static void MX_ETH_Init(void)
{

  /* USER CODE BEGIN ETH_Init 0 */

  /* USER CODE END ETH_Init 0 */

   static uint8_t MACAddr[6];

  /* USER CODE BEGIN ETH_Init 1 */

  /* USER CODE END ETH_Init 1 */
  heth.Instance = ETH;
  MACAddr[0] = 0x00;
  MACAddr[1] = 0x80;
  MACAddr[2] = 0xE1;
  MACAddr[3] = 0x00;
  MACAddr[4] = 0x00;
  MACAddr[5] = 0x00;
  heth.Init.MACAddr = &MACAddr[0];
  heth.Init.MediaInterface = HAL_ETH_RMII_MODE;
  heth.Init.TxDesc = DMATxDscrTab;
  heth.Init.RxDesc = DMARxDscrTab;
  heth.Init.RxBuffLen = 1524;

  /* USER CODE BEGIN MACADDRESS */

  /* USER CODE END MACADDRESS */

  if (HAL_ETH_Init(&heth) != HAL_OK)
  {
    Error_Handler();
  }

  memset(&TxConfig, 0 , sizeof(ETH_TxPacketConfig));
  TxConfig.Attributes = ETH_TX_PACKETS_FEATURES_CSUM | ETH_TX_PACKETS_FEATURES_CRCPAD;
  TxConfig.ChecksumCtrl = ETH_CHECKSUM_IPHDR_PAYLOAD_INSERT_PHDR_CALC;
  TxConfig.CRCPadCtrl = ETH_CRC_PAD_INSERT;
  /* USER CODE BEGIN ETH_Init 2 */

  /* USER CODE END ETH_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_IC_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim1, &sConfigIC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 2000;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 1900;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim4, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 0;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 1-1;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart3, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart3, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief USB_OTG_FS Initialization Function
  * @param None
  * @retval None
  */
static void MX_USB_OTG_FS_PCD_Init(void)
{

  /* USER CODE BEGIN USB_OTG_FS_Init 0 */

  /* USER CODE END USB_OTG_FS_Init 0 */

  /* USER CODE BEGIN USB_OTG_FS_Init 1 */

  /* USER CODE END USB_OTG_FS_Init 1 */
  hpcd_USB_OTG_FS.Instance = USB_OTG_FS;
  hpcd_USB_OTG_FS.Init.dev_endpoints = 9;
  hpcd_USB_OTG_FS.Init.speed = PCD_SPEED_FULL;
  hpcd_USB_OTG_FS.Init.dma_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.phy_itface = PCD_PHY_EMBEDDED;
  hpcd_USB_OTG_FS.Init.Sof_enable = ENABLE;
  hpcd_USB_OTG_FS.Init.low_power_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.lpm_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.battery_charging_enable = ENABLE;
  hpcd_USB_OTG_FS.Init.vbus_sensing_enable = ENABLE;
  hpcd_USB_OTG_FS.Init.use_dedicated_ep1 = DISABLE;
  if (HAL_PCD_Init(&hpcd_USB_OTG_FS) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USB_OTG_FS_Init 2 */

  /* USER CODE END USB_OTG_FS_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOF, COL_1_Pin|COL_2_Pin|COL_3_Pin|LCD_D7_Pin
                          |LCD_D4_Pin|LCD_RS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, LCD_D6_Pin|LCD_D5_Pin|LCD_E_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LD3_Pin|LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(COL_4_GPIO_Port, COL_4_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(USB_PowerSwitchOn_GPIO_Port, USB_PowerSwitchOn_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : USER_Btn_Pin */
  GPIO_InitStruct.Pin = USER_Btn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USER_Btn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : ROW_4_Pin */
  GPIO_InitStruct.Pin = ROW_4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(ROW_4_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : COL_1_Pin COL_2_Pin COL_3_Pin LCD_D7_Pin
                           LCD_D4_Pin LCD_RS_Pin */
  GPIO_InitStruct.Pin = COL_1_Pin|COL_2_Pin|COL_3_Pin|LCD_D7_Pin
                          |LCD_D4_Pin|LCD_RS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pins : ROW_2_Pin ROW_3_Pin */
  GPIO_InitStruct.Pin = ROW_2_Pin|ROW_3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : ROW_1_Pin */
  GPIO_InitStruct.Pin = ROW_1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(ROW_1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LCD_D6_Pin LCD_D5_Pin LCD_E_Pin */
  GPIO_InitStruct.Pin = LCD_D6_Pin|LCD_D5_Pin|LCD_E_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : LD3_Pin LD2_Pin */
  GPIO_InitStruct.Pin = LD3_Pin|LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : COL_4_Pin */
  GPIO_InitStruct.Pin = COL_4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(COL_4_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_PowerSwitchOn_Pin */
  GPIO_InitStruct.Pin = USB_PowerSwitchOn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(USB_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_OverCurrent_Pin */
  GPIO_InitStruct.Pin = USB_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USB_OverCurrent_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

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
