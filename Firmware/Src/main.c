/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  *
  * COPYRIGHT(c) 2016 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* USER CODE BEGIN Includes */
    
#include <math.h>
#include "i2c_manager.h"
#include "mot_ctrl.h"
#include "orient_sens_drv.h"
#include "quaternion.h"
#include "vector_int.h"
#include "uart_pkt_xcvr.h"
#include "generic.h"

/* Constants -----------------------------------------------------------------*/
#define BODY_ORIENTATION_OFFSET                         {1.0f, 0.0f, 0.0f, 0.0f}

/* Link zero positions */
#define FIRST_ZERO_POSITION                             0.72475f
#define SECOND_ZERO_POSITION                            -1.25f
#define THIRD_ZERO_POSITION                             1.091f

#define CONTROLLER_DROPOUT_PERIOD                       25U     /* In units of 10ms */

#define OPERATION_MODE_STABILIZER                       0x00
#define OPERATION_MODE_STABILIZER_AND_CONTROLLER        0x01

#define DATA_PACKET_SIZE                                12U

#define NUM_OF_SUCCESSIVE_PACKETS_FOR_SYNCHRONIZATION   100U
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c2;
I2C_HandleTypeDef hi2c3;
DMA_HandleTypeDef hdma_i2c2_rx;
DMA_HandleTypeDef hdma_i2c2_tx;
DMA_HandleTypeDef hdma_i2c3_rx;
DMA_HandleTypeDef hdma_i2c3_tx;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim7;

UART_HandleTypeDef huart3;
UART_HandleTypeDef huart6;
DMA_HandleTypeDef hdma_usart3_rx;
DMA_HandleTypeDef hdma_usart6_rx;
DMA_HandleTypeDef hdma_usart6_tx;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
volatile uint8_t        OperationMode;

uint16_t                LastControllerInteractionStopwatch = 0U;
volatile uint32_t       ControlTick = 0U;

Quaternion_Quaternion_t BodyOrientation = {1.0f, 0.0f, 0.0f, 0.0f};
Quaternion_Quaternion_t ControllerOrientation = {1.0f, 0.0f, 0.0f, 0.0f};
Quaternion_EulerAngles_t EulerAngles = {0.0f, 0.0f, 0.0f};

uint32_t                SuccessivePackets = 0U;
int32_t                 TickDifferenceCoarse = 0U;
float                   TickDifferenceFine = 0.0f;

volatile uint8_t        ErrorOccured = FALSE;
volatile uint8_t        NewPacketFlag = FALSE;
uint8_t                 DataPacket[DATA_PACKET_SIZE];

I2CManager_Handle_t     i2c2ManagerHandle;
I2CManager_Handle_t     i2c3ManagerHandle;

OrientSensDrv_Handle_t  orientSensDrvHandle;

VectorInt_Handle_t      bodyVectorIntHandle;
VectorInt_Handle_t      controllerVectorIntHandle;

MotCtrl_Handle_t        firstMotCtrlHandle;
MotCtrl_Handle_t        secondMotCtrlHandle;
MotCtrl_Handle_t        thirdMotCtrlHandle;

UartPktXcvr_Handle_t    uartPktXcvrHandle;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C2_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C3_Init(void);
static void MX_TIM6_Init(void);
static void MX_TIM7_Init(void);
static void MX_USART6_UART_Init(void);

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);
                

/* USER CODE BEGIN PFP */

/* Private function prototypes -----------------------------------------------*/
static void MotionControlTask(void);
static void OrientationReadTask(void);
static void ProcessPacketDataTask(void);
static void Error_Handler(void);
void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_I2C2_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_USART3_UART_Init();
  MX_ADC1_Init();
  MX_I2C3_Init();
  MX_TIM6_Init();
  MX_TIM7_Init();
  MX_USART6_UART_Init();

  /* USER CODE BEGIN 2 */
  /* Initialize Eeprom Emulator */
  if (EepromEmulator_Init() != EEPROM_EMULATOR_OP_RES_SUCCESS)
  {
    Error_Handler();
  }
  
  /* Initialize I2C managers */
  {
    I2CManager_Init_t i2cManagerInit;
    
    i2cManagerInit.hi2c = &hi2c2;
    
    if (I2CManager_Init(&i2c2ManagerHandle, &i2cManagerInit) != I2C_MANAGER_OP_RES_SUCCESS)
    {
      Error_Handler();
    }
    
    i2cManagerInit.hi2c = &hi2c3;
    
    if (I2CManager_Init(&i2c3ManagerHandle, &i2cManagerInit) != I2C_MANAGER_OP_RES_SUCCESS)
    {
      Error_Handler();
    }
  }
  
  /* Start I2C managers */
  {
    if (I2CManager_Start(&i2c2ManagerHandle) != I2C_MANAGER_OP_RES_SUCCESS)
    {
      Error_Handler();
    }
    
    if (I2CManager_Start(&i2c3ManagerHandle) != I2C_MANAGER_OP_RES_SUCCESS)
    {
      Error_Handler();
    }
  }
  
  /* Initialize orientation sensor driver */
  {
    OrientSensDrv_Init_t orientSensDrvInit;
    
    /* Init orientation sensor driver */
    orientSensDrvInit.devAddr = 0x50;
    orientSensDrvInit.eeEmulObjId = 0;
    orientSensDrvInit.pI2CManagerHandle = &i2c3ManagerHandle;
    
    if (OrientSensDrv_Init(&orientSensDrvHandle, &orientSensDrvInit) != ORIENT_SENS_DRV_OP_RES_SUCCESS)
    {
      Error_Handler();
    }
  }
  
  /* Initialize interpolaters */
  {
    VectorInt_Init_t vectorIntInitStruct;
    
    vectorIntInitStruct.filterTC = 0.01f;
    vectorIntInitStruct.timeShift = 0.012f;
    vectorIntInitStruct.circBufferSize = 5;
    vectorIntInitStruct.vectorSize = 4U;
    
    vectorIntInitStruct.unitVector[0] = 1.0f;
    vectorIntInitStruct.unitVector[1] = 0.0f;
    vectorIntInitStruct.unitVector[2] = 0.0f;
    vectorIntInitStruct.unitVector[3] = 0.0f;
    
    if (VectorInt_Init(&bodyVectorIntHandle, &vectorIntInitStruct) != VECTOR_INT_OP_RES_SUCCESS)
    {
      Error_Handler();
    }
    
    vectorIntInitStruct.filterTC = 0.1f;
    vectorIntInitStruct.timeShift = 0.05f;
    vectorIntInitStruct.circBufferSize = 12;
    vectorIntInitStruct.vectorSize = 4U;
    if (VectorInt_Init(&controllerVectorIntHandle, &vectorIntInitStruct) != VECTOR_INT_OP_RES_SUCCESS)
    {
      Error_Handler();
    }
  }
  
  /* Initialize motor controllers */
  
  {
    MotCtrl_Init_t motCtrlInit;
    
    /* Init first motor controller */
    motCtrlInit.bldcDrvInit.hTim = &htim2;
    motCtrlInit.bldcDrvInit.poleCount = 14U;
    motCtrlInit.bldcDrvInit.pinMap.inRChannel = TIM_CHANNEL_1;
    motCtrlInit.bldcDrvInit.pinMap.inSChannel = TIM_CHANNEL_2;
    motCtrlInit.bldcDrvInit.pinMap.inTChannel = TIM_CHANNEL_3;
    motCtrlInit.bldcDrvInit.pinMap.nFaultPin = First_Driver_nFAULT_Pin;
    motCtrlInit.bldcDrvInit.pinMap.nFaultPort = First_Driver_nFAULT_GPIO_Port;
    motCtrlInit.bldcDrvInit.pinMap.nResetPin = First_Driver_nRESET_Pin;
    motCtrlInit.bldcDrvInit.pinMap.nResetPort = First_Driver_nRESET_GPIO_Port;
    motCtrlInit.bldcDrvInit.pinMap.nSleepPin = First_Driver_nSLEEP_Pin;
    motCtrlInit.bldcDrvInit.pinMap.nSleepPort = First_Driver_nSLEEP_GPIO_Port;
    
    motCtrlInit.pidCtrlInit.gain = 10.0f;
    motCtrlInit.pidCtrlInit.integralTC = 100000000.0f;
    motCtrlInit.pidCtrlInit.derivativeTC = 0.05f;
    motCtrlInit.pidCtrlInit.filterTC = 0.01f;
    motCtrlInit.pidCtrlInit.highLimit = 1.0f;
    motCtrlInit.pidCtrlInit.lowLimit = -1.0f;
    
    motCtrlInit.posSensDrvInit.devAddr = 0x80;
    motCtrlInit.posSensDrvInit.pI2CManagerHandle = &i2c2ManagerHandle;
    motCtrlInit.posSensDrvInit.filterTC = 0.0015;
    motCtrlInit.posSensDrvInit.zeroPosition = FIRST_ZERO_POSITION;
    
    motCtrlInit.eeEmulObjId = 1;

    if (MotCtrl_Init(&firstMotCtrlHandle, &motCtrlInit) != MOT_CTRL_OP_RES_SUCCESS)
    {
      Error_Handler();
    }
    
    /* Init second motor controller */
    motCtrlInit.bldcDrvInit.hTim = &htim3;
    motCtrlInit.bldcDrvInit.poleCount = 14U;
    motCtrlInit.bldcDrvInit.pinMap.inRChannel = TIM_CHANNEL_1;
    motCtrlInit.bldcDrvInit.pinMap.inSChannel = TIM_CHANNEL_2;
    motCtrlInit.bldcDrvInit.pinMap.inTChannel = TIM_CHANNEL_3;
    motCtrlInit.bldcDrvInit.pinMap.nFaultPin = Second_Driver_nFAULT_Pin;
    motCtrlInit.bldcDrvInit.pinMap.nFaultPort = Second_Driver_nFAULT_GPIO_Port;
    motCtrlInit.bldcDrvInit.pinMap.nResetPin = Second_Driver_nRESET_Pin;
    motCtrlInit.bldcDrvInit.pinMap.nResetPort = Second_Driver_nRESET_GPIO_Port;
    motCtrlInit.bldcDrvInit.pinMap.nSleepPin = Second_Driver_nSLEEP_Pin;
    motCtrlInit.bldcDrvInit.pinMap.nSleepPort = Second_Driver_nSLEEP_GPIO_Port;
    
    motCtrlInit.pidCtrlInit.gain = 7.0f;
    motCtrlInit.pidCtrlInit.integralTC = 1000000.0f;
    motCtrlInit.pidCtrlInit.derivativeTC = 0.05f;
    motCtrlInit.pidCtrlInit.filterTC = 0.01f;
    motCtrlInit.pidCtrlInit.highLimit = 1.0f;
    motCtrlInit.pidCtrlInit.lowLimit = -1.0f;
    
    motCtrlInit.posSensDrvInit.devAddr = 0x82;
    motCtrlInit.posSensDrvInit.pI2CManagerHandle = &i2c2ManagerHandle;
    motCtrlInit.posSensDrvInit.filterTC = 0.0015;
    motCtrlInit.posSensDrvInit.zeroPosition = SECOND_ZERO_POSITION;

    motCtrlInit.eeEmulObjId = 2;
    
    if (MotCtrl_Init(&secondMotCtrlHandle, &motCtrlInit) != MOT_CTRL_OP_RES_SUCCESS)
    {
      Error_Handler();
    }
    
    /* Init third motor controller */
    motCtrlInit.bldcDrvInit.hTim = &htim4;
    motCtrlInit.bldcDrvInit.poleCount = 14U;
    motCtrlInit.bldcDrvInit.pinMap.inRChannel = TIM_CHANNEL_1;
    motCtrlInit.bldcDrvInit.pinMap.inSChannel = TIM_CHANNEL_2;
    motCtrlInit.bldcDrvInit.pinMap.inTChannel = TIM_CHANNEL_3;
    motCtrlInit.bldcDrvInit.pinMap.nFaultPin = Third_Driver_nFAULT_Pin;
    motCtrlInit.bldcDrvInit.pinMap.nFaultPort = Third_Driver_nFAULT_GPIO_Port;
    motCtrlInit.bldcDrvInit.pinMap.nResetPin = Third_Driver_nRESET_Pin;
    motCtrlInit.bldcDrvInit.pinMap.nResetPort = Third_Driver_nRESET_GPIO_Port;
    motCtrlInit.bldcDrvInit.pinMap.nSleepPin = Third_Driver_nSLEEP_Pin;
    motCtrlInit.bldcDrvInit.pinMap.nSleepPort = Third_Driver_nSLEEP_GPIO_Port;
    
    motCtrlInit.pidCtrlInit.gain = 5.5f;
    motCtrlInit.pidCtrlInit.integralTC = 10000000.0f;
    motCtrlInit.pidCtrlInit.derivativeTC = 0.04f;
    motCtrlInit.pidCtrlInit.filterTC = 0.01f;
    motCtrlInit.pidCtrlInit.highLimit = 1.0f;
    motCtrlInit.pidCtrlInit.lowLimit = -1.0f;
    
    motCtrlInit.posSensDrvInit.devAddr = 0x84;
    motCtrlInit.posSensDrvInit.pI2CManagerHandle = &i2c2ManagerHandle;
    motCtrlInit.posSensDrvInit.filterTC = 0.0015;
    motCtrlInit.posSensDrvInit.zeroPosition = THIRD_ZERO_POSITION;

    motCtrlInit.eeEmulObjId = 3;
    
    if (MotCtrl_Init(&thirdMotCtrlHandle, &motCtrlInit) != MOT_CTRL_OP_RES_SUCCESS)
    {
      Error_Handler();
    }
  }
    
  /* Initialize UART packet transceiver */
  {
    UartPktXcvr_Init_t  uartPktXcvrInitStruct;
    uartPktXcvrInitStruct.huart = &huart6;
    
    if (UartPktXcvr_Init(&uartPktXcvrHandle, &uartPktXcvrInitStruct) != UART_PKT_XCVR_OP_RES_SUCCESS)
    {
      Error_Handler();
    }
  }
  
  /* Start orientation sensor driver */
  {
    if (OrientSensDrv_Start(&orientSensDrvHandle) != ORIENT_SENS_DRV_OP_RES_SUCCESS)
    {
      Error_Handler();
    }
  }
  
  /* Start body and controller vector interpolater */
  {
    if (VectorInt_Start(&bodyVectorIntHandle) != VECTOR_INT_OP_RES_SUCCESS)
    {
      Error_Handler();
    }
    
    if (VectorInt_Start(&controllerVectorIntHandle) != VECTOR_INT_OP_RES_SUCCESS)
    {
      Error_Handler();
    }
  }
  
  /* Start motor controllers */
  {
    if (MotCtrl_Start(&firstMotCtrlHandle) != MOT_CTRL_OP_RES_SUCCESS)
    {
      Error_Handler();
    }

    if (MotCtrl_Start(&secondMotCtrlHandle) != MOT_CTRL_OP_RES_SUCCESS)
    {
      Error_Handler();
    }

    if (MotCtrl_Start(&thirdMotCtrlHandle) != MOT_CTRL_OP_RES_SUCCESS)
    {
      Error_Handler();
    }
  }

  /* Start UART Packet Transceiver */
  {
    if (UartPktXcvr_Start(&uartPktXcvrHandle) != UART_PKT_XCVR_OP_RES_SUCCESS)
    {
      Error_Handler();
    }
  }
  
  /* Start timers */
  if (HAL_TIM_Base_Start_IT(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  
  if (HAL_TIM_Base_Start_IT(&htim7) != HAL_OK)
  {
    Error_Handler();
  }
  
  OperationMode = OPERATION_MODE_STABILIZER;
  
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5);

  HAL_RCC_EnableCSS();

  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* ADC1 init function */
void MX_ADC1_Init(void)
{

  ADC_AnalogWDGConfTypeDef AnalogWDGConfig;
  ADC_ChannelConfTypeDef sConfig;

    /**Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
    */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  HAL_ADC_Init(&hadc1);

    /**Configure the analog watchdog 
    */
  AnalogWDGConfig.WatchdogMode = ADC_ANALOGWATCHDOG_SINGLE_REG;
  AnalogWDGConfig.HighThreshold = 2300;
  AnalogWDGConfig.LowThreshold = 2100;
  AnalogWDGConfig.Channel = ADC_CHANNEL_3;
  AnalogWDGConfig.ITMode = ENABLE;
  HAL_ADC_AnalogWDGConfig(&hadc1, &AnalogWDGConfig);

    /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
    */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  HAL_ADC_ConfigChannel(&hadc1, &sConfig);

}

/* I2C2 init function */
void MX_I2C2_Init(void)
{

  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 400000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  HAL_I2C_Init(&hi2c2);

}

/* I2C3 init function */
void MX_I2C3_Init(void)
{

  hi2c3.Instance = I2C3;
  hi2c3.Init.ClockSpeed = 100000;
  hi2c3.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c3.Init.OwnAddress1 = 0;
  hi2c3.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c3.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c3.Init.OwnAddress2 = 0;
  hi2c3.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c3.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  HAL_I2C_Init(&hi2c3);

}

/* TIM2 init function */
void MX_TIM2_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4200;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  HAL_TIM_Base_Init(&htim2);

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig);

  HAL_TIM_PWM_Init(&htim2);

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig);

  sConfigOC.OCMode = TIM_OCMODE_PWM2;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_LOW;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1);

  HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2);

  HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3);

  HAL_TIM_MspPostInit(&htim2);

}

/* TIM3 init function */
void MX_TIM3_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 4200;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  HAL_TIM_Base_Init(&htim3);

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig);

  HAL_TIM_PWM_Init(&htim3);

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig);

  sConfigOC.OCMode = TIM_OCMODE_PWM2;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_LOW;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1);

  HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2);

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3);

  HAL_TIM_MspPostInit(&htim3);

}

/* TIM4 init function */
void MX_TIM4_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;

  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 4200;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  HAL_TIM_Base_Init(&htim4);

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig);

  HAL_TIM_PWM_Init(&htim4);

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig);

  sConfigOC.OCMode = TIM_OCMODE_PWM2;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_LOW;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1);

  HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_2);

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_3);

  HAL_TIM_MspPostInit(&htim4);

}

/* TIM6 init function */
void MX_TIM6_Init(void)
{

  TIM_MasterConfigTypeDef sMasterConfig;

  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 83;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 1000;
  HAL_TIM_Base_Init(&htim6);

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig);

}

/* TIM7 init function */
void MX_TIM7_Init(void)
{

  TIM_MasterConfigTypeDef sMasterConfig;

  htim7.Instance = TIM7;
  htim7.Init.Prescaler = 83;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 10000;
  HAL_TIM_Base_Init(&htim7);

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig);

}

/* USART3 init function */
void MX_USART3_UART_Init(void)
{

  huart3.Instance = USART3;
  huart3.Init.BaudRate = 9600;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  HAL_UART_Init(&huart3);

}

/* USART6 init function */
void MX_USART6_UART_Init(void)
{

  huart6.Instance = USART6;
  huart6.Init.BaudRate = 9600;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  HAL_UART_Init(&huart6);

}

/** 
  * Enable DMA controller clock
  */
void MX_DMA_Init(void) 
{
  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);
  /* DMA1_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream2_IRQn, 8, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream2_IRQn);
  /* DMA1_Stream3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream3_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream3_IRQn);
  /* DMA1_Stream4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream4_IRQn, 9, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream4_IRQn);
  /* DMA1_Stream7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream7_IRQn, 3, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream7_IRQn);
  /* DMA2_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream1_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream1_IRQn);
  /* DMA2_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream6_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream6_IRQn);

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
        * Free pins are configured automatically as Analog (this feature is enabled through 
        * the Code Generation settings)
     PA9   ------> USB_OTG_FS_VBUS
     PA10   ------> USB_OTG_FS_ID
     PA11   ------> USB_OTG_FS_DM
     PA12   ------> USB_OTG_FS_DP
*/
void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pins : PE2 PE3 PE4 PE5 
                           PE6 PE0 PE1 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5 
                          |GPIO_PIN_6|GPIO_PIN_0|GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : PC13 PC1 PC2 PC3 
                           PC4 PC5 PC10 PC11 
                           PC12 */
  GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3 
                          |GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_10|GPIO_PIN_11 
                          |GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : OTG_FS_PowerSwitchOn_Pin BNO055_nRESET_Pin */
  GPIO_InitStruct.Pin = OTG_FS_PowerSwitchOn_Pin|BNO055_nRESET_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PA4 PA15 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB1 PB13 PB14 PB4 
                           PB5 PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_4 
                          |GPIO_PIN_5|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : BOOT1_Pin */
  GPIO_InitStruct.Pin = BOOT1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BOOT1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : First_Driver_nFAULT_Pin Second_Driver_nFAULT_Pin Third_Driver_nFAULT_Pin */
  GPIO_InitStruct.Pin = First_Driver_nFAULT_Pin|Second_Driver_nFAULT_Pin|Third_Driver_nFAULT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : First_Driver_nRESET_Pin First_Driver_nSLEEP_Pin Second_Driver_nRESET_Pin Second_Driver_nSLEEP_Pin 
                           Third_Driver_nRESET_Pin Third_Driver_nSLEEP_Pin */
  GPIO_InitStruct.Pin = First_Driver_nRESET_Pin|First_Driver_nSLEEP_Pin|Second_Driver_nRESET_Pin|Second_Driver_nSLEEP_Pin 
                          |Third_Driver_nRESET_Pin|Third_Driver_nSLEEP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : AS504BB_BUS_POWER_Pin HM_11_nRESET_Pin */
  GPIO_InitStruct.Pin = AS504BB_BUS_POWER_Pin|HM_11_nRESET_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PD10 PD11 PD0 PD1 
                           PD2 PD3 PD4 PD6 
                           PD7 */
  GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_0|GPIO_PIN_1 
                          |GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_6 
                          |GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : LD4_Pin LD3_Pin LD5_Pin LD6_Pin */
  GPIO_InitStruct.Pin = LD4_Pin|LD3_Pin|LD5_Pin|LD6_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : VBUS_FS_Pin */
  GPIO_InitStruct.Pin = VBUS_FS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(VBUS_FS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : OTG_FS_ID_Pin OTG_FS_DM_Pin OTG_FS_DP_Pin */
  GPIO_InitStruct.Pin = OTG_FS_ID_Pin|OTG_FS_DM_Pin|OTG_FS_DP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF10_OTG_FS;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : OTG_FS_OverCurrent_Pin */
  GPIO_InitStruct.Pin = OTG_FS_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(OTG_FS_OverCurrent_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(OTG_FS_PowerSwitchOn_GPIO_Port, OTG_FS_PowerSwitchOn_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, First_Driver_nRESET_Pin|First_Driver_nSLEEP_Pin|Second_Driver_nRESET_Pin|Second_Driver_nSLEEP_Pin 
                          |Third_Driver_nRESET_Pin|Third_Driver_nSLEEP_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, AS504BB_BUS_POWER_Pin|HM_11_nRESET_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, LD4_Pin|LD3_Pin|LD5_Pin|LD6_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(BNO055_nRESET_GPIO_Port, BNO055_nRESET_Pin, GPIO_PIN_SET);

}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* If the timer is TIM6 */
  if ((htim->Instance == TIM6) && (ErrorOccured == FALSE))
  {
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_SET);
    
    ControlTick++;

    if (NewPacketFlag == TRUE)
    {
      ProcessPacketDataTask();
      
      NewPacketFlag = FALSE;
    }
    
    MotionControlTask();
    
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_RESET);
  }
  
  /* If TIM7 interrupt occured */
  if ((htim->Instance == TIM7) && (ErrorOccured == FALSE))
  {
    OrientationReadTask();

    if (OperationMode == OPERATION_MODE_STABILIZER_AND_CONTROLLER)
    {
      if (++LastControllerInteractionStopwatch >= CONTROLLER_DROPOUT_PERIOD)
      {
        SuccessivePackets = 0;
        OperationMode = OPERATION_MODE_STABILIZER;
      }
    }
  }
}

static void MotionControlTask(void)
{
  Quaternion_Quaternion_t body_orientation_inverted;
  Quaternion_Quaternion_t orientation_difference;
  
  /* Take action according to operation mode */
  if (OperationMode == OPERATION_MODE_STABILIZER_AND_CONTROLLER)
  {
    if (VectorInt_Execute(&controllerVectorIntHandle, ((float *)&ControllerOrientation), 
                          ControlTick) != VECTOR_INT_OP_RES_SUCCESS)
    {
      Error_Handler();
    }
  }

  if (VectorInt_Execute(&bodyVectorIntHandle, ((float *)&BodyOrientation), 
                        ControlTick) != VECTOR_INT_OP_RES_SUCCESS)
  {
    Error_Handler();
  }
  
  /* Calculate inverted body orientation */
  if (Quaternion_CalculateInverse(&BodyOrientation, 
                                  &body_orientation_inverted) != QUATERNION_OP_RES_SUCCESS)
  {
    Error_Handler();
  }

  if (Quaternion_Multiply(&body_orientation_inverted, &ControllerOrientation, 
                          &orientation_difference) != QUATERNION_OP_RES_SUCCESS)
  {
    Error_Handler();
  }
  
  /* Calculate euler angles */
  if (Quaternion_CalculateEulerAngles(&orientation_difference,
                                      &EulerAngles,
                                      &EulerAngles) != QUATERNION_OP_RES_SUCCESS)
  {
    Error_Handler();
  }
  
  /* Execute motion controllers */
  if ((MotCtrl_Execute(&firstMotCtrlHandle, EulerAngles.first) != MOT_CTRL_OP_RES_SUCCESS))
  {
    Error_Handler();
  }
  
  if ((MotCtrl_Execute(&secondMotCtrlHandle, EulerAngles.second) != MOT_CTRL_OP_RES_SUCCESS))
  {
    Error_Handler();
  }
  
  if ((MotCtrl_Execute(&thirdMotCtrlHandle, EulerAngles.third) != MOT_CTRL_OP_RES_SUCCESS))
  {
    Error_Handler();
  }
}

static void OrientationReadTask(void)
{
  Quaternion_Quaternion_t body_orientation;
  Quaternion_Quaternion_t body_orientation_offset = BODY_ORIENTATION_OFFSET;

  /* Get orientation */
  if (OrientSensDrv_Execute(&orientSensDrvHandle, &body_orientation) != ORIENT_SENS_DRV_OP_RES_SUCCESS)
  {
    Error_Handler();
  }
  
  /* Normalize quaternion */
  if (Quaternion_NormalizeQuaternion(&body_orientation, &body_orientation) != QUATERNION_OP_RES_SUCCESS)
  {
    Error_Handler();
  }
  
  /* Add orientation offset */
  if (Quaternion_Rotate(&body_orientation, &body_orientation_offset,
                        &body_orientation) != QUATERNION_OP_RES_SUCCESS)
  {
    Error_Handler();
  }

  /* Input value to body vector interpolater */
  if (VectorInt_Input(&bodyVectorIntHandle, (float *)&body_orientation, 
                      ControlTick) != VECTOR_INT_OP_RES_SUCCESS)
  {
    Error_Handler();
  }
}

static void ProcessPacketDataTask(void)
{
  uint32_t packet_tick_raw;
  uint32_t packet_tick;
  uint32_t synchronized_packet_tick;
  int32_t tick_difference;
  
  /* Parse packet time */
  packet_tick_raw = ((uint32_t)DataPacket[8]);
  packet_tick_raw |= ((uint32_t)DataPacket[9] << 8);
  packet_tick_raw |= ((uint32_t)DataPacket[10] << 16);
  packet_tick_raw |= ((uint32_t)DataPacket[11] << 24);
  
  packet_tick = (uint32_t)(packet_tick_raw * ((float)CONTROL_FREQ / 1000.0f));
                 
  /* Calculate tick difference */
  tick_difference = (int32_t)(ControlTick - packet_tick);
  
  /* If the operation mode was stabilizer, switch to stabilizer and controller mode. */
  if (OperationMode == OPERATION_MODE_STABILIZER)
  {
    TickDifferenceCoarse = tick_difference;
    TickDifferenceFine = 0.0f;
    OperationMode = OPERATION_MODE_STABILIZER_AND_CONTROLLER;
  }
  else
  {
    TickDifferenceFine = TickDifferenceFine * 0.9f + (((float)tick_difference - TickDifferenceCoarse) * 0.1f);
    SuccessivePackets++;
  }

  synchronized_packet_tick = (uint32_t)((int32_t)packet_tick + TickDifferenceCoarse + \
                                        ((int32_t)TickDifferenceFine));
  
  if (SuccessivePackets > NUM_OF_SUCCESSIVE_PACKETS_FOR_SYNCHRONIZATION)
  {
    Quaternion_Quaternion_t controller_orientation;
  
    /* Cast signed integers to floating pointer variables
      Turn the MSB and LSB into a signed 16-bit value */
    controller_orientation.w = (float)((int16_t)(DataPacket[1] << 8) | \
                                                 DataPacket[0]);    
    controller_orientation.x = (float)((int16_t)(DataPacket[3] << 8) | \
                                                 DataPacket[2]);
    controller_orientation.y = (float)((int16_t)(DataPacket[5] << 8) | \
                                                 DataPacket[4]);
    controller_orientation.z = (float)((int16_t)(DataPacket[7] << 8) | \
                                                 DataPacket[6]);
    /* Normalize quaternion */
    if (Quaternion_NormalizeQuaternion(&controller_orientation, 
                                       &controller_orientation) != QUATERNION_OP_RES_SUCCESS)
    {
      Error_Handler();
    }
    
    /* Input to controller vector interpolater */
    if (VectorInt_Input(&controllerVectorIntHandle,
                        (float *)&controller_orientation,
                        synchronized_packet_tick) != VECTOR_INT_OP_RES_SUCCESS)
    {
      Error_Handler();
    }
  }
  
  LastControllerInteractionStopwatch = 0;
}

void I2CManager_ErrorCallback(I2CManager_Handle_t *pHandle)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(pHandle);
  
  Error_Handler();
}

void UartPktXcvr_ErrorOccuredCallback(UartPktXcvr_Handle_t *pHandle)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(pHandle);
  
  Error_Handler();
}

void UartPktXcvr_PacketReceivedCallback(UartPktXcvr_Handle_t *pHandle, 
                                        uint8_t *pPacket)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(pHandle);
  
  /* If there is unprocessed data, discard the new packet */
  if (NewPacketFlag == FALSE)
  {
    /* Copy data packet */
    for (uint8_t i = 0; i < DATA_PACKET_SIZE; i++)
    {
      DataPacket[i] = pPacket[i];
    }
    
    NewPacketFlag = TRUE;
  }
}

static void Error_Handler(void)
{
  ErrorOccured = TRUE;
  
  HAL_GPIO_WritePin(GPIOD, LD3_Pin, GPIO_PIN_SET);
  
  while(1);
}
/* USER CODE END 4 */

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
