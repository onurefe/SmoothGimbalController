/**
  ******************************************************************************
  * File Name          : mxconstants.h
  * Description        : This file contains the common defines of the application
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

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/

#define PC14_OSC32_IN_Pin GPIO_PIN_14
#define PC14_OSC32_IN_GPIO_Port GPIOC
#define PC15_OSC32_OUT_Pin GPIO_PIN_15
#define PC15_OSC32_OUT_GPIO_Port GPIOC
#define PH0_OSC_IN_Pin GPIO_PIN_0
#define PH0_OSC_IN_GPIO_Port GPIOH
#define PH1_OSC_OUT_Pin GPIO_PIN_1
#define PH1_OSC_OUT_GPIO_Port GPIOH
#define OTG_FS_PowerSwitchOn_Pin GPIO_PIN_0
#define OTG_FS_PowerSwitchOn_GPIO_Port GPIOC
#define B1_Pin GPIO_PIN_0
#define B1_GPIO_Port GPIOA
#define First_Driver_IN_R_Pin GPIO_PIN_1
#define First_Driver_IN_R_GPIO_Port GPIOA
#define First_Driver_IN_S_Pin GPIO_PIN_2
#define First_Driver_IN_S_GPIO_Port GPIOA
#define Battery_Sense_Pin GPIO_PIN_3
#define Battery_Sense_GPIO_Port GPIOA
#define First_Driver_IN_T_Pin GPIO_PIN_5
#define First_Driver_IN_T_GPIO_Port GPIOA
#define Second_Driver_IN_R_Pin GPIO_PIN_6
#define Second_Driver_IN_R_GPIO_Port GPIOA
#define Second_Driver_IN_S_Pin GPIO_PIN_7
#define Second_Driver_IN_S_GPIO_Port GPIOA
#define Second_Driver_IN_T_Pin GPIO_PIN_0
#define Second_Driver_IN_T_GPIO_Port GPIOB
#define BOOT1_Pin GPIO_PIN_2
#define BOOT1_GPIO_Port GPIOB
#define First_Driver_nFAULT_Pin GPIO_PIN_7
#define First_Driver_nFAULT_GPIO_Port GPIOE
#define First_Driver_nRESET_Pin GPIO_PIN_8
#define First_Driver_nRESET_GPIO_Port GPIOE
#define First_Driver_nSLEEP_Pin GPIO_PIN_9
#define First_Driver_nSLEEP_GPIO_Port GPIOE
#define Second_Driver_nFAULT_Pin GPIO_PIN_10
#define Second_Driver_nFAULT_GPIO_Port GPIOE
#define Second_Driver_nRESET_Pin GPIO_PIN_11
#define Second_Driver_nRESET_GPIO_Port GPIOE
#define Second_Driver_nSLEEP_Pin GPIO_PIN_12
#define Second_Driver_nSLEEP_GPIO_Port GPIOE
#define Third_Driver_nFAULT_Pin GPIO_PIN_13
#define Third_Driver_nFAULT_GPIO_Port GPIOE
#define Third_Driver_nRESET_Pin GPIO_PIN_14
#define Third_Driver_nRESET_GPIO_Port GPIOE
#define Third_Driver_nSLEEP_Pin GPIO_PIN_15
#define Third_Driver_nSLEEP_GPIO_Port GPIOE
#define AS5048B_BUS_SCLK_Pin GPIO_PIN_10
#define AS5048B_BUS_SCLK_GPIO_Port GPIOB
#define AS5048B_BUS_SDA_Pin GPIO_PIN_11
#define AS5048B_BUS_SDA_GPIO_Port GPIOB
#define AS504BB_BUS_POWER_Pin GPIO_PIN_12
#define AS504BB_BUS_POWER_GPIO_Port GPIOB
#define HM_11_nRESET_Pin GPIO_PIN_15
#define HM_11_nRESET_GPIO_Port GPIOB
#define HM_11_RX_Pin GPIO_PIN_8
#define HM_11_RX_GPIO_Port GPIOD
#define HM_11_TX_Pin GPIO_PIN_9
#define HM_11_TX_GPIO_Port GPIOD
#define LD4_Pin GPIO_PIN_12
#define LD4_GPIO_Port GPIOD
#define LD3_Pin GPIO_PIN_13
#define LD3_GPIO_Port GPIOD
#define LD5_Pin GPIO_PIN_14
#define LD5_GPIO_Port GPIOD
#define LD6_Pin GPIO_PIN_15
#define LD6_GPIO_Port GPIOD
#define HM_11_2_RX_Pin GPIO_PIN_6
#define HM_11_2_RX_GPIO_Port GPIOC
#define HM_11_2_TX_Pin GPIO_PIN_7
#define HM_11_2_TX_GPIO_Port GPIOC
#define BNO055_nRESET_Pin GPIO_PIN_8
#define BNO055_nRESET_GPIO_Port GPIOC
#define BNO055_SDA_Pin GPIO_PIN_9
#define BNO055_SDA_GPIO_Port GPIOC
#define BNO055_SCL_Pin GPIO_PIN_8
#define BNO055_SCL_GPIO_Port GPIOA
#define VBUS_FS_Pin GPIO_PIN_9
#define VBUS_FS_GPIO_Port GPIOA
#define OTG_FS_ID_Pin GPIO_PIN_10
#define OTG_FS_ID_GPIO_Port GPIOA
#define OTG_FS_DM_Pin GPIO_PIN_11
#define OTG_FS_DM_GPIO_Port GPIOA
#define OTG_FS_DP_Pin GPIO_PIN_12
#define OTG_FS_DP_GPIO_Port GPIOA
#define SWDIO_Pin GPIO_PIN_13
#define SWDIO_GPIO_Port GPIOA
#define SWCLK_Pin GPIO_PIN_14
#define SWCLK_GPIO_Port GPIOA
#define OTG_FS_OverCurrent_Pin GPIO_PIN_5
#define OTG_FS_OverCurrent_GPIO_Port GPIOD
#define SWO_Pin GPIO_PIN_3
#define SWO_GPIO_Port GPIOB
#define Third_Driver_IN_R_Pin GPIO_PIN_6
#define Third_Driver_IN_R_GPIO_Port GPIOB
#define Third_Driver_IN_S_Pin GPIO_PIN_7
#define Third_Driver_IN_S_GPIO_Port GPIOB
#define Third_Driver_IN_T_Pin GPIO_PIN_8
#define Third_Driver_IN_T_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
