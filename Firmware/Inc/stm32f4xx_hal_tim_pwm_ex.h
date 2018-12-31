/**
  ******************************************************************************
  * @file    stm32f4xx_hal_tim_pwm_ex.h
  * @author  Onur Efe
  * @date    03.02.2016
  * @brief   STM32F4xx series hal driver extension header file.
  ******************************************************************************
  *
  * Copyright (c) 2016 MEDIALAB TECHNOLOGY
  *
  * This program is free software; you can redistribute it and/or modify it 
  * under the terms of the GNU General Public License as published by the Free 
  * Software Foundation; either version 2 of the License, or (at your option) 
  * any later version. This program is distributed in the hope that it will be 
  * useful, but WITHOUT ANY WARRANTY; without even the implied warranty of
  * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General 
  * Public License for more details.
  *
  * You should have received a copy of the GNU General Public License
  * along with this program; if not, write to the Free Software
  * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __STM32F4XX_HAL_TIM_PWM_EX_H
#define __STM32F4XX_HAL_TIM_PWM_EX_H

#ifdef __cplusplus
 extern "C" {
#endif
   
/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Exported functions --------------------------------------------------------*/
   
/**
  * @brief  This function updates channel pulse value of the given timer on the fly.
  *
  * @param  htim: Timer handle.
  * @param  Pulse: Pulse value of the channel which is related with the duty cycle.
  * @param  Channel: Channel label.
  *
  * @retval HAL_BUSY or HAL_OK.
  */
uint8_t HAL_TIM_PWMEx_SetPulse(TIM_HandleTypeDef *htim, uint16_t Pulse, uint32_t Channel);

#ifdef __cplusplus
}
#endif

#endif