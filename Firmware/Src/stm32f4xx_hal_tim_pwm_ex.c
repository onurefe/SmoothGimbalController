/**
  ******************************************************************************
  * @file    stm32f4xx_hal_tim_pwm_ex.c
  * @author  Onur Efe
  * @date    03.02.2016
  * @brief   STM32F4xx series hal driver extension for updating pwm duty cycle on fly.
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
/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal_tim_pwm_ex.h"

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
uint8_t HAL_TIM_PWMEx_SetPulse(TIM_HandleTypeDef *htim, uint16_t Pulse, uint32_t Channel)
{
  /* Lock the peripheral */
  __HAL_LOCK(htim);
  
  /* Set peripheral state busy */
  htim->State = HAL_TIM_STATE_BUSY;
  
  /* Ensure that the channel parameter is valid */
  assert_param(IS_TIM_CHANNELS(Channel)); 
  
  switch (Channel)
  {
  case TIM_CHANNEL_1:
    {
      /* Set Channel 1 pulse value */
      htim->Instance->CCR1 = Pulse;
    }
    break;
    
  case TIM_CHANNEL_2:
    {
      /* Set Channel 2 pulse value */
      htim->Instance->CCR2 = Pulse;
    }
    break;
    
  case TIM_CHANNEL_3:
    {
      /* Set Channel 3 pulse value */
      htim->Instance->CCR3 = Pulse;
    }
    break;
    
  case TIM_CHANNEL_4:
    {
      /* Set Channel 4 pulse value */
      htim->Instance->CCR4 = Pulse;
    }
    break;
  }
  
  /* Peripheral is ready */
  htim->State = HAL_TIM_STATE_READY;
  
  /* Unlock the peripheral */
  __HAL_UNLOCK(htim);
  
  return HAL_OK;
}