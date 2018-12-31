/**
  ******************************************************************************
  * @file    template_source_file.c
  * @author  Onur Efe
  * @date    02.02.2016
  * @brief   Source file template.
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
#include "generic.h"

/* Includes ------------------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/

/**
  * @brief  Waits until system tick reaches to the given value
  *
  * @param  sysTick: System tick value.
  *
  * @retval None.
  */
void WaitUntil(uint32_t sysTick)
{
  while (sysTick > HAL_GetTick())
  {
  }
}

/**
  * @brief  Delay for period, in units of system time. Which is 1ms.
  *
  * @param  period: Period value.
  *
  * @retval None.
  */
void DelayMs(uint32_t period)
{
  uint32_t sys_tick;
  
  /* Get system tick */
  sys_tick = HAL_GetTick();
  
  /* Wait until period expired */
  while ((sys_tick + period) > HAL_GetTick())
  {
  }
}

/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private macros ------------------------------------------------------------*/
/* Private functions ---------------------------------------------------------*/