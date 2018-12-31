/**
  ******************************************************************************
  * @file    generic.h
  * @author  Onur Efe
  * @date    06.03.2016
  * @brief   Header file for generic data types and constants.
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
#ifndef __GENERIC_H
#define __GENERIC_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h"

/* Exported constants --------------------------------------------------------*/
#define PI                      3.14159265359f                  /* PI */
#define TRUE                    0x01                            /* TRUE, Boolean value */
#define FALSE                   0x00                            /* FALSE, Boolean value */
#define CONTROL_FREQ            2500U                           /* Control frequency in units of Hertz */
   
/* Exported functions --------------------------------------------------------*/

/**
  * @brief  Waits until system tick reaches to the given value
  *
  * @param  sysTick: System tick value.
  *
  * @retval None.
  */
void WaitUntil(uint32_t sysTick);

/**
  * @brief  Delay for period, in units of system time. Which is 1ms.
  *
  * @param  period: Period value.
  *
  * @retval None.
  */
void DelayMs(uint32_t period);

#ifdef __cplusplus
}
#endif

#endif