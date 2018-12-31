/**
  ******************************************************************************
  * @file    bldc_driver.c
  * @author  Onur Efe
  * @date    18.05.2016
  * @brief   Class drives BLDC motors by interacting with TI's DRV8313 IC. 
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
#include <math.h>
#include "stm32f4xx_hal_tim_pwm_ex.h"
#include "bldc_drv.h"

/* Private define ------------------------------------------------------------*/
#define DRV8313_WAKEUP_PERIOD                   5U              /* Wakeup period of DRV8313 in units of miliseconds */
#define DRV8313_RESET_PERIOD                    2U              /* Reset period of DRV8313 in units of miliseconds */

/* Private macro -------------------------------------------------------------*/
#define __ERROR(_HANDLE_, _ERR_CODE_)   \
                                          do {  \
                                                (_HANDLE_)->state = (BLDC_DRV_STATE_ERROR);                     \
                                                (_HANDLE_)->errCode = (_ERR_CODE_);                             \
                                                (_HANDLE_)->lockState = (BLDC_DRV_LOCK_STATE_UNLOCKED);         \
                                                return (BLDC_DRV_OP_RES_FAILURE);                               \
                                          } while(0)

#define __LOCK(_HANDLE_)                        ((_HANDLE_)->lockState = (BLDC_DRV_LOCK_STATE_LOCKED)) 
                                          
#define __UNLOCK(_HANDLE_)                      ((_HANDLE_)->lockState = (BLDC_DRV_LOCK_STATE_UNLOCKED))
                                          
#define IS_LOCKED(_HANDLE_)                     ((_HANDLE_)->lockState == (BLDC_DRV_LOCK_STATE_LOCKED))
                   
                                            
#define IS_DUTY_RATIO(_DUTY_RATIO_)             (((_DUTY_RATIO_) >= 0.0f) && ((_DUTY_RATIO_) <= 1.0f))
#define IS_SHIFT_RATIO(_SHIFT_RATIO_)            (((_SHIFT_RATIO_) >= -1.0f) && ((_SHIFT_RATIO_) <= 1.0f))

/* Private function prototypes -----------------------------------------------*/
static ErrorStatus updatePwmChannels(BldcDrv_Handle_t *pHandle, float dutyRatio, 
                                     float phaseAngle);
static float EffSin(float angle);

/* Exported functions --------------------------------------------------------*/

/**
  * @brief  Initializes handle.
  *
  * @param  pHandle: Handle.
  * @param  pInitStruct: Initializer object.
  *
  * @retval Operation result.
  */
BldcDrv_OpRes_t BldcDrv_Init(BldcDrv_Handle_t *pHandle, BldcDrv_Init_t *pInitStruct)
{
  /* Check if the module is at error state */
  if (pHandle->state == BLDC_DRV_STATE_ERROR)
  {
    return BLDC_DRV_OP_RES_FAILURE;
  }
  
  __LOCK(pHandle);
  
  /* Check for state compatibility */
  if (pHandle->state != BLDC_DRV_STATE_UNINIT)
  {
    __ERROR(pHandle, BLDC_DRV_ERR_CODE_INCOMPATIBLE_STATE);
  }
  
  /* Check if the timer module has been initialized */
  if (pInitStruct->hTim->State != HAL_TIM_STATE_READY)
  {
    __ERROR(pHandle, BLDC_DRV_ERR_CODE_HAL_TIM_ERROR);
  }

  /* Set parameters */
  pHandle->init.hTim = pInitStruct->hTim;
  pHandle->init.poleCount = pInitStruct->poleCount;
  pHandle->init.pinMap.inRChannel = pInitStruct->pinMap.inRChannel;
  pHandle->init.pinMap.inSChannel = pInitStruct->pinMap.inSChannel;
  pHandle->init.pinMap.inTChannel = pInitStruct->pinMap.inTChannel;
  pHandle->init.pinMap.nFaultPort = pInitStruct->pinMap.nFaultPort;
  pHandle->init.pinMap.nFaultPin = pInitStruct->pinMap.nFaultPin;
  pHandle->init.pinMap.nResetPort = pInitStruct->pinMap.nResetPort;
  pHandle->init.pinMap.nResetPin = pInitStruct->pinMap.nResetPin;
  pHandle->init.pinMap.nSleepPort = pInitStruct->pinMap.nSleepPort;
  pHandle->init.pinMap.nSleepPin = pInitStruct->pinMap.nSleepPin;
  
  /* Set module's error code to none */
  pHandle->errCode = BLDC_DRV_ERR_CODE_NONE;
  
  /* Set module's state to ready */
  pHandle->state = BLDC_DRV_STATE_READY;
 
  __UNLOCK(pHandle);
  
  return BLDC_DRV_OP_RES_SUCCESS;
}

/**
  * @brief  Starts module.
  *
  * @param  pHandle: Pointer of the handle.
  *
  * @retval Operation result.
  */
BldcDrv_OpRes_t BldcDrv_Start(BldcDrv_Handle_t *pHandle)
{
  /* Check if the module is at error state */
  if (pHandle->state == BLDC_DRV_STATE_ERROR)
  {
    return BLDC_DRV_OP_RES_FAILURE;
  }
  
  /* Check if the module is locked */
  if (IS_LOCKED(pHandle))
  {
    __ERROR(pHandle, BLDC_DRV_ERR_CODE_RACE_CONDITION);
  }
  
  __LOCK(pHandle);
  
  /* Check for state compability */
  if (pHandle->state != BLDC_DRV_STATE_READY)
  {
    __ERROR(pHandle, BLDC_DRV_ERR_CODE_INCOMPATIBLE_STATE);
  }

  /* Start static variables */
  pHandle->statics.direction = BLDC_DRV_ROT_DIR_POSITIVE;
  pHandle->statics.offset = 0.0f;
  
  /* Set nRESET pin to high(so the device is not at reset condition) */
  HAL_GPIO_WritePin(pHandle->init.pinMap.nResetPort, 
                    pHandle->init.pinMap.nResetPin, 
                    GPIO_PIN_SET);
  
  DelayMs(DRV8313_RESET_PERIOD);
  
  /* Set nSLEEP pin to high */
  HAL_GPIO_WritePin(pHandle->init.pinMap.nSleepPort, 
                    pHandle->init.pinMap.nSleepPin, 
                    GPIO_PIN_SET);
  
  DelayMs(DRV8313_WAKEUP_PERIOD);
  
  /* Read DRV8313's nFAULT pin */
  if (HAL_GPIO_ReadPin(pHandle->init.pinMap.nFaultPort, 
                       pHandle->init.pinMap.nFaultPin) == GPIO_PIN_RESET)
  {
    __ERROR(pHandle, BLDC_DRV_ERR_CODE_DRV8313_ERROR);
  }

  /* Set initial phase duty cycles */
  if (updatePwmChannels(pHandle, BLDC_DRV_INITIAL_DUTY_RATIO, 
                        BLDC_DRV_INITIAL_POSITION) != SUCCESS)
  {
    __ERROR(pHandle, BLDC_DRV_ERR_CODE_HAL_TIM_ERROR);
  }
  
  /* Start PWM generation */
  if ((HAL_TIM_PWM_Start(pHandle->init.hTim, 
                         pHandle->init.pinMap.inRChannel) != HAL_OK) || \
      (HAL_TIM_PWM_Start(pHandle->init.hTim, 
                         pHandle->init.pinMap.inSChannel) != HAL_OK) || \
      (HAL_TIM_PWM_Start(pHandle->init.hTim, 
                         pHandle->init.pinMap.inTChannel) != HAL_OK))
  {
    __ERROR(pHandle, BLDC_DRV_ERR_CODE_HAL_TIM_ERROR);
  }
  
  /* Set module's state to operating */
  pHandle->state = BLDC_DRV_STATE_OPERATING;
  
  __UNLOCK(pHandle);
  
  return BLDC_DRV_OP_RES_SUCCESS;
}

/**
  * @brief  Executes handle. Should be called periodically 
  *         for driving the brushless motor properly.
  *
  * @param  pHandle: Handle.
  * @param  position: Destinated rotor position of the motor.
  * @param  dutyRatio: Controls the duty cycle.
  * @param  shiftRatio: Controls the phase shift between electrical angle and 
  *         rotor position.
  *      
  * @retval Operation result.
  */
BldcDrv_OpRes_t BldcDrv_Execute(BldcDrv_Handle_t *pHandle, float position, 
                                float dutyRatio, float shiftRatio)
{
  /* Check if the module is at error state */
  if (pHandle->state == BLDC_DRV_STATE_ERROR)
  {
    return BLDC_DRV_OP_RES_FAILURE;
  }
  
  /* Check if the module is locked */
  if (IS_LOCKED(pHandle))
  {
    __ERROR(pHandle, BLDC_DRV_ERR_CODE_RACE_CONDITION);
  }
  
  __LOCK(pHandle);
  
  /* Check parameters */
  if ((!IS_DUTY_RATIO(dutyRatio)) || (!IS_SHIFT_RATIO(shiftRatio)))
  {
    __ERROR(pHandle, BLDC_DRV_ERR_CODE_INVALID_PARAMETER);
  }
 
  /* Check for state compatibility */
  if (pHandle->state != BLDC_DRV_STATE_OPERATING)
  {
    __ERROR(pHandle, BLDC_DRV_ERR_CODE_INCOMPATIBLE_STATE);
  }

  /* Read DRV8313's nFAULT pin */
  if (HAL_GPIO_ReadPin(pHandle->init.pinMap.nFaultPort, 
                       pHandle->init.pinMap.nFaultPin) == GPIO_PIN_RESET)
  {
    __ERROR(pHandle, BLDC_DRV_ERR_CODE_DRV8313_ERROR);
  }

  float phase_angle;
  
  /* Calculate new phase angle */
  if (pHandle->statics.direction == BLDC_DRV_ROT_DIR_POSITIVE)
  {
    phase_angle = (position + ((float)shiftRatio * PI / pHandle->init.poleCount)) + \
                  pHandle->statics.offset;
  }
  else
  {
    phase_angle = -(position + ((float)shiftRatio * PI / pHandle->init.poleCount)) + \
                  pHandle->statics.offset;
  }
  

  /* Update phase duty cycles */
  if (updatePwmChannels(pHandle, dutyRatio, phase_angle) != SUCCESS)
  {
    __ERROR(pHandle, BLDC_DRV_ERR_CODE_HAL_TIM_ERROR);
  }
  
  __UNLOCK(pHandle);
  
  return BLDC_DRV_OP_RES_SUCCESS;
}

/**
  * @brief  Sets direction of the BLDC motor.
  *
  * @param  direction: Positive or negative rotation direction in right handed 
  *         coordinate system.
  * @param  offset: Angular difference in radians from electrical position and 
  *         real position.
  *     
  * @retval Operation result.
  */
BldcDrv_OpRes_t BldcDrv_SetParameters(BldcDrv_Handle_t *pHandle, 
                                      BldcDrv_Direction_t direction, float offset)
{
  /* Check if the module is at error state */
  if (pHandle->state == BLDC_DRV_STATE_ERROR)
  {
    return BLDC_DRV_OP_RES_FAILURE;
  }
  
  /* Check if the module is locked */
  if (IS_LOCKED(pHandle))
  {
    __ERROR(pHandle, BLDC_DRV_ERR_CODE_RACE_CONDITION);
  }
  
  __LOCK(pHandle);
  
  float reduced_offset;
  reduced_offset = offset;
  
  /* Calculate reduced offset */
  while (reduced_offset > PI)
  {
    reduced_offset -= (2.0f * PI);
  }
  
  while (reduced_offset < -PI)
  {
    reduced_offset += (2.0F * PI);
  }
  
  /* Set parameters */
  pHandle->statics.direction = direction;
  pHandle->statics.offset = reduced_offset;
  
  __UNLOCK(pHandle);
  
  return BLDC_DRV_OP_RES_SUCCESS;
}
                    
/**
  * @brief  Stops the module.
  *
  * @param  pHandle: Pointer of the handle.
  *
  * @retval Operation result.
  */
BldcDrv_OpRes_t BldcDrv_Stop(BldcDrv_Handle_t *pHandle)
{
  /* Check if the module is at error state */
  if (pHandle->state == BLDC_DRV_STATE_ERROR)
  {
    return BLDC_DRV_OP_RES_FAILURE;
  }
  
  /* Check if the module is locked */
  if (IS_LOCKED(pHandle))
  {
    __ERROR(pHandle, BLDC_DRV_ERR_CODE_RACE_CONDITION);
  }
  
  __LOCK(pHandle);

  /* Check for state compatibility */
  if (pHandle->state != BLDC_DRV_STATE_OPERATING)
  {
    __ERROR(pHandle, BLDC_DRV_ERR_CODE_INCOMPATIBLE_STATE);
  }
  
  /* Stop PWM generation */
  if ((HAL_TIM_PWM_Stop(pHandle->init.hTim, 
                        pHandle->init.pinMap.inRChannel) != HAL_OK) || \
      (HAL_TIM_PWM_Stop(pHandle->init.hTim, 
                        pHandle->init.pinMap.inSChannel) != HAL_OK) || \
      (HAL_TIM_PWM_Stop(pHandle->init.hTim, 
                        pHandle->init.pinMap.inTChannel) != HAL_OK))
  {
    __ERROR(pHandle, BLDC_DRV_ERR_CODE_HAL_TIM_ERROR);
  }
  
  /* Set nSLEEP pin to low(this puts motor driver into sleep mode) */
  HAL_GPIO_WritePin(pHandle->init.pinMap.nSleepPort, 
                    pHandle->init.pinMap.nSleepPin, 
                    GPIO_PIN_RESET);
  
  /* Set module's state to ready */
  pHandle->state = BLDC_DRV_STATE_READY;
  
  __UNLOCK(pHandle);
  
  return BLDC_DRV_OP_RES_SUCCESS;
}

/**
  * @brief  Resets the module. Switches to uninitialized state.
  *         
  * @param  pHandle: Pointer of the handle.
  *
  * @retval Operation result.
  */
BldcDrv_OpRes_t BldcDrv_Reset(BldcDrv_Handle_t *pHandle)
{
  __LOCK(pHandle);
  
  /* Stop PWM generation */
  if ((HAL_TIM_PWM_Stop(pHandle->init.hTim, 
                         pHandle->init.pinMap.inRChannel) != HAL_OK) || \
      (HAL_TIM_PWM_Stop(pHandle->init.hTim, 
                         pHandle->init.pinMap.inSChannel) != HAL_OK) || \
      (HAL_TIM_PWM_Stop(pHandle->init.hTim, 
                         pHandle->init.pinMap.inTChannel) != HAL_OK))
  {
    __ERROR(pHandle, BLDC_DRV_ERR_CODE_HAL_TIM_ERROR);
  }
  
  /* Set module's state to ready */
  pHandle->state = BLDC_DRV_STATE_UNINIT;
  
  __UNLOCK(pHandle);
  
  return BLDC_DRV_OP_RES_SUCCESS;
}

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Updates pwm channel duty cycles according to given phase angle.
  *
  * @param  pHandle: Brushless DC motor driver object handle
  * @param  dutyRatio: Duty ratio multiplier for driving phase currents
  * @param  phaseAngle: Phase angle of the generated three phase magnetic field.
  *
  * @retval SUCCESS or ERROR.
  */
static ErrorStatus updatePwmChannels(BldcDrv_Handle_t *pHandle, float dutyRatio, float phaseAngle)
{
  float r_mult, s_mult, t_mult;
  uint16_t r_value, s_value, t_value;
  float pole_count, max_pulse;
  
  max_pulse = (float)(pHandle->init.hTim->Init.Period - 1);
  pole_count = (float)pHandle->init.poleCount;
  
  /* Calculate the phase values */
  r_mult = EffSin(((pole_count * phaseAngle * 0.5f) - (2.0f * PI / 3.0f)));
  s_mult = EffSin((pole_count * phaseAngle * 0.5f));
  t_mult = EffSin(((pole_count * phaseAngle * 0.5f) + (2.0f * PI / 3.0f)));
  
  r_value = (uint16_t)round((max_pulse * 0.5f * (1.0f + dutyRatio * 0.99f * r_mult)));
  s_value = (uint16_t)round((max_pulse * 0.5f * (1.0f + dutyRatio * 0.99f * s_mult)));
  t_value = (uint16_t)round((max_pulse * 0.5f * (1.0f + dutyRatio * 0.99f * t_mult)));
  
  /* Set pulse value of the channels */
  if ((HAL_TIM_PWMEx_SetPulse(pHandle->init.hTim, r_value, 
                              pHandle->init.pinMap.inRChannel) != HAL_OK) || \
      (HAL_TIM_PWMEx_SetPulse(pHandle->init.hTim, s_value, 
                              pHandle->init.pinMap.inSChannel) != HAL_OK) || \
      (HAL_TIM_PWMEx_SetPulse(pHandle->init.hTim, t_value, 
                              pHandle->init.pinMap.inTChannel) != HAL_OK))
  {
    return ERROR;
  }
  
  return SUCCESS;
}

/**
  * @brief  Efficient algorithm to take the sine of an angle.
  *
  * @param  angle: Input in units of radians.
  *
  * @retval sin(angle)
  */
static float EffSin(float angle)
{
  float angle_mod;
  float angle_mod2th;
  float angle_mod3th;
  float angle_mod5th;
  float angle_mod7th;
  float result;
  
  angle_mod = angle;
  
  /* Find angle's modulus */
  while (angle_mod > PI)
  {
    angle_mod -= (2.0 * PI);
  }
  
  while (angle_mod < -PI)
  {
    angle_mod += (2.0 * PI);
  }
  
  if (angle_mod > (PI / 2.0))
  {
    angle_mod = PI - angle_mod;
  }
  else if (angle_mod < (-PI / 2.0))
  {
    angle_mod = -PI - angle_mod;
  }
  
  /* Calculate power's of the angle modulus, which is required for calculation */
  angle_mod2th = angle_mod * angle_mod;
  angle_mod3th = angle_mod * angle_mod2th;
  angle_mod5th = angle_mod2th * angle_mod3th;
  angle_mod7th = angle_mod2th * angle_mod5th;
 
  /* Calculate the result using chebyshev approximation */
  result = (0.99999660f * angle_mod) + (-0.16664824f * angle_mod3th) + \
           (0.00830629f * angle_mod5th) + (-0.00018363f * angle_mod7th);
  
  return result;
}