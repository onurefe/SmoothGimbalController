/**
  ******************************************************************************
  * @file    pid_ctrl.c
  * @author  Onur Efe
  * @date    17.05.2016
  * @brief   PID Controller class implementation.
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
#include "pid_ctrl.h"

/* Private macros ------------------------------------------------------------*/
#define __ERROR(_HANDLE_, _ERR_CODE_)   \
                                                   do {                                                         \
                                                        (_HANDLE_)->state = (PID_CTRL_STATE_ERROR);             \
                                                        (_HANDLE_)->errCode = (_ERR_CODE_);                     \
                                                        (_HANDLE_)->lockState = (PID_CTRL_LOCK_STATE_UNLOCKED); \
                                                        return (PID_CTRL_OP_RES_FAILURE);                   \
                                                      } while (0)

#define __LOCK(_HANDLE_)                                ((_HANDLE_)->lockState = (PID_CTRL_LOCK_STATE_LOCKED)) 
                                          
#define __UNLOCK(_HANDLE_)                              ((_HANDLE_)->lockState = (PID_CTRL_LOCK_STATE_UNLOCKED))
                                          
#define IS_LOCKED(_HANDLE_)                             ((_HANDLE_)->lockState == (PID_CTRL_LOCK_STATE_LOCKED))
                                            
#define CONTROL_PERIOD                                  (1.0f / CONTROL_FREQ)
#define IS_OUTPUT_LIMITS(_LOW_LIMIT_, _HIGH_LIMIT_)     ((_LOW_LIMIT_) < (_HIGH_LIMIT_))
#define IS_INTEGRAL_TIME(_INTEGRAL_TIME_)               ((_INTEGRAL_TIME_) > 0.0f)

/* Exported functions --------------------------------------------------------*/

/**
  * @brief  Initializes module. 
  *
  * @param  pHandle: Pointer to module handle.
  * @param  pInitStruct: Struct which contains parameters used with initializing the module.
  *
  * @retval Operation result.
  */
PidCtrl_OpRes_t PidCtrl_Init(PidCtrl_Handle_t *pHandle, PidCtrl_Init_t *pInitStruct)
{
  /* Check if the module is at error state */
  if (pHandle->state == PID_CTRL_STATE_ERROR)
  {
    return PID_CTRL_OP_RES_FAILURE;
  }
  
  __LOCK(pHandle);
  
  /* Validate parameters */
  if ((!IS_INTEGRAL_TIME(pInitStruct->integralTC)) || \
      (!IS_OUTPUT_LIMITS(pInitStruct->lowLimit, pInitStruct->highLimit)))
  {
    __ERROR(pHandle, PID_CTRL_ERR_CODE_INVALID_PARAMETER);
  }
  
  /* Check if the module's state is compatible for the operation */
  if (pHandle->state != PID_CTRL_STATE_UNINIT)
  {
    __ERROR(pHandle, PID_CTRL_ERR_CODE_INCOMPATIBLE_STATE);
  }

  /* Set the constant parameters */
  pHandle->init.gain = pInitStruct->gain;
  pHandle->init.integralTC = pInitStruct->integralTC;
  pHandle->init.derivativeTC = pInitStruct->derivativeTC;
  pHandle->init.filterTC = pInitStruct->filterTC;
  pHandle->init.lowLimit = pInitStruct->lowLimit;
  pHandle->init.highLimit = pInitStruct->highLimit;
  
  /* Set error code to none */
  pHandle->errCode = PID_CTRL_ERR_CODE_NONE;
  
  /* Set module's state to ready */
  pHandle->state = PID_CTRL_STATE_READY;
  
  __UNLOCK(pHandle);

  return PID_CTRL_OP_RES_SUCCESS;
}

/**
  * @brief  Starts module. 
  *
  * @param  pHandle: Pointer to module handle.
  *
  * @retval Operation result.
  */
PidCtrl_OpRes_t PidCtrl_Start(PidCtrl_Handle_t *pHandle)
{
  /* Check if the module is at error state */
  if (pHandle->state == PID_CTRL_STATE_ERROR)
  {
    return PID_CTRL_OP_RES_FAILURE;
  }
  
  if (IS_LOCKED(pHandle))
  {
    __ERROR(pHandle, PID_CTRL_ERR_CODE_RACE_CONDITION);
  }
  
  __LOCK(pHandle);
  
  /* Check if the module's state is compatible for the operation */
  if (pHandle->state != PID_CTRL_STATE_READY)
  {
    __ERROR(pHandle, PID_CTRL_ERR_CODE_INCOMPATIBLE_STATE);
  }

  /* Initialize static variables */
  pHandle->statics.lastInput = 0.0f;
  pHandle->statics.lastIntOutput = 0.0f;
  pHandle->statics.lastFiltOutput = 0.0f;
  pHandle->statics.filterMultiplier = CONTROL_PERIOD / \
                                      (pHandle->init.filterTC + CONTROL_PERIOD);
  
  /* Set module's state to operating */
  pHandle->state = PID_CTRL_STATE_OPERATING;
  
  __UNLOCK(pHandle);
  
  return PID_CTRL_OP_RES_SUCCESS;
}

/**
  * @brief  Module executer. Should be called periodically for proper
  *         operation.
  *
  * @param  pHandle: Pointer to the module's handle.
  * @param  input: Input signal of the PID controller.
  * @param  pOutput: Pointer to store output signal.
  *
  * @retval Operation result.
  */
PidCtrl_OpRes_t PidCtrl_Execute(PidCtrl_Handle_t *pHandle, float input, float *pOutput)
{
  /* Check if the module is at error state */
  if (pHandle->state == PID_CTRL_STATE_ERROR)
  {
    return PID_CTRL_OP_RES_FAILURE;
  }
  
  if (IS_LOCKED(pHandle))
  {
    __ERROR(pHandle, PID_CTRL_ERR_CODE_RACE_CONDITION);
  }
  
  __LOCK(pHandle);
  
  /* Check if the module's state is compatible with the operation */
  if (pHandle->state != PID_CTRL_STATE_OPERATING)
  {
    __ERROR(pHandle, PID_CTRL_ERR_CODE_INCOMPATIBLE_STATE);
  }
  
  float filter_output;

  filter_output = (input * pHandle->statics.filterMultiplier) + \
                  (pHandle->statics.lastFiltOutput * (1.0f - pHandle->statics.filterMultiplier));
  
  /* Calculate integral block output signal */
  float int_output_sign;
  int_output_sign = pHandle->statics.lastIntOutput + \
                    ((pHandle->statics.lastInput + input) / (2.0f * CONTROL_FREQ));

  /* Calculate derivative block output signal */
  float der_output_sign;
  der_output_sign = (filter_output - pHandle->statics.lastFiltOutput) * CONTROL_FREQ;
    
  /* Calculate total output */
  float pid_output_sign;
  pid_output_sign = (input + (int_output_sign / pHandle->init.integralTC) + \
                    (der_output_sign * pHandle->init.derivativeTC)) * \
                     pHandle->init.gain;
  
  /* Limit the output value */
  if (pid_output_sign > pHandle->init.highLimit)
  {
    pid_output_sign = pHandle->init.highLimit;
  }
  else if (pid_output_sign < pHandle->init.lowLimit)
  {
    pid_output_sign = pHandle->init.lowLimit;
  }
  else
  {
    /* Update integral variable if integral windup didn't occur */
    pHandle->statics.lastIntOutput = int_output_sign;
  }
  
  /* Update static variables */
  pHandle->statics.lastInput = input;
  pHandle->statics.lastFiltOutput = filter_output;

  /* Set output signal */
  *pOutput = pid_output_sign;
  
  __UNLOCK(pHandle);

  return PID_CTRL_OP_RES_SUCCESS;
}

/**
  * @brief  Stops operating module.
  * 
  * @param  pHandle: Pointer to the module handle.
  *
  * @retval Operation result.
  */
PidCtrl_OpRes_t PidCtrl_Stop(PidCtrl_Handle_t *pHandle)
{
  /* Check if the module is at error state */
  if (pHandle->state == PID_CTRL_STATE_ERROR)
  {
    return PID_CTRL_OP_RES_FAILURE;
  }
  
  if (IS_LOCKED(pHandle))
  {
    __ERROR(pHandle, PID_CTRL_ERR_CODE_RACE_CONDITION);
  }
  
  __LOCK(pHandle);
  
  /* Check if the module's state is compatible for the operation */
  if (pHandle->state != PID_CTRL_STATE_OPERATING)
  {
    __ERROR(pHandle, PID_CTRL_ERR_CODE_INCOMPATIBLE_STATE);
  }

  /* Set module's state to ready */
  pHandle->state = PID_CTRL_STATE_READY;
  
  __UNLOCK(pHandle);
  
  return PID_CTRL_OP_RES_SUCCESS;
}

/**
  * @brief  Resets module.
  *
  * @param  pHandle: Pointer to the module's handle.
  *
  * @retval Operation result.
  */
PidCtrl_OpRes_t PidCtrl_Reset(PidCtrl_Handle_t *pHandle)
{
  __LOCK(pHandle);
  
  /* Set module's state to UNINIT */
  pHandle->state = PID_CTRL_STATE_UNINIT;
  
  __UNLOCK(pHandle);
  
  return PID_CTRL_OP_RES_SUCCESS;
}