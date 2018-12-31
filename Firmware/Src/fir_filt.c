/**
  ******************************************************************************
  * @file    fir_filt.c
  * @author  Onur Efe
  * @date    17.05.2016
  * @brief   Finite impulse response filter class implementation.
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
#include "fir_filt.h"

/* Private macros ------------------------------------------------------------*/
#define IS_FILTER_ORDER(_ORDER_) \
(((_ORDER_) >= FIR_FILT_MIN_FILTER_ORDER) && ((_ORDER_) <= FIR_FILT_MAX_FILTER_ORDER))

/* Exported functions --------------------------------------------------------*/

/**
  * @brief  FIR filter module initializer. Initializes constant members.
  *
  * @param  pFirFiltHandle: Pointer to filter module handle.
  * @param  pFirFiltInit: Pointer filter module initializer object.
  *
  * @retval Operation result.
  */
FirFilt_OpRes_t FirFilt_Init(FirFilt_Handle_t *pFirFiltHandle, FirFilt_Init_t *pFirFiltInit)
{
  /* Check if the filter order is between limits */
  assert_param(IS_FILTER_ORDER(pFirFiltInit->filterOrder));

  /* Check if the module's state is compatible with the operation */
  if (pFirFiltHandle->state != FIR_FILT_STATE_UNINIT)
  {
    return FIR_FILT_OP_RES_INCOMPATIBLE_STATE;
  }
  
  /* Set FIR filter state to busy */
  pFirFiltHandle->state = FIR_FILT_STATE_BUSY;
  
  /* Calculate coefficient vector length */
  uint8_t coeffs_len;
  coeffs_len = pFirFiltInit->filterOrder + 1;
  
  float sum_of_coeffs = 0.0;
  
  /* Calculate filter gain */
  for (uint8_t i = 0; i < coeffs_len; i++)
  {
    sum_of_coeffs += pFirFiltInit->pCoeffs[i];
  }
  
  /* Normalize coefficient vector according to make the gain unity */
  for (uint8_t i = 0; i < coeffs_len; i++)
  {
    pFirFiltHandle->privates.pCoeffs[i] = pFirFiltInit->pCoeffs[i] / sum_of_coeffs;
  }
  
  /* Set constants */
  pFirFiltHandle->privates.filterOrder = pFirFiltInit->filterOrder;
  pFirFiltHandle->privates.pCoeffs = pFirFiltInit->pCoeffs;
  
  /* Set module's state to ready */
  pFirFiltHandle->state = FIR_FILT_STATE_READY;
  
  return FIR_FILT_OP_RES_SUCCESS;
}

/**
  * @brief  Starts module.
  *
  * @param  pFirFiltHandle: Pointer to filter module handle.
  *
  * @retval Operation result.
  */
FirFilt_OpRes_t FirFilt_Start(FirFilt_Handle_t *pFirFiltHandle)
{
  /* Check if the module's state is compatible for the operation */
  if (pFirFiltHandle->state != FIR_FILT_STATE_READY)
  {
    return FIR_FILT_OP_RES_INCOMPATIBLE_STATE;
  }
  
  /* Set module's state to busy */
  pFirFiltHandle->state = FIR_FILT_STATE_BUSY;
  
  /* Initialize static variables */
  /* Calculate circular buffer length */
  uint8_t buffer_len;
  buffer_len = pFirFiltHandle->privates.filterOrder + 1;
  
  /* Initialize circular buffer with zeros */
  for (uint8_t i = 0; i < buffer_len; i++)
  {
    pFirFiltHandle->privates.circBuffer[i] = 0.0;
  }
  
  /* Start index */
  pFirFiltHandle->privates.startIndex = 0;
  
  /* Set module's state to operating */
  pFirFiltHandle->state = FIR_FILT_STATE_OPERATING;
  
  return FIR_FILT_OP_RES_SUCCESS;
}

/**
  * @brief  FIR filter module executer.
  *
  * @param  pFirFiltHandle: Pointer to filter module handle.
  * @param  inputSignal: Present input signal.
  * @param  pOutputSignal: Pointer to store output signal.
  *
  * @retval Operation result.
  */
FirFilt_OpRes_t FirFilt_Execute(FirFilt_Handle_t *pFirFiltHandle, float inputSignal, 
                                float *pOutputSignal)
{
  /* Check if the module's state is compatible for the operation */
  if (pFirFiltHandle->state != FIR_FILT_STATE_OPERATING)
  {
    return FIR_FILT_OP_RES_INCOMPATIBLE_STATE;
  }

  /* Set module's state to busy */
  pFirFiltHandle->state = FIR_FILT_STATE_BUSY;
  
  /* Calculate circular buffer size */
  uint8_t buffer_size;
  buffer_size = pFirFiltHandle->privates.filterOrder + 1;
  
  /* Make the recent start index to current write index */
  uint8_t write_index;
  write_index = pFirFiltHandle->privates.startIndex;
     
  /* Write input signal to the buffer */
  pFirFiltHandle->privates.circBuffer[write_index] = inputSignal;
    
  /* Update start index */
  if (++(pFirFiltHandle->privates.startIndex) >= buffer_size)
  {
    pFirFiltHandle->privates.startIndex -= buffer_size;
  }
  
  /* Set read index to the initial start index */
  uint8_t read_index;
  read_index = pFirFiltHandle->privates.startIndex;
  
  float weighted_sum = 0.0;
  
  /* Calculate the filter response */
  for (uint8_t i = 0; i < buffer_size; i++)
  {
    weighted_sum += (pFirFiltHandle->privates.circBuffer[read_index] * \
                     pFirFiltHandle->privates.pCoeffs[i]);
      
    if (++read_index >= buffer_size)
    {
      read_index -= buffer_size;
    }
  }
  
  *pOutputSignal = weighted_sum;
  
  /* Set module's state back to operating */
  pFirFiltHandle->state = FIR_FILT_STATE_OPERATING;
  
  return FIR_FILT_OP_RES_SUCCESS;
}

/**
  * @brief  Stops module.
  *
  * @param  pFirFiltHandle: Pointer to filter module handle.
  *
  * @retval Operation result.
  */
FirFilt_OpRes_t FirFilt_Stop(FirFilt_Handle_t *pFirFiltHandle)
{
  /* Check if the module's state is compatible for the operation */
  if (pFirFiltHandle->state != FIR_FILT_STATE_OPERATING)
  {
    return FIR_FILT_OP_RES_INCOMPATIBLE_STATE;
  }
  
  /* Set module's state to ready */
  pFirFiltHandle->state = FIR_FILT_STATE_READY;
  
  return FIR_FILT_OP_RES_SUCCESS;
}

/**
  * @brief  Resets FIR filter module.
  *
  * @param  pFirFiltHandle: Pointer to filter module handle.
  *
  * @retval Operation result.
  */
FirFilt_OpRes_t FirFilt_Reset(FirFilt_Handle_t *pFirFiltHandle)
{
  /* Check for state compatibility */
  if (pFirFiltHandle->state == FIR_FILT_STATE_BUSY)
  {
    return FIR_FILT_OP_RES_INCOMPATIBLE_STATE;
  }
  
  /* Set module's state to initialized state */
  pFirFiltHandle->state = FIR_FILT_STATE_UNINIT;
  
  return FIR_FILT_OP_RES_SUCCESS;
}