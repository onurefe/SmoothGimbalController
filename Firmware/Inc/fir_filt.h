/**
  ******************************************************************************
  * @file    fir_filt.h
  * @author  Onur Efe
  * @date    17.05.2016
  * @brief   Finite impulse response(FIR) filter class interface.
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
#ifndef __FIR_FILT_H
#define __FIR_FILT_H

#ifdef __cplusplus
 extern "C" {
#endif
   
/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h"
#include "generic.h"
   
/* Exported constants --------------------------------------------------------*/
#define FIR_FILT_MAX_FILTER_ORDER       150             /* Maximum filter order */
#define FIR_FILT_MIN_FILTER_ORDER       1               /* Minimum filter order */

/* Exported types ------------------------------------------------------------*/
typedef enum
{
  FIR_FILT_STATE_UNINIT                 = 0x00,         /* Module is not initialized */
  FIR_FILT_STATE_READY                  = 0x01,         /* Module is ready to operate */
  FIR_FILT_STATE_BUSY                   = 0x02,         /* Internal process is going on */
  FIR_FILT_STATE_OPERATING              = 0x03          /* Module is operating */
} FirFilt_State_t;                                      /* State typedef */

typedef enum
{
  FIR_FILT_OP_RES_SUCCESS               = 0x00,         /* Success */
  FIR_FILT_OP_RES_INVALID_POINTER       = 0x01,         /* Invalid pointer */
  FIR_FILT_OP_RES_INVALID_PARAMETER     = 0x02,         /* Invalid parameter */
  FIR_FILT_OP_RES_INCOMPATIBLE_STATE    = 0x03,         /* Module's state is not compatible for the operation */
} FirFilt_OpRes_t;                                      /* Operation result typedef */

typedef struct
{
  uint8_t                               startIndex;     /* Start index for the circular buffer */
  uint8_t                               filterOrder;    /* Filter order */
  float                                 *pCoeffs;       /* Pointer to coefficient vector */
  float                                 circBuffer[FIR_FILT_MAX_FILTER_ORDER + 1];      /* Circular buffer */
} FirFilt_Privates_t;                                   /* Privates typedef */

typedef struct
{
  float                                 *pCoeffs;       /* Pointer to coefficients vector */
  uint8_t                               filterOrder;    /* Filter order */
} FirFilt_Init_t;                                       /* Init typedef */

typedef struct
{
  FirFilt_State_t                       state;          /* Module state */
  FirFilt_Privates_t                    privates;       /* Private variables */
} FirFilt_Handle_t;

/* Exported functions --------------------------------------------------------*/
/**
  * @brief  FIR filter module initializer.
  *
  * @param  pFirFiltHandle: Pointer to filter module handle.
  * @param  pFirFiltInit: Pointer filter module initializer object.
  *
  * @retval Operation result.
  */
FirFilt_OpRes_t FirFilt_Init(FirFilt_Handle_t *pFirFiltHandle, FirFilt_Init_t *pFirFiltInit);

/**
  * @brief  Starts module.
  *
  * @param  pFirFiltHandle: Pointer to filter module handle.
  *
  * @retval Operation result.
  */
FirFilt_OpRes_t FirFilt_Start(FirFilt_Handle_t *pFirFiltHandle);

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
                                float *pOutputSignal);
/**
  * @brief  Stops module.
  *
  * @param  pFirFiltHandle: Pointer to filter module handle.
  *
  * @retval Operation result.
  */
FirFilt_OpRes_t FirFilt_Stop(FirFilt_Handle_t *pFirFiltHandle);

/**
  * @brief  Resets FIR filter module.
  *
  * @param  pFirFiltHandle: Pointer to filter module handle.
  *
  * @retval Operation result.
  */
FirFilt_OpRes_t FirFilt_Reset(FirFilt_Handle_t *pFirFiltHandle);

#ifdef __cplusplus
}
#endif

#endif