/**
  ******************************************************************************
  * @file    i2c_manager.c
  * @author  Onur Efe
  * @date    03.06.2016
  * @brief   I2C manager module implementation.
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
#include "i2c_manager.h"

/* Private constants ---------------------------------------------------------*/
#define MAX_MODULE_COUNT                        5U                   
#define MEM_ADDR_SIZE                           1U
#define MAX_NUM_OF_SUCC_ERRORS                  10U

#define RX_COMPLETED_CB                         0x00
#define TX_COMPLETED_CB                         0x01
#define ERROR_CB                                0x02

/* Private macros ------------------------------------------------------------*/
#define __ERROR(_HANDLE_, _ERR_CODE_)   \
                                          do {                                                                  \
                                                (_HANDLE_)->state = (I2C_MANAGER_STATE_ERROR);                  \
                                                (_HANDLE_)->errCode = (_ERR_CODE_);                             \
                                                (_HANDLE_)->lockState = (I2C_MANAGER_LOCK_STATE_UNLOCKED);      \
                                                return (I2C_MANAGER_OP_RES_FAILURE);                            \
                                          } while(0)
                                            
#define __ERROR_INT(_HANDLE_, _ERR_CODE_)       \
                                          do {                                                                  \
                                                (_HANDLE_)->state = (I2C_MANAGER_STATE_ERROR);                  \
                                                (_HANDLE_)->errCode = (_ERR_CODE_);                             \
                                                (_HANDLE_)->lockState = (I2C_MANAGER_LOCK_STATE_UNLOCKED);      \
                                                I2CManager_ErrorCallback((_HANDLE_));                           \
                                                return;                                                         \
                                          } while(0)

#define __LOCK(_HANDLE_)                        ((_HANDLE_)->lockState = (I2C_MANAGER_LOCK_STATE_LOCKED)) 
                                          
#define __UNLOCK(_HANDLE_)                      ((_HANDLE_)->lockState = (I2C_MANAGER_LOCK_STATE_UNLOCKED))
                                          
#define IS_LOCKED(_HANDLE_)                     ((_HANDLE_)->lockState == (I2C_MANAGER_LOCK_STATE_LOCKED))

/* Private functions ---------------------------------------------------------*/
static void memCBHandler(I2C_HandleTypeDef *hi2c, uint8_t memCBType);
static ErrorStatus processRequest(I2CManager_Handle_t *pHandle, I2CManager_Request_t *pRequest);
static void front(I2CManager_ReqQueue_t *Q, I2CManager_Request_t *pElement);
static void enqueue(I2CManager_ReqQueue_t *Q, I2CManager_Request_t *pElement);
static void dequeue(I2CManager_ReqQueue_t *Q);

/* Private variables ---------------------------------------------------------*/
static I2CManager_Handle_t *RegisteredModules[MAX_MODULE_COUNT];
static uint8_t NumOfModules = 0;

/* Exported functions --------------------------------------------------------*/

/**
  * @brief  Initializes module. Module's state is switched from uninitialized state
  *         to ready.
  *
  * @param  pHandle: Pointer to module's handle.
  * @param  pInitStruct: Pointer to module's init struct.
  *
  * @retval None.
  */
I2CManager_OpRes_t I2CManager_Init(I2CManager_Handle_t *pHandle, 
                                   I2CManager_Init_t *pInitStruct)
{
  /* Check if the module is at error state */
  if (pHandle->state == I2C_MANAGER_STATE_ERROR)
  {
    return I2C_MANAGER_OP_RES_FAILURE;
  }
  
  /* Lock module */
  __LOCK(pHandle);
  
  /* Check if the module's state is compatible with the operation */
  if (pHandle->state != I2C_MANAGER_STATE_UNINIT)
  {
    __ERROR(pHandle, I2C_MANAGER_ERR_CODE_INCOMPATIBLE_STATE);
  }
  
  /* Check if the i2c module has been initialized */
  if (pInitStruct->hi2c->State != HAL_I2C_STATE_READY)
  {
    /* Set error flags and return */
    __ERROR(pHandle, I2C_MANAGER_ERR_CODE_HAL_I2C_ERROR);
  }
  
  /* Check if the maximum registered object limit exceeded */
  if (NumOfModules >= (MAX_MODULE_COUNT - 1))
  {
    __ERROR(pHandle, I2C_MANAGER_ERR_CODE_REG_OBJ_LIM_EXCEEDED);
  }
  
  /* Register module's pointer */
  RegisteredModules[NumOfModules++] = pHandle;
  
  /* Set module's I2C handle */
  pHandle->init.hi2c = pInitStruct->hi2c;
  
  /* Set module's error code to none */
  pHandle->errCode = I2C_MANAGER_ERR_CODE_NONE;
  
  /* Set module's state to ready */
  pHandle->state = I2C_MANAGER_STATE_READY;
  
  /* Unlock the module */
  __UNLOCK(pHandle);
  
  return I2C_MANAGER_OP_RES_SUCCESS;
}

/**
  * @brief  Switches module to operating state(when the module is in ready state).
  *
  * @param  pHandle: Pointer to module's handle.
  *
  * @retval None.
  */
I2CManager_OpRes_t I2CManager_Start(I2CManager_Handle_t *pHandle)
{
  /* Check if the module is at error state */
  if (pHandle->state == I2C_MANAGER_STATE_ERROR)
  {
    return I2C_MANAGER_OP_RES_FAILURE;
  }
  
  if (IS_LOCKED(pHandle))
  {
    __ERROR(pHandle, I2C_MANAGER_ERR_CODE_RACE_CONDITION);
  }
  
  /* Lock module */
  __LOCK(pHandle);
  
  /* Check for state compatibility */
  if (pHandle->state != I2C_MANAGER_STATE_READY)
  {
    __ERROR(pHandle, I2C_MANAGER_ERR_CODE_INCOMPATIBLE_STATE);
  }
  
  /* Start static variables */
  pHandle->statics.reqQueue.size = 0;
  pHandle->statics.reqQueue.front = 0;
  pHandle->statics.reqQueue.rear = 0;
  pHandle->statics.unhandledIntFlag = FALSE;
  
  /* Set module's state to operating */
  pHandle->state = I2C_MANAGER_STATE_OPERATING;

  /* Unlock the module */
  __UNLOCK(pHandle);
  
  return I2C_MANAGER_OP_RES_SUCCESS;
}

/**
  * @brief  Stops operating module. 
  *
  * @param  pHandle: Pointer to module's handle.
  *
  * @retval None.
  */
I2CManager_OpRes_t I2CManager_Stop(I2CManager_Handle_t *pHandle)
{
  /* Check if the module is at error state */
  if (pHandle->state == I2C_MANAGER_STATE_ERROR)
  {
    return I2C_MANAGER_OP_RES_FAILURE;
  }
  
  if (IS_LOCKED(pHandle))
  {
    __ERROR(pHandle, I2C_MANAGER_ERR_CODE_RACE_CONDITION);
  }
  
  /* Lock module */
  __LOCK(pHandle);
  
  /* Check for state compatibility */
  if (pHandle->state != I2C_MANAGER_STATE_OPERATING)
  {
    __ERROR(pHandle, I2C_MANAGER_ERR_CODE_INCOMPATIBLE_STATE);
  }
  
  /* Set module's state to ready */
  pHandle->state = I2C_MANAGER_STATE_READY;
  
  __UNLOCK(pHandle);
  
  return I2C_MANAGER_OP_RES_SUCCESS;
}

/**
  * @brief  Resets module to uninitialized state. 
  *
  * @param  pHandle: Pointer to module's handle.
  *
  * @retval None.
  */
I2CManager_OpRes_t I2CManager_Reset(I2CManager_Handle_t *pHandle)
{
  /* Lock module */
  __LOCK(pHandle);

  /* Set module's state to uninitialized state */
  pHandle->state = I2C_MANAGER_STATE_UNINIT;
  
  __UNLOCK(pHandle);
  
  return I2C_MANAGER_OP_RES_SUCCESS;
}

/**
  * @brief  Requests I2C read/write operation from the manager module.
  *
  * @param  pHandle: Pointer to module's handle.
  * @param  pRequest: Pointer to request parameters.
  *
  * @retval None.
  */
I2CManager_OpRes_t I2CManager_Request(I2CManager_Handle_t *pHandle, 
                                      I2CManager_Request_t *pRequest)
{
  /* Check if the module is at error state */
  if (pHandle->state == I2C_MANAGER_STATE_ERROR)
  {
    return I2C_MANAGER_OP_RES_FAILURE;
  }
  
  if (IS_LOCKED(pHandle))
  {
    __ERROR(pHandle, I2C_MANAGER_ERR_CODE_RACE_CONDITION);
  }
  
  /* Lock module */
  __LOCK(pHandle);
  
  /* Check if the module's state is compatible with the operation */
  if (pHandle->state != I2C_MANAGER_STATE_OPERATING)
  {
    /* Set error flags and return */
    __ERROR(pHandle, I2C_MANAGER_ERR_CODE_INCOMPATIBLE_STATE);
  }

  /* Check if the request queue is already full */
  if (pHandle->statics.reqQueue.size >= I2C_MANAGER_REQ_QUEUE_SIZE)
  {
    /* Set error flags and return */
    __ERROR(pHandle, I2C_MANAGER_ERR_CODE_REQ_QUEUE_IS_FULL);
  }
  /* Else if there is any element in the queue */
  else if (pHandle->statics.reqQueue.size > 0)
  {
    /* Set request's status to pending */
    *(pRequest->pReqStatusFlag) = I2C_MANAGER_REQ_STATUS_PENDING;
    
    /* Enqueue request */
    enqueue(&(pHandle->statics.reqQueue), pRequest);
  }
  else
  {
    /* Enqueue request */
    enqueue(&(pHandle->statics.reqQueue), pRequest);
    
    /* Set request's status to processing */
    *(pRequest->pReqStatusFlag) = I2C_MANAGER_REQ_STATUS_PROCESSING;
    
    /* Process request */
    if (processRequest(pHandle, pRequest) != SUCCESS)
    {
      /* Set request status to error */
      *(pRequest->pReqStatusFlag) = I2C_MANAGER_REQ_STATUS_ERROR;

      /* Set error flags and return */
      __ERROR(pHandle, I2C_MANAGER_ERR_CODE_HAL_I2C_ERROR);
    }
  }
  
  /* Unlock the module */
  __UNLOCK(pHandle);

  /* If interrupt occured during process, handle it */
  if (pHandle->statics.unhandledIntFlag == TRUE)
  {
    memCBHandler(pHandle->init.hi2c, pHandle->statics.callbackType);
  }
  
  return I2C_MANAGER_OP_RES_SUCCESS;
}

/**
  * @brief  Cancels requests.
  *
  * @param  pHandle: Pointer to module's handle.
  * @param  pRequest: Pointer to request parameters.
  *
  * @retval None.
  */
I2CManager_OpRes_t I2CManager_CancelRequest(I2CManager_Handle_t *pHandle, 
                                            I2CManager_Request_t *pRequest)
{
  /* Check if the module is at error state */
  if (pHandle->state == I2C_MANAGER_STATE_ERROR)
  {
    return I2C_MANAGER_OP_RES_FAILURE;
  }
  
  if (IS_LOCKED(pHandle))
  {
    __ERROR(pHandle, I2C_MANAGER_ERR_CODE_RACE_CONDITION);
  }
  
  /* Lock module */
  __LOCK(pHandle);
  
  /* Check for state compatibility */
  if (pHandle->state != I2C_MANAGER_STATE_OPERATING)
  {
    __ERROR(pHandle, I2C_MANAGER_ERR_CODE_INCOMPATIBLE_STATE);
  }
  
  /* If the request is a pending request, cancel it */
  if (*(pRequest->pReqStatusFlag) == I2C_MANAGER_REQ_STATUS_PENDING)
  {
    *(pRequest->pReqStatusFlag) = I2C_MANAGER_REQ_STATUS_CANCELED;
  }
  
  __UNLOCK(pHandle);
  
  /* If interrupt occured during process, handle it */
  if (pHandle->statics.unhandledIntFlag == TRUE)
  {
    memCBHandler(pHandle->init.hi2c, pHandle->statics.callbackType);
  }
  
  return I2C_MANAGER_OP_RES_SUCCESS;
}

/**
  * @brief  Error callback.
  *
  * @param  pHandle: Pointer to module's handle.
  *
  * @retval None.
  */
__weak void I2CManager_ErrorCallback(I2CManager_Handle_t *pHandle)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(pHandle);
}

/**
  * @brief  Callback function which is triggered when data reception is completed.
  *
  * @param  hi2c: I2C handle.
  *
  * @retval None.
  */
void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
  memCBHandler(hi2c, RX_COMPLETED_CB);
}

/**
  * @brief  Callback function which is triggered when data transmission is completed.
  *
  * @param  hi2c: I2C handle.
  *
  * @retval None.
  */
void HAL_I2C_MemTxCpltCallback(I2C_HandleTypeDef *hi2c)
{
  memCBHandler(hi2c, TX_COMPLETED_CB);
}

/**
  * @brief  Callback function which is triggered when error occured during data 
  *         communication.
  *
  * @param  hi2c: I2C handle.
  *
  * @retval None.
  */
void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c)
{
  memCBHandler(hi2c, ERROR_CB);
}

static void memCBHandler(I2C_HandleTypeDef *hi2c, uint8_t memCBType)
{
  /* If the handle is invalid, discard the process */
  if (hi2c == NULL)
  {
    return;
  }
  
  uint8_t module_found = FALSE;
  
  I2CManager_Handle_t *pHandle;
  
  /* Parse I2C manager handle, if it's registered */
  for (uint8_t i = 0; i < NumOfModules; i++)
  {
    if (RegisteredModules[i]->init.hi2c == hi2c)
    {
      module_found = TRUE;
      
      pHandle = RegisteredModules[i];
      break;
    }
  }
  
  /* Handle is not registered, discard the process */
  if (module_found == FALSE)
  {
    return;
  }
  
  /* Check if the module is at error state */
  if (pHandle->state == I2C_MANAGER_STATE_ERROR)
  {
    return;
  }
  
  /* If the module is locked, set unhandled interrupt flag and its type */
  if (IS_LOCKED(pHandle))
  {
    pHandle->statics.callbackType = memCBType;
    pHandle->statics.unhandledIntFlag = TRUE;
    
    return;
  }
  
  /* Lock module */
  __LOCK(pHandle);
  
  /* Check for the state compatibility */
  if (pHandle->state != I2C_MANAGER_STATE_OPERATING)
  {
    __ERROR_INT(pHandle, I2C_MANAGER_ERR_CODE_INCOMPATIBLE_STATE);
  }
  
  pHandle->statics.unhandledIntFlag = FALSE;
  
  I2CManager_Request_t request;
   
  /* If the handled callback is RX or TX Completed callback */
  if ((memCBType == RX_COMPLETED_CB) || (memCBType == TX_COMPLETED_CB))
  {
    while (pHandle->statics.reqQueue.size > 0) 
    {
      /* Read the front element */
      front(&(pHandle->statics.reqQueue), &request);
       
      /* If there is a pending request at the front */
      if (*(request.pReqStatusFlag) == I2C_MANAGER_REQ_STATUS_PENDING)
      {
        /* Set request's state to processing */
        *(request.pReqStatusFlag) = I2C_MANAGER_REQ_STATUS_PROCESSING;
          
        /* Process request */           
        if (processRequest(pHandle, &request) != SUCCESS)
        {
          __ERROR_INT(pHandle, I2C_MANAGER_ERR_CODE_HAL_I2C_ERROR);
        }
          
        break;
      }

      /* Else if there was a processed request at the front */
      else if (*(request.pReqStatusFlag) == I2C_MANAGER_REQ_STATUS_PROCESSING)
      {
        /* Set request's status flag to completed */
        *(request.pReqStatusFlag) = I2C_MANAGER_REQ_STATUS_COMPLETED;
          
        /* Dequeue element */
        dequeue(&(pHandle->statics.reqQueue));
      }
      else
      {
        /* Dequeue element */
        dequeue(&(pHandle->statics.reqQueue));
      }
    }
  }
  
  if (memCBType == ERROR_CB)
  {
    /* If the request queue is empty, discard the interrupt */
    if (pHandle->statics.reqQueue.size == 0)
    {
      /* Unlock the module */
      __UNLOCK(pHandle);
      
      return;
    }
      
    /* Read the front element */
    front(&(pHandle->statics.reqQueue), &request);
      
    /* Set request's flag to error */
    *(request.pReqStatusFlag) = I2C_MANAGER_REQ_STATUS_ERROR;
      
    __ERROR_INT(pHandle, I2C_MANAGER_ERR_CODE_HAL_I2C_ERROR);
  }
  
  /* Unlock the module */
  __UNLOCK(pHandle);
  
  return;
}

/* Private functions ---------------------------------------------------------*/
static ErrorStatus processRequest(I2CManager_Handle_t *pHandle, I2CManager_Request_t *pRequest)
{
  /* Process request according to it's type */
  if (pRequest->type == I2C_MANAGER_REQ_TYPE_READ)
  {
    /* Memory read */
    if (HAL_I2C_Mem_Read_DMA(pHandle->init.hi2c, (uint16_t)pRequest->devAddr, 
                             (uint16_t)pRequest->memAddr, MEM_ADDR_SIZE, 
                             pRequest->pData, (uint8_t)pRequest->size) != HAL_OK)
    {
      return ERROR;
    }
  }
  
  else if (pRequest->type == I2C_MANAGER_REQ_TYPE_WRITE)
  {
    /* Memory write */
    if (HAL_I2C_Mem_Write_DMA(pHandle->init.hi2c, (uint16_t)pRequest->devAddr, 
                              (uint16_t)pRequest->memAddr, MEM_ADDR_SIZE, 
                              pRequest->pData, (uint8_t)pRequest->size) != HAL_OK)
    {
      return ERROR;
    }
  }
  else
  {
    return ERROR;
  }
 
  return SUCCESS;
}

static void front(I2CManager_ReqQueue_t *Q, I2CManager_Request_t *pElement)
{
  /* Read element which is at the front */
  pElement->type = Q->elements[Q->front].type;
  pElement->devAddr = Q->elements[Q->front].devAddr;
  pElement->memAddr = Q->elements[Q->front].memAddr;
  pElement->pData = Q->elements[Q->front].pData;
  pElement->pReqStatusFlag = Q->elements[Q->front].pReqStatusFlag;
  pElement->size = Q->elements[Q->front].size;
}

static void enqueue(I2CManager_ReqQueue_t *Q, I2CManager_Request_t *pElement)
{
  /* Insert the element in its rear side */ 
  Q->elements[Q->rear].type = pElement->type;
  Q->elements[Q->rear].devAddr = pElement->devAddr;
  Q->elements[Q->rear].memAddr = pElement->memAddr;
  Q->elements[Q->rear].pData = pElement->pData;
  Q->elements[Q->rear].pReqStatusFlag = pElement->pReqStatusFlag;
  Q->elements[Q->rear].size = pElement->size;
  
  Q->size++;
  Q->rear++;
    
  /* As we fill the queue in circular fashion */
  if (Q->rear == I2C_MANAGER_REQ_QUEUE_SIZE)
  {
    Q->rear = 0;
  }
}

static void dequeue(I2CManager_ReqQueue_t *Q)
{
  Q->size--;
  Q->front++;
    
  /* As we fill elements in circular fashion */
  if(Q->front == I2C_MANAGER_REQ_QUEUE_SIZE)
  {
    Q->front = 0;
  }
}