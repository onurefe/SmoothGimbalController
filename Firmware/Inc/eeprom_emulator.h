/**
  ******************************************************************************
  * @file    eeprom_emulator.h
  * @author  Onur Efe
  * @date    31.03.2016
  * @brief   EEPROM Emulator class interface.EEPROM Emulator is implemented on
  *          last two pages(flash sector 10-11). If the complier,
  *          this migth
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
#ifndef __EEPROM_EMULATOR_H
#define __EEPROM_EMULATOR_H

#ifdef __cplusplus
 extern "C" {
#endif
   
/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h"
#include "generic.h"

/* Exported constants --------------------------------------------------------*/
#define EEPROM_EMULATOR_INVALID_OBJ_ID          = 0xFF
   
/* Exported types ------------------------------------------------------------*/
typedef enum
{
  EEPROM_EMULATOR_OP_RES_SUCCESS                = 0x00,
  EEPROM_EMULATOR_OP_RES_INVALID_POINTER        = 0x01,
  EEPROM_EMULATOR_OP_RES_INVALID_PARAMETER      = 0x02,
  EEPROM_EMULATOR_OP_RES_MODULE_NOT_INIT        = 0x03,
  EEPROM_EMULATOR_OP_RES_OBJ_NOT_FOUND          = 0x04,
  EEPROM_EMULATOR_OP_RES_FLASH_ERASE_ERROR      = 0x05,
  EEPROM_EMULATOR_OP_RES_FLASH_WRITE_ERROR      = 0x06,
} EepromEmulator_OpRes_t;                               /* Operation results. */

/* Exported functions --------------------------------------------------------*/

/**
  * @brief  Eeprom emulator module initializer. Should be called before any other 
  *         functions at this module could be used.
  *
  * @param  None  
  *
  * @retval EepromEmulator_OpRes_t
  */
EepromEmulator_OpRes_t EepromEmulator_Init();

/**
  * @brief  Reads data object from the flash.
  *
  * @param  objectId: Data object id.
  * @param  maxLength: Maximum read length.
  * @param  pLength: Pointer to return length of the data object.
  * @param  pData: Pointer to return data.
  *
  * @retval EepromEmulator_OpRes_t.
  */
EepromEmulator_OpRes_t EepromEmulator_ReadObject(uint8_t objectId, uint8_t maxLength, 
                                                 uint8_t *pLength, uint8_t *pData);

/**
  * @brief  Writes data object to the flash memory. 
  *
  * @param  objectId: Id of the data object which is to be written.
  * @param  length: Length of the data object.
  * @param  pData: Pointer of the data.
  *
  * @retval EepromEmulator_OpRes_t
  */
EepromEmulator_OpRes_t EepromEmulator_WriteObject(uint8_t objectId, uint8_t length, 
                                                  uint8_t *pData);
   
#ifdef __cplusplus
}
#endif

#endif