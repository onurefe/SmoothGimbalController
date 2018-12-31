/**
  ******************************************************************************
  * @file    eeprom_emulator.c
  * @author  Onur Efe
  * @date    31.03.2016
  * @brief   EEPROM Emualtor class implementation.
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
#include "eeprom_emulator.h"

/* Private types -------------------------------------------------------------*/
/* Entry_t struct typedef. Size of the struct should be four bytes */
typedef enum
{
  UNINITIALIZED         = 0x00,
  INITIALIZED           = 0x01 
} State_t;

typedef struct
{
  uint8_t               id;
  uint8_t               length;
  uint16_t              data_offset;       
} Entry_t;

typedef struct
{
  const uint32_t        base_address;
  const uint32_t        entry_stack_address;
  const uint32_t        data_stack_address;
  const uint32_t        flash_sector;
  const uint8_t         page_id;
  uint16_t              header;
  uint32_t              entry_stack_pointer;
  uint32_t              data_stack_pointer;
} Page_t;

/* Private constants ---------------------------------------------------------*/
#define PAGE_SIZE                               0x00020000              /* Page size = 128KByte */
#define PAGE_HEADER_SIZE                        0x02
#define ENTRY_STACK_SIZE                        0x00004000              /* Entry stack size = 16Kbyte */
#define DATA_STACK_SIZE                         (PAGE_SIZE - (PAGE_HEADER_SIZE + ENTRY_STACK_SIZE))

#define PAGE_A_ID                               0x00
#define PAGE_A_BASE_ADDRESS                     ((uint32_t)(0x080C0000))
#define PAGE_A_SECTOR                           FLASH_SECTOR_10

#define PAGE_B_ID                               0x01
#define PAGE_B_BASE_ADDRESS                     ((uint32_t)(0x080E0000))
#define PAGE_B_SECTOR                           FLASH_SECTOR_11

#define PAGE_A_ENTRY_STACK_BASE_ADDRESS         PAGE_A_BASE_ADDRESS + PAGE_HEADER_SIZE
#define PAGE_A_DATA_STACK_BASE_ADDRESS          (uint32_t)(PAGE_A_ENTRY_STACK_BASE_ADDRESS + ENTRY_STACK_SIZE)

#define PAGE_B_ENTRY_STACK_BASE_ADDRESS         PAGE_B_BASE_ADDRESS + PAGE_HEADER_SIZE
#define PAGE_B_DATA_STACK_BASE_ADDRESS          (uint32_t)(PAGE_B_ENTRY_STACK_BASE_ADDRESS + ENTRY_STACK_SIZE)

#define PAGE_EMPTY                              ((uint16_t)0xFFFF)
#define PAGE_RECEIVING                          ((uint16_t)0xEEEE)
#define PAGE_ACTIVE                             ((uint16_t)0xAAAA)

#define NULL_OBJECT_ID                          ((uint8_t)0xFF)

/* Private functions ---------------------------------------------------------*/
static EepromEmulator_OpRes_t   toggleActivePage();
static void                     readEntry(uint32_t entryStackBaseAddr, uint32_t entryPointer, Entry_t *pEntry);
static EepromEmulator_OpRes_t   writeEntry(uint32_t entryStackBaseAddr, uint32_t entryStackPointer, Entry_t *pEntry);
static void                     readData(uint32_t dataStackBaseAddr, uint32_t offset, uint8_t *pData, uint8_t length);
static EepromEmulator_OpRes_t   writeData(uint32_t dataStackBaseAddr, uint32_t offset, uint8_t *pData, uint8_t length);
static void                     initPage(Page_t *pPage);
static EepromEmulator_OpRes_t   formatPage(Page_t *pPage, uint16_t page_header);

/* Private variables ---------------------------------------------------------*/
Page_t PageA = 
{
  .base_address = PAGE_A_BASE_ADDRESS, 
  .entry_stack_address = PAGE_A_ENTRY_STACK_BASE_ADDRESS,
  .data_stack_address = PAGE_A_DATA_STACK_BASE_ADDRESS,
  .flash_sector = PAGE_A_SECTOR,
  .page_id = PAGE_A_ID,
  .header = PAGE_EMPTY,
  .entry_stack_pointer = 0,
  .data_stack_pointer = 0
};

Page_t PageB = 
{
  .base_address = PAGE_B_BASE_ADDRESS, 
  .entry_stack_address = PAGE_B_ENTRY_STACK_BASE_ADDRESS,
  .data_stack_address = PAGE_B_DATA_STACK_BASE_ADDRESS,
  .flash_sector = PAGE_B_SECTOR,
  .page_id = PAGE_B_ID,
  .header = PAGE_EMPTY,
  .entry_stack_pointer = 0,
  .data_stack_pointer = 0
};

Page_t  *pActivePage;
State_t State = UNINITIALIZED;
uint8_t TransferBuffer[0x100];

/**
  * @brief  Eeprom emulator module initializer. Should be called before any other 
  *         functions at this module could be used.
  *
  * @param  None  
  *
  * @retval EepromEmulator_OpRes_t
  */
EepromEmulator_OpRes_t EepromEmulator_Init()
{
  EepromEmulator_OpRes_t operation_result;
  
  initPage(&PageA);
  initPage(&PageB);
  
  /* Set active page pointer and format pages */
  if (PageA.header == PAGE_ACTIVE)
  {
    /* PageB header isn't empty, format page B*/
    if (PageB.header != PAGE_EMPTY)
    {
      operation_result = formatPage(&PageB, PAGE_EMPTY);
      if (operation_result != EEPROM_EMULATOR_OP_RES_SUCCESS)
      {
        return operation_result;
      }
    }
    
    pActivePage = &PageA;
  }
  else if (PageB.header == PAGE_ACTIVE)
  {
    /* PageA header isn't empty, format page A*/
    if (PageA.header != PAGE_EMPTY)
    {
      operation_result = formatPage(&PageA, PAGE_EMPTY);
      if (operation_result != EEPROM_EMULATOR_OP_RES_SUCCESS)
      {
        return operation_result;
      }
    }
    
    pActivePage = &PageB;
  }
  else
  {
    /* If neither of the pages is active, format both of them and select page A as active page */
    operation_result = formatPage(&PageA, PAGE_ACTIVE);
    if (operation_result != EEPROM_EMULATOR_OP_RES_SUCCESS)
    {
      return operation_result;
    }
    
    operation_result = formatPage(&PageB, PAGE_EMPTY);
    if (operation_result != EEPROM_EMULATOR_OP_RES_SUCCESS)
    {
      return operation_result;
    }
    
    pActivePage = &PageA;
  }
  
  State = INITIALIZED;
  
  return EEPROM_EMULATOR_OP_RES_SUCCESS;
}

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
                                                 uint8_t *pLength, uint8_t *pData)
{
  Entry_t entry;
  uint8_t object_found = FALSE;
  
  /* Pointer validation */
  if ((pLength == NULL) || (pData == NULL))
  {
    return EEPROM_EMULATOR_OP_RES_INVALID_POINTER;
  }
  
  /* Parameter validation */
  if (objectId == NULL_OBJECT_ID)
  {
    return EEPROM_EMULATOR_OP_RES_INVALID_PARAMETER;
  }
  
  /* Check if the module is initialized */
  if (State != INITIALIZED)
  {
    return EEPROM_EMULATOR_OP_RES_MODULE_NOT_INIT;
  }
  
  /* Check if the stack is empty */
  if (pActivePage->entry_stack_pointer == 0)
  {
    return EEPROM_EMULATOR_OP_RES_OBJ_NOT_FOUND;
  }
  
  /* Scan entries in a reverse order, break when the entry at given id found */
  uint16_t entry_scan_pointer = pActivePage->entry_stack_pointer;
  do
  {
    /* Decrease entry scan pointer by one */
    entry_scan_pointer--;
    
    /* Read entry */
    readEntry(pActivePage->entry_stack_address, entry_scan_pointer, &entry);
    
    /* If the entry has been found, set object_found flag */
    if (entry.id == objectId)
    {
      object_found = TRUE;
      break;
    }
  } while (entry_scan_pointer > 0);

  if (object_found != TRUE)
  {
    return EEPROM_EMULATOR_OP_RES_OBJ_NOT_FOUND;
  }
  
  uint8_t read_len;
  
  /* Set data length */
  if (entry.length > maxLength)
  {
    read_len = maxLength;
  }
  else
  {
    read_len = entry.length;
  }
  
  *pLength = entry.length;
  readData(pActivePage->data_stack_address, entry.data_offset, pData, read_len);

  /* Return object doesn't exist */
  return EEPROM_EMULATOR_OP_RES_SUCCESS;
}

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
                                                  uint8_t *pData)
{
  EepromEmulator_OpRes_t operation_result;
  Entry_t entry;
  
  /* Pointer validation */
  if (pData == NULL)
  {
    return EEPROM_EMULATOR_OP_RES_INVALID_POINTER;
  }
  
  /* Parameter validation */
  if (objectId == NULL_OBJECT_ID)
  {
    return EEPROM_EMULATOR_OP_RES_INVALID_PARAMETER;
  }
  
  /* Check if the module is initialized */
  if (State != INITIALIZED)
  {
    return EEPROM_EMULATOR_OP_RES_MODULE_NOT_INIT;
  }

  /* Check if entry stack or data stack overflow will occur */
  if (((pActivePage->entry_stack_pointer + 1) > ((ENTRY_STACK_SIZE / 4) - 2)) || \
      ((pActivePage->data_stack_pointer + length) > (DATA_STACK_SIZE - 1)))
  {
    operation_result = toggleActivePage();
    if (operation_result != EEPROM_EMULATOR_OP_RES_SUCCESS)
    {
       return operation_result;
    }
  }
  
  /* Set entry object members */
  entry.id = objectId;
  entry.length = length;
  entry.data_offset = pActivePage->data_stack_pointer;
  
  /* Write entry to the entry stack */
  operation_result = writeEntry(pActivePage->entry_stack_address, 
                                pActivePage->entry_stack_pointer, &entry);
  if (operation_result != EEPROM_EMULATOR_OP_RES_SUCCESS)
  {
    return operation_result;
  }
  
  /* Write data to the data stack */
  operation_result = writeData(pActivePage->data_stack_address, 
                               pActivePage->data_stack_pointer, pData, length);
  if (operation_result != EEPROM_EMULATOR_OP_RES_SUCCESS)
  {
    return operation_result;
  }
  
  /* Update stack pointers */
  pActivePage->entry_stack_pointer++;
  pActivePage->data_stack_pointer += length;

  /* Return SUCCESS */
  return EEPROM_EMULATOR_OP_RES_SUCCESS;
}

/**
  * @brief  Toggles active page and moves current active page objects to the new
  *         active page.
  *
  * @param  None.   
  *
  * @retval EepromEmulator_OpRes_t.
  */
static EepromEmulator_OpRes_t toggleActivePage()
{
  Entry_t entry;
  Page_t *p_src_page;
  Page_t *p_dest_page;
  uint8_t object_found = FALSE;
  EepromEmulator_OpRes_t operation_result;
  
  /* Set destination and source page pointers */
  p_src_page = pActivePage;
  if (pActivePage->page_id == PAGE_A_ID)
  {
    p_dest_page = &PageB;
  }
  else
  {
    p_dest_page = &PageA;
  }
  
  /* Format destination page(RECEIVING) */
  operation_result = formatPage(p_dest_page, PAGE_EMPTY);
  if (operation_result != EEPROM_EMULATOR_OP_RES_SUCCESS)
  {
    return operation_result;
  }
  
  for (uint8_t scan_id = 0; scan_id < NULL_OBJECT_ID; scan_id++)
  {
    /* Scan entries in a reverse order, break when the entry at given id found */
    uint16_t entry_scan_pointer = pActivePage->entry_stack_pointer;
    
    object_found = FALSE;
    do
    {
      /* Decrease entry scan pointer by one */
      entry_scan_pointer--;
      
      /* Read entry */
      readEntry(p_src_page->entry_stack_address, entry_scan_pointer, &entry);
      
      /* If the entry has been found, set object_found flag */
      if (entry.id == scan_id)
      {
        object_found = TRUE;
        break;
      }
    } while (entry_scan_pointer > 0);
      
    if (object_found == FALSE)
    {
      continue;
    }
    else
    {
      /* Read data */
      readData(p_src_page->data_stack_address, entry.data_offset, TransferBuffer, entry.length);
     
      /* Change entry data_offset, because writing to new stack changes it */
      entry.data_offset = p_dest_page->data_stack_pointer;
      
      /* Write entry to destination page */
      operation_result = writeEntry(p_dest_page->entry_stack_address, 
                                    p_dest_page->entry_stack_pointer, &entry);
      if (operation_result != EEPROM_EMULATOR_OP_RES_SUCCESS)
      {
        return operation_result;
      }
      
      operation_result = writeData(p_dest_page->data_stack_address, 
                                   p_dest_page->data_stack_pointer,
                                   TransferBuffer, entry.length);
      if (operation_result != EEPROM_EMULATOR_OP_RES_SUCCESS)
      {
        return operation_result;
      }
      
      /* Update destination page stack pointers */
      p_dest_page->entry_stack_pointer++;
      p_dest_page->data_stack_pointer += entry.length;
    }
  }
  
  /* Format destination page(ACTIVE) */
  operation_result = formatPage(p_dest_page, PAGE_ACTIVE);
  if (operation_result != EEPROM_EMULATOR_OP_RES_SUCCESS)
  {
    return operation_result;
  }
  
  pActivePage = p_dest_page;
  
  /* Format source page(EMPTY) */
  operation_result = formatPage(p_dest_page, PAGE_EMPTY);
  if (operation_result != EEPROM_EMULATOR_OP_RES_SUCCESS)
  {
    return operation_result;
  }
  
  return EEPROM_EMULATOR_OP_RES_SUCCESS;
}

/**
  * @brief  Reads entry from the entry stack. Entry contains information about
  *         object and it's data.
  *
  * @param  entryStackAddr: Entry stack base address.
  * @param  entryStackPointer: Entry stack pointer.
  * @param  pEntry: Pointer to return read entry object.
  *
  * @retval None.
  */
static void readEntry(uint32_t entryStackAddr, uint32_t entryStackPointer, Entry_t *pEntry)
{
  uint32_t entry_addr = entryStackAddr + (entryStackPointer << 2);
  
  /* Read entry from the flash */
  pEntry->id = ((__IO Entry_t *)entry_addr)->id;
  pEntry->length = ((__IO Entry_t *)entry_addr)->length;
  pEntry->data_offset = ((__IO Entry_t *)entry_addr)->data_offset;
}

/**
  * @brief  Writes entry to the entry stack.
  *
  * @param  entryStackAddr: Entry stack base address.
  * @param  entryStackPointer: Entry stack pointer.
  * @param  pEntry: Pointer to entry object.
  *
  * @retval EepromEmulator_OpRes_t.
  */
static EepromEmulator_OpRes_t writeEntry(uint32_t entryStackAddr, uint32_t entryStackPointer, 
                                         Entry_t *pEntry)
{
  uint32_t entry_addr = entryStackAddr + (entryStackPointer << 2);
  
  HAL_FLASH_Unlock();
  
  /* Program entry into the flash */
  if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, entry_addr, 
                        ((uint16_t *)pEntry)[0]) != HAL_OK)
  {
    return EEPROM_EMULATOR_OP_RES_FLASH_WRITE_ERROR;
  }
  
  if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, (entry_addr + 2), 
                        ((uint16_t *)pEntry)[1]) != HAL_OK)
  {
    return EEPROM_EMULATOR_OP_RES_FLASH_WRITE_ERROR;
  }

  HAL_FLASH_Lock();
  
  return EEPROM_EMULATOR_OP_RES_SUCCESS;
}

/**
  * @brief  Reads data from the data stack. 
  *
  * @param  dataStackAddr: Data stack base address.
  * @param  dataStackPointer: Pointer to the data stack.
  * @param  pData: Pointer to data which is to be returned.
  * @param  length: Length of the data.
  *
  * @retval None.
  */
static void readData(uint32_t dataStackAddr, uint32_t dataStackPointer, uint8_t *pData, 
                     uint8_t length)
{
  uint32_t read_addr = dataStackAddr + dataStackPointer;
  
  /* Read data as byte array */
  for (uint8_t i = 0; i < length; i++)
  {
    pData[i] = *((__IO uint8_t *)read_addr);
    read_addr++;
  }
}

/**
  * @brief  Writes data to the data stack. 
  *
  * @param  dataStackAddr: Data stack base address.
  * @param  dataStackPointer: Pointer to the data stack.
  * @param  pData: Pointer to data which is to be written.
  * @param  length: Length of the data.
  *
  * @retval EepromEmulator_OpRes_t.
  */
static EepromEmulator_OpRes_t writeData(uint32_t dataStackAddr, uint32_t dataStackPointer, 
                                        uint8_t *pData, uint8_t length)
{
  /* Write data to the flash memory */
  uint32_t write_base_addr = dataStackAddr + dataStackPointer;
  uint8_t index = 0;
  
  HAL_FLASH_Unlock();
  
  while ((length - index) > 0)
  {
    /* Conditional write size according to number of remaining bytes */
    if ((length - index) >= 2)
    {
      if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, (write_base_addr + index), 
                            *((uint16_t *)(&pData[index]))) != HAL_OK)
      {
        return EEPROM_EMULATOR_OP_RES_FLASH_WRITE_ERROR;
      }
      
      index += 2;
    }
    else
    {
      if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_BYTE, (write_base_addr + index), 
                            pData[index]) != HAL_OK)
      {
        return EEPROM_EMULATOR_OP_RES_FLASH_WRITE_ERROR;
      }
      
      index += 1;
    }
  }
  
  HAL_FLASH_Lock();
  
  return EEPROM_EMULATOR_OP_RES_SUCCESS;
}

/**
  * @brief  Initializes page object. 
  *
  * @param  pPage: Pointer to the page object.
  *
  * @retval None. 
  */
static void initPage(Page_t *pPage)
{
  uint8_t object_id;

  /* Parse page header */
  pPage->header = *((__IO uint16_t *)pPage->base_address);
 
  /* Parse entry stack pointer */
  pPage->entry_stack_pointer = 0;
  
  /* Start scanning from the stack base address */
  for (uint32_t scan_pointer = 0; scan_pointer < (ENTRY_STACK_SIZE / 4); scan_pointer++)
  {
    /* Parse entry object */
    object_id = ((__IO Entry_t *)(pPage->entry_stack_address + (scan_pointer << 2)))->id;
    
    /* If object ID is NULL_OBJECT_ID */
    if (object_id == NULL_OBJECT_ID)
    {
      pPage->entry_stack_pointer = scan_pointer;
      break;
    }
  }
  
  Entry_t top_entry = {.id = 0, .length = 0, .data_offset = 0};

  /* Set entry object elements */
  if (pPage->entry_stack_pointer > 0)
  {
    /* Parse top entry */
    readEntry(pPage->entry_stack_address, (pPage->entry_stack_pointer - 1), &top_entry);
  }
  
  pPage->data_stack_pointer = top_entry.data_offset + top_entry.length;
}
 
/**
  * @brief  Formats given page according to it's page header.  
  *
  * @param  pPage: Pointer to the page object.  
  *
  * @retval EepromEmulator_OpRes_t; 
  */
static EepromEmulator_OpRes_t formatPage(Page_t *pPage, uint16_t page_header)
{
  switch (page_header)
  {
  case PAGE_EMPTY:
    {
      uint32_t sector_error;
      FLASH_EraseInitTypeDef erase_init_struct;
  
      /* Set erase init struct parameters */
      erase_init_struct.Banks = FLASH_BANK_1;
      erase_init_struct.NbSectors = 1;
      erase_init_struct.Sector = pPage->flash_sector;
      erase_init_struct.TypeErase = FLASH_TYPEERASE_SECTORS;
      erase_init_struct.VoltageRange = FLASH_VOLTAGE_RANGE_3;
  
      HAL_FLASH_Unlock();
      
      /* Erase page */
      if (HAL_FLASHEx_Erase(&erase_init_struct, &sector_error) != HAL_OK)
      {
        return EEPROM_EMULATOR_OP_RES_FLASH_ERASE_ERROR;
      }
      
      HAL_FLASH_Lock();
    }
    break;
    
  case PAGE_RECEIVING:
  case PAGE_ACTIVE:
  default:
    break;
  }
  
  HAL_FLASH_Unlock();
  
  /* Write the page header */
  if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, pPage->base_address, 
                        (uint64_t)page_header) != HAL_OK)
  {
    return EEPROM_EMULATOR_OP_RES_FLASH_WRITE_ERROR;
  }
  
  HAL_FLASH_Lock();
  
  pPage->entry_stack_pointer = 0;
  pPage->data_stack_pointer = 0;
  pPage->header = page_header;  
  
  return EEPROM_EMULATOR_OP_RES_SUCCESS;
}