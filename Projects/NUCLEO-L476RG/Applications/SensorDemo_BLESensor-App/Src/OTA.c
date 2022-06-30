/**
  ******************************************************************************
  * @file    OTA.c
  * @author  SRA Application Team
  * @brief   Over-the-Air Update API implementation
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "OTA.h"

#include "target_platform.h"

#ifdef STM32L476xx

/* Private typedef -----------------------------------------------------------*/
typedef struct
{
  uint32_t Version;
  uint32_t MagicNum;
  uint32_t OTAStartAdd;
  uint32_t OTADoneAdd;
  uint32_t OTAMaxSize;
  uint32_t ProgStartAdd;
} BootLoaderFeatures_t;

/* Private defines -----------------------------------------------------------*/

/* Compliant BootLoader version */
#define BL_VERSION_MAJOR 2
#define BL_VERSION_MINOR 0
#define BL_VERSION_PATCH 0

#ifndef STM32_SENSORTILEBOX

  /* Board/BlueNRG FW OTA Position */
  #define OTA_ADDRESS_START  0x08080010

  /* Board  FW OTA Magic Number Position */
  #define OTA_MAGIC_NUM_POS  0x08080000

  /* Board  FW OTA DONE Magic Number Position */
  #define OTA_MAGIC_DONE_NUM_POS  0x08080008

#else /* STM32_SENSORTILEBOX */

  /* Board/BlueNRG FW OTA Position */
  #define OTA_ADDRESS_START  0x08100010

  /* Board  FW OTA Magic Number Position */
  #define OTA_MAGIC_NUM_POS  0x08100000

  /* Board  FW OTA DONE Magic Number Position */
  #define OTA_MAGIC_DONE_NUM_POS  0x08100008

#endif /* STM32_SENSORTILEBOX */

/* Board  FW OTA Magic Number */
#define OTA_MAGIC_NUM 0xDEADBEEF

/* Uncomment the following define for enabling the PRINTF capability if it's supported */
#define OTA_ENABLE_PRINTF

#ifdef OTA_ENABLE_PRINTF
  #define OTA_PRINTF printf
#else /* OTA_ENABLE_PRINTF */
  #define OTA_PRINTF(...)
#endif /* OTA_ENABLE_PRINTF */

/* Private macros ------------------------------------------------------------*/
#define OTA_ERROR_FUNCTION() { while(1);}

/* Private variables ---------------------------------------------------------*/
static uint32_t SizeOfUpdateBlueFW = 0;
static uint32_t AspecteduwCRCValue = 0;
static uint32_t WritingAddress = OTA_ADDRESS_START;

static BootLoaderFeatures_t *BootLoaderFeatures = (BootLoaderFeatures_t *)0x08003F00;

/* Exported functions  -------------------------------------------------------*/
/**
 * @brief Function for Testing the BootLoader Compliance
 * @param None
 * @retval int8_t Return value for checking purpouse (0/1 == Ok/Error)
 */
int8_t CheckBootLoaderCompliance(void)
{
  OTA_PRINTF("Testing BootLoaderCompliance:\r\n");
  OTA_PRINTF("\tVersion  %d.%d.%d\r\n",
              (uint16_t)(BootLoaderFeatures->Version>>16),
              (uint16_t)((BootLoaderFeatures->Version>>8)&0xFF),
              (uint16_t)(BootLoaderFeatures->Version    &0xFF));

  if((( BootLoaderFeatures->Version>>16      )!=BL_VERSION_MAJOR) |
     (((BootLoaderFeatures->Version>>8 )&0xFF)!=BL_VERSION_MINOR) |
      ((BootLoaderFeatures->Version     &0xFF)!=BL_VERSION_PATCH)) {
    OTA_PRINTF("\tBL Version  Ko\r\n");
    return 0;
  } else {
    OTA_PRINTF("\tBL Version  Ok\r\n");
  }

  if(BootLoaderFeatures->MagicNum==OTA_MAGIC_NUM) {
    OTA_PRINTF("\tMagicNum    OK\r\n");
  } else {
    OTA_PRINTF("\tMagicNum    KO\r\n");
    return 0;
  }

  OTA_PRINTF("\tMaxSize =%lx\r\n", (long)BootLoaderFeatures->OTAMaxSize);

  if(BootLoaderFeatures->OTAStartAdd==(OTA_ADDRESS_START-16)) {
    OTA_PRINTF("\tOTAStartAdd OK\r\n");
  } else {
    OTA_PRINTF("\tOTAStartAdd KO\r\n");
    return 0;
  }

  if(BootLoaderFeatures->OTADoneAdd==OTA_MAGIC_DONE_NUM_POS) {
    OTA_PRINTF("\tOTADoneAdd  OK\r\n");
  } else {
    OTA_PRINTF("\tOTADoneAdd  KO\r\n");
    return 0;
  }

  return 1;
}

/**
 * @brief Function for Updating the Firmware
 * @param uint32_t *SizeOfUpdate Remaining size of the firmware image [bytes]
 * @param uint8_t *att_data attribute data
 * @param int32_t data_length length of the data
 * @param uint8_t WriteMagicNum 1/0 for writing or not the magic number
 * @retval int8_t Return value for checking purpouse (1/-1 == Ok/Error)
 */
int8_t UpdateFWBlueNRG(uint32_t *SizeOfUpdate,uint8_t * att_data, int32_t data_length,uint8_t WriteMagicNum)
{
  int8_t ReturnValue=0;
  /* Save the Packed received */

  if(data_length>(*SizeOfUpdate)){
    /* Too many bytes...Something wrong... necessity to send it again... */
    OTA_PRINTF("OTA something wrong data_length=%ld RemSizeOfUpdate=%ld....\r\nPlease Try again\r\n", (long)data_length, (long)(*SizeOfUpdate));
    ReturnValue = -1;
    /* Reset for Restarting again */
    *SizeOfUpdate=0;
  } else {
    uint64_t ValueToWrite;
    int32_t Counter;
    /* Save the received OTA packed ad save it to flash */
    /* Unlock the Flash to enable the flash control register access *************/
    HAL_FLASH_Unlock();

    for(Counter=0;Counter<data_length;Counter+=8) {
      memcpy((uint8_t*) &ValueToWrite,att_data+Counter,data_length-Counter+1);

      if(HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, WritingAddress,ValueToWrite)==HAL_OK) {
       WritingAddress+=8;
      } else {
        /* Error occurred while writing data in Flash memory.
           User can add here some code to deal with this error
           FLASH_ErrorTypeDef errorcode = HAL_FLASH_GetError(); */
        OTA_ERROR_FUNCTION();
      }
    }
    /* Reduce the remaining bytes for OTA completion */
    *SizeOfUpdate -= data_length;

    if(*SizeOfUpdate==0) {
      /* We had received the whole firmware and we have saved it in Flash */
      OTA_PRINTF("OTA Update saved\r\n");

      if(WriteMagicNum) {
        uint32_t uwCRCValue = 0;

        if(AspecteduwCRCValue) {
          /* Make the CRC integrity check */
          /* CRC handler declaration */
          CRC_HandleTypeDef   CrcHandle;

          /* Init CRC for OTA-integrity check */
          CrcHandle.Instance = CRC;
          /* The default polynomial is used */
          CrcHandle.Init.DefaultPolynomialUse    = DEFAULT_POLYNOMIAL_ENABLE;

          /* The default init value is used */
          CrcHandle.Init.DefaultInitValueUse     = DEFAULT_INIT_VALUE_ENABLE;

          /* The input data are not inverted */
          CrcHandle.Init.InputDataInversionMode  = CRC_INPUTDATA_INVERSION_NONE;

          /* The output data are not inverted */
          CrcHandle.Init.OutputDataInversionMode = CRC_OUTPUTDATA_INVERSION_DISABLE;

          /* The input data are 32-bit long words */
          CrcHandle.InputDataFormat              = CRC_INPUTDATA_FORMAT_WORDS;

          if(HAL_CRC_GetState(&CrcHandle) != HAL_CRC_STATE_RESET) {
            HAL_CRC_DeInit(&CrcHandle);
          }

          if (HAL_CRC_Init(&CrcHandle) != HAL_OK) {
            /* Initialization Error */
            OTA_ERROR_FUNCTION();
          } else {
            OTA_PRINTF("CRC  Initialized\n\r");
          }
          /* Compute the CRC */
          uwCRCValue = HAL_CRC_Calculate(&CrcHandle, (uint32_t *)OTA_ADDRESS_START, SizeOfUpdateBlueFW>>2);

          if(uwCRCValue==AspecteduwCRCValue) {
            ReturnValue=1;
            OTA_PRINTF("OTA CRC-checked\r\n");
          } else {
            OTA_PRINTF("OTA Error CRC-checking\r\n");
          }
        } else {
          ReturnValue=1;

        }
        if(ReturnValue==1) {
          /* We write the Magic number for making the OTA at the next Board reset and the size of Update*/
          WritingAddress = OTA_MAGIC_NUM_POS;
          ValueToWrite=(((uint64_t)SizeOfUpdateBlueFW)<<32)| (OTA_MAGIC_NUM);

          if(HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, WritingAddress,ValueToWrite)!=HAL_OK) {
            /* Error occurred while writing data in Flash memory.
               User can add here some code to deal with this error
               FLASH_ErrorTypeDef errorcode = HAL_FLASH_GetError(); */
            OTA_ERROR_FUNCTION();
          } else {
            WritingAddress = OTA_MAGIC_NUM_POS+8;
            /* Destination WritingAddress and HeaderSize==0 */
            ValueToWrite=((((uint64_t)(BootLoaderFeatures->ProgStartAdd))<<32)| (0x00));

            if(HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, WritingAddress,ValueToWrite)!=HAL_OK) {
              /* Error occurred while writing data in Flash memory.
                 User can add here some code to deal with this error
                 FLASH_ErrorTypeDef errorcode = HAL_FLASH_GetError(); */
              OTA_ERROR_FUNCTION();
            } else {
              OTA_PRINTF("OTA will be installed at next board reset\r\n");
            }
          }
        } else {
          ReturnValue=-1;
          if(AspecteduwCRCValue) {
            OTA_PRINTF("Wrong CRC! Computed=%lx  expected=%lx ... Try again\r\n", (long)uwCRCValue, (long)AspecteduwCRCValue);
          }
        }
      }
    }

    /* Lock the Flash to disable the flash control register access (recommended
     to protect the FLASH memory against possible unwanted operation) *********/
    HAL_FLASH_Lock();
  }
  return ReturnValue;
}

/**
 * @brief Start Function for Updating the Firmware
 * @param uint32_t SizeOfUpdate  size of the firmware image [bytes]
 * @param uint32_t uwCRCValue expected CRV value
 * @retval None
 */
void StartUpdateFWBlueNRG(uint32_t SizeOfUpdate, uint32_t uwCRCValue)
{
  FLASH_EraseInitTypeDef EraseInitStruct;
  uint32_t SectorError = 0;
  OTA_PRINTF("Start FLASH Erase\r\n");

  SizeOfUpdateBlueFW = SizeOfUpdate;
  AspecteduwCRCValue = uwCRCValue;
  WritingAddress = OTA_ADDRESS_START;

  EraseInitStruct.TypeErase   = FLASH_TYPEERASE_PAGES;
  EraseInitStruct.Banks       = GetBank(OTA_MAGIC_NUM_POS);
  EraseInitStruct.Page        = GetPage(OTA_MAGIC_NUM_POS);
  EraseInitStruct.NbPages     = (SizeOfUpdate+16+FLASH_PAGE_SIZE-1)/FLASH_PAGE_SIZE;

  /* Unlock the Flash to enable the flash control register access *************/
  HAL_FLASH_Unlock();

#ifdef STM32L4R9xx
   /* Clear OPTVERR bit set on virgin samples */
  __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_OPTVERR);
  /* Clear PEMPTY bit set (as the code is executed from Flash which is not empty) */
  if (__HAL_FLASH_GET_FLAG(FLASH_FLAG_PEMPTY) != 0) {
    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_PEMPTY);
  }
#endif /* STM32L4R9xx */

  if(HAL_FLASHEx_Erase(&EraseInitStruct, &SectorError) != HAL_OK){
    /* Error occurred while sector erase.
      User can add here some code to deal with this error.
      SectorError will contain the faulty sector and then to know the code error on this sector,
      user can call function 'HAL_FLASH_GetError()'
      FLASH_ErrorTypeDef errorcode = HAL_FLASH_GetError(); */
    OTA_ERROR_FUNCTION();
  } else {
    OTA_PRINTF("End FLASH Erase %ld Pages of 4KB\r\n", (long)EraseInitStruct.NbPages);
  }

  /* Lock the Flash to disable the flash control register access (recommended
  to protect the FLASH memory against possible unwanted operation) *********/
  HAL_FLASH_Lock();
}

#endif /* STM32L476xx */
