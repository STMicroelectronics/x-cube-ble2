/**
  ******************************************************************************
  * @file    OTA.h
  * @author  SRA Application Team
  * @brief   Header file for OTA.c
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef OTA_H
#define OTA_H

#ifdef __cplusplus
 extern "C" {
#endif

#include <stdint.h>

#ifdef STM32L476xx

/* Exported defines ----------------------------------------------------------*/

#ifdef STM32_SENSORTILEBOX
  /* 1008Kbytes Max Program Size */
  #define OTA_MAX_PROG_SIZE (0x100000-0x4000-8)
#else /* STM32_SENSORTILEBOX */
  /* 496Kbytes Max Program Size */
  #define OTA_MAX_PROG_SIZE (0x80000-0x4000-8)
#endif /* STM32_SENSORTILEBOX */

/* Exported functions --------------------------------------------------------*/

/* API for preparing the Flash for receiving the Update. It defines also the Size of the Update and the CRC value expected */
void StartUpdateFWBlueNRG(uint32_t SizeOfUpdate,uint32_t uwCRCValue);

/* API for storing chuck of data to Flash.
 * When it has received the total number of byte defined by StartUpdateFWBlueNRG,
 * it computes the CRC value and if it matches the expected CRC value,
 * it writes the Magic Number in Flash for BootLoader */
int8_t UpdateFWBlueNRG(uint32_t *SizeOfUpdateBlueFW,uint8_t * att_data, int32_t data_length,uint8_t WriteMagicNum);

/* API for checking the BootLoader compliance */
int8_t CheckBootLoaderCompliance(void);

/* API for checking if it's the first Run after a FOTA */
int8_t CheckFirstRunAfterFOTA(void);

#endif /* STM32L476xx */

#ifdef __cplusplus
}
#endif

#endif /* OTA_H */
