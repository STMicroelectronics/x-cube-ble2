/**
  ******************************************************************************
  * @file    gatt_db.h
  * @author  SRA Application Team
  * @brief   Header file for gatt_db.c
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
#ifndef GATT_DB_H
#define GATT_DB_H

#include "hci.h"

/**
 * This sample application uses a char value length greater than 20 bytes
 * (typically used).
 * For using greater values for CHAR_VALUE_LENGTH (max 512) and
 * CLIENT_MAX_MTU_SIZE (max 247):
 * - increase both the two #define below to their max values
 * - increase both the HCI_READ_PACKET_SIZE and HCI_MAX_PAYLOAD_SIZE to 256
 *   (file bluenrg_conf.h)
 * - increase the CSTACK in the IDE project options (0xC00 is enough)
*/
#define CHAR_VALUE_LENGTH 63
#define CLIENT_MAX_MTU_SIZE 158

tBleStatus Add_Sample_Service(void);
void APP_UserEvtRx(void *pData);

extern uint16_t sampleServHandle, TXCharHandle, RXCharHandle;

#endif /* GATT_DB_H */
