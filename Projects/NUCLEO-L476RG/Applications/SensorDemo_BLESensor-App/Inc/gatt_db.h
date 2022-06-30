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

#ifndef GATT_DB_H
#define GATT_DB_H

/* Includes ------------------------------------------------------------------*/
#include "hci.h"

/* Exported defines ----------------------------------------------------------*/
#define X_OFFSET 200
#define Y_OFFSET 50
#define Z_OFFSET 1000

/**
 * @brief Number of application services
 */
#define NUMBER_OF_APPLICATION_SERVICES (2)

/**
 * @brief Define How Many quaterions you want to transmit (from 1 to 3)
 *        In this sample application use only 1
 */
#define SEND_N_QUATERNIONS 1

/* Exported typedef ----------------------------------------------------------*/
/**
 * @brief Structure containing acceleration value of each axis.
 */
typedef struct {
  int32_t AXIS_X;
  int32_t AXIS_Y;
  int32_t AXIS_Z;
} AxesRaw_t;

enum {
  ACCELERATION_SERVICE_INDEX = 0,
  ENVIRONMENTAL_SERVICE_INDEX = 1
};

/* Exported function prototypes ----------------------------------------------*/
tBleStatus Add_HWServW2ST_Service(void);
tBleStatus Add_SWServW2ST_Service(void);
#ifdef STM32L476xx
tBleStatus Add_ConsoleW2ST_Service(void);
#endif /* STM32L476xx */
void Read_Request_CB(uint16_t handle);
void Attribute_Modified_Request_CB(uint16_t Connection_Handle, uint16_t attr_handle,
                                   uint16_t Offset, uint8_t data_length, uint8_t *att_data);
tBleStatus Environmental_Update(int32_t press, int16_t temp);
tBleStatus Acc_Update(AxesRaw_t *x_axes, AxesRaw_t *g_axes, AxesRaw_t *m_axes);
tBleStatus Quat_Update(AxesRaw_t *q_axes);
#ifdef STM32L476xx
tBleStatus Term_Update(uint8_t *data, uint8_t length);
tBleStatus Term_Update_AfterRead(void);
#endif /* STM32L476xx */

#endif /* GATT_DB_H */
