/**
  ******************************************************************************
  * @file    st_data_parser.h
  * @author  SRA Application Team
  * @brief   Header file for st_data_parser.c
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

#ifndef ST_DATA_PARSER_H
#define ST_DATA_PARSER_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include "bluenrg_conf.h"

/* Exported defines ----------------------------------------------------------*/
/* characteristic value data length */
#define ENV_DATA_LEN_SHORT         8
#define ENV_DATA_LEN_LONG         12
#define PRESS_DATA_LEN             4
#define TEMP_DATA_LEN              4
#define HUM_DATA_LEN               4
#define CO_DATA_LEN                6
#define LED_DATA_LEN               3
#define MIC_DATA_LEN               3
#define PROX_DATA_LEN              4
#define LUX_DATA_LEN               4
#define ECOMPASS_DATA_LEN          4
#define ACT_REC_DATA_LEN           3
#define CARRY_POS_REC_DATA_LEN     3
#define GESTURE_REC_DATA_LEN       3
#define MOT_INTENSITY_DATA_LEN     3
#define PEDOMETER_DATA_LEN         8
#define ACCGYRMAG_DATA_LEN        20

/* Acc. event data length */
#define ACC_EVENT_DATA_LEN_SHORT  4
#define ACC_EVENT_DATA_LEN_LONG   5
/* Quaternions data length */
#define QUATERNIONS_1_DATA_LEN  8
#define QUATERNIONS_2_DATA_LEN  14
#define QUATERNIONS_3_DATA_LEN  20
/* Proximity range */
#define HIGH_RANGE_DATA_MAX  0x7FFE
#define LOW_RANGE_DATA_MAX   0x00FE
#define OUT_OF_RANGE_VALUE   0xFFFF
#define INT16_FIRST_BIT_MASK 0x8000

/* Acceleration Events */
#define ACC_NOT_USED     0x00
#define ACC_6D_OR_TOP    0x01
#define ACC_6D_OR_LEFT   0x02
#define ACC_6D_OR_BOTTOM 0x03
#define ACC_6D_OR_RIGHT  0x04
#define ACC_6D_OR_UP     0x05
#define ACC_6D_OR_DOWN   0x06
#define ACC_TILT         0x08
#define ACC_FREE_FALL    0x10
#define ACC_SINGLE_TAP   0x20
#define ACC_DOUBLE_TAP   0x40
#define ACC_WAKE_UP      0x80

/* Exported types ------------------------------------------------------------*/
typedef struct {
  int16_t val;
  uint8_t  is_neg;
} axes_t;

typedef struct {
  axes_t x;
  axes_t y;
  axes_t z;
} axis_t;

typedef struct {
  axis_t acc;
  axis_t gyr;
  axis_t mag;
} motionData_t;

typedef struct {
  axis_t q1;
  axis_t q2;
  axis_t q3;
} quatData_t;

typedef struct {
  uint16_t type_val;
  char*    type_name;
} accEvent_t;

typedef struct {
  uint8_t type_val;
  char*   type_name;
} activityRec_t;

typedef struct {
  uint8_t type_val;
  char*   type_name;
} carryPosition_t;

typedef struct {
  uint8_t type_val;
  char*   type_name;
} gestureRec_t;

typedef struct {
  uint8_t type_val;
  char*   type_name;
} intensityDet_t;

typedef struct {
  uint32_t steps;
  uint16_t steps_min;
} pedometerInfo_t;

/* Exported variables --------------------------------------------------------*/
/* Exported function prototypes ----------------------------------------------*/
uint16_t     Get_Timestamp(uint8_t data_length, uint8_t* value);
uint32_t     Get_Pressure(uint8_t data_length, uint8_t* value);
uint16_t     Get_Humidity(uint8_t data_length, uint8_t* value);
uint16_t     Get_Temperature(uint8_t data_length, uint8_t* value, uint8_t sensor);
uint32_t     Get_CO(uint8_t data_length, uint8_t* value);
uint8_t      Get_LED_Status(uint8_t data_length, uint8_t* value);
accEvent_t   Get_Acc_Event(uint8_t data_length, uint8_t* value);
uint8_t      Get_Mic_Audio_Level(uint8_t data_length, uint8_t* value);
uint16_t     Get_Proximity(uint8_t data_length, uint8_t* value);
uint16_t     Get_Lux_Level(uint8_t data_length, uint8_t* value);
motionData_t Get_Motion_Data(uint8_t data_length, uint8_t* value);
quatData_t   Get_Quaternions_Data(uint8_t data_length, uint8_t* value);

uint16_t        Get_ECompass(uint8_t data_length, uint8_t* value);
activityRec_t   Get_Activity_Recognition(uint8_t data_length, uint8_t* value);
carryPosition_t Get_Carry_Pos_Recognition(uint8_t data_length, uint8_t* value);
gestureRec_t    Get_Gesture_Recognition(uint8_t data_length, uint8_t* value);
intensityDet_t  Get_Motion_Intensity(uint8_t data_length, uint8_t* value);
pedometerInfo_t Get_Pedometer_Info(uint8_t data_length, uint8_t* value);

#ifdef __cplusplus
}
#endif

#endif /* ST_DATA_PARSER_H */
