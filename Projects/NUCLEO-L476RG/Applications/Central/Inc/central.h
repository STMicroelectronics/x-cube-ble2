/**
  ******************************************************************************
  * @file    central.h
  * @author  SRA Application Team
  * @brief   Header file for central.c
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

#ifndef CENTRAL_H
#define CENTRAL_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include "app_bluenrg_2.h"
#include "link_layer.h"

/* Exported defines ----------------------------------------------------------*/
#define CENTRAL_MAJOR_VERSION            1
#define CENTRAL_MINOR_VERSION            1
#define CENTRAL_PATCH_VERSION            0

#define MAX_NUM_OF_DEVICES   10
#define MAX_NUM_OF_SERVICES  10
#define MAX_NUM_OF_CHARS     10
#define TYPED_ERROR_VALUE    11
#define MAX_NAME_LENGTH      32
#define MAX_STRING_LENGTH    20
#define UUID_MIN_LENGTH       2
#define UUID_MAX_LENGTH      16

#define GENERIC_ACCESS_PROFILE_NAME    "Generic Access Profile"
#define GENERIC_ATTRIBUTE_PROFILE_NAME "Generic Attribute Profile"

#define ST_HARDWARE_SERVICE_NAME "ST Custom Service (HARDWARE)"
#define ST_CONFIG_SERVICE_NAME   "ST Custom Service (CONFIGUR.)"
#define ST_SW_SENS_SERVICE_NAME  "ST Custom Service (SW SERVICE)"
#define ST_CONSOLE_SERVICE_NAME  "ST Custom Service (CONSOLE)"
#define CUSTOM_SERVICE_NAME      "Custom Service (UNKNOWN)"

#define ST_ENVIRONMENTAL_CHAR_NAME       "ST Environmental Characteristic"
#define ST_PRESSURE_CHAR_NAME            "ST Pressure Characteristic"
#define ST_HUMIDITY_CHAR_NAME            "ST Humidity Characteristic";
#define ST_TEMPERATURE_CHAR_NAME         "ST Temperature Characteristic"
#define ST_LED_CHAR_NAME                 "ST LED Characteristic"
#define ST_CONFIG_CHAR_NAME              "ST CFG Characteristic"
#define ST_ACC_EVENT_CHAR_NAME           "ST Acc. Event Characteristic"
#define ST_MIC_EVENT_CHAR_NAME           "ST MicEvt Characteristic"
#define ST_PROXIMITY_CHAR_NAME           "ST Proximity Characteristic"
#define ST_LUX_CHAR_NAME                 "ST Lux Characteristic"
#define ST_ACC_GYRO_MAG_CHAR_NAME        "ST Acc-Gyr-Mag Characteristic"
#define ST_QUATERNIONS_CHAR_NAME         "ST Quaternion Characteristic"
#define ST_ECOMPASS_CHAR_NAME            "ST ECompass Characteristic"
#define ST_ACTIVITY_REC_CHAR_NAME        "ST Activity Rec. Characteristic"
#define ST_CARRY_POSITION_REC_CHAR_NAME  "ST Carry Pos. Rec. Charact."
#define ST_GESTURE_REC_CHAR_NAME         "ST Gesture Rec. Characteristic"
#define ST_ACC_PEDO_CHAR_NAME            "ST Pedometer Characteristic"
#define ST_INTENSITY_DET_CHAR_NAME       "ST Intensity Det. Charact."
#define ST_TERM_CHAR_NAME                "ST Terminal Characteristic"
#define ST_STDERR_CHAR_NAME              "ST Std Error Characteristic"
#define CUSTOM_CHAR_NAME                 "Custom Characteristic"

/* Exported types ------------------------------------------------------------*/
typedef enum {
  INIT_STATUS,
  START_SCANNING,
  SCANNING_STARTED,
  SELECT_DEVICE,
  START_CONNECTION,
  CONNECTION_STARTED,
  CONNECTION_COMPLETE,
  SERVICE_DISCOVERY,
  SERVICE_DISCOVERY_STARTED,
  CLOSE_CONNECTION,
  DISCONNECTION_COMPLETE,
  CHARACTERISTIC_DISCOVERY,
  CHARACTERISTIC_DISCOVERY_STARTED,
  SELECT_CHARACTERISTIC,
  SELECT_ANOTHER_CHARACTERISTIC,
  UPDATE_CHARACTERISTIC,
  WRITING_CHARACTERISTIC_VALUE,
  WAITING_UPDATE_CHARACTERISTIC,
  RECEIVE_NOTIFICATIONS,
  DISABLE_NOTIFICATIONS,
  WAITING_DISABLE_NOTIFICATIONS,
  CHARACTERISTIC_PROPERTIES,
  PRINT_DEVICE_INFO
} centralStatus_t;

typedef enum {
  NO_SERVICE_TYPE,
  GENERIC_ACCESS_PROFILE_TYPE,
  GENERIC_ATTRIBUTE_PROFILE_TYPE,
  ST_HARDWARE_SERVICE_TYPE,
  ST_CONFIG_SERVICE_TYPE,
  ST_SW_SENS_SERVICE_TYPE,
  ST_CONSOLE_SERVICE_TYPE,
  CUSTOM_SERVICE_TYPE
} serviceType_t;

typedef enum {
  NO_CHARACTERISTIC_TYPE,
  ST_ENVIRONMENTAL_CHAR_TYPE,
  ST_PRESSURE_CHAR_TYPE,
  ST_HUMIDITY_CHAR_TYPE,
  ST_TEMPERATURE_CHAR_TYPE,
  ST_CO_CHAR_TYPE,
  ST_LED_CHAR_TYPE,
  ST_CONFIG_CHAR_TYPE,
  ST_ACC_EVENT_CHAR_TYPE,
  ST_MIC_EVENT_CHAR_TYPE,
  ST_PROXIMITY_CHAR_TYPE,
  ST_LUX_CHAR_TYPE,
  ST_ACC_GYRO_MAG_CHAR_TYPE,
  ST_QUATERNIONS_CHAR_TYPE,
  ST_ECOMPASS_CHAR_TYPE,
  ST_ACTIVITY_REC_CHAR_TYPE,
  ST_CARRY_POSITION_REC_CHAR_TYPE,
  ST_GESTURE_REC_CHAR_TYPE,
  ST_ACC_PEDO_CHAR_TYPE,
  ST_INTENSITY_DET_CHAR_TYPE,
  ST_TERM_CHAR_TYPE,
  ST_STDERR_CHAR_TYPE,
  CUSTOM_CHAR_TYPE
} characteristicType_t;

typedef struct {
  uint16_t             decl_handle;
  uint16_t             value_handle;
  uint8_t              name_length;
  uint8_t              name[MAX_NAME_LENGTH];
  uint8_t              uuid_length;
  uint8_t              uuid[UUID_MAX_LENGTH];
  uint8_t              prop_idx;
  uint8_t              broadcast;
  uint8_t              read;
  uint8_t              write_wo_resp;
  uint8_t              write;
  uint8_t              notify;
  uint8_t              indicate;
  uint8_t              auth_signed_write;
  characteristicType_t char_type;
} chacteristicInfo_t;

typedef struct {
  uint16_t           start_handle;
  uint16_t           end_handle;
  uint8_t            name_length;
  uint8_t            name[MAX_NAME_LENGTH];
  uint8_t            uuid_length;
  uint8_t            uuid[UUID_MAX_LENGTH];
  uint8_t            char_idx;
  uint8_t            char_num;
  serviceType_t      serv_type;
  chacteristicInfo_t char_info[MAX_NUM_OF_CHARS];
} serviceInfo_t;

typedef struct {
  uint16_t       conn_handle;
  tBDAddr        bdaddr;
  uint8_t        addr_type; //0x00 Public Device Address, 0x01 Random Device Address
  uint8_t        name_length;
  uint8_t        name[MAX_NAME_LENGTH];
  uint8_t        serv_idx;
  uint8_t        serv_num;
  serviceInfo_t* serv_info;
} devInfo_t;

typedef struct {
  uint8_t   dev_idx;
  uint8_t   connected;
  uint8_t   dev_num;
  devInfo_t dev_info[MAX_NUM_OF_DEVICES];
} savedDevices_t;

typedef struct {
  uint8_t dev_idx;
  uint8_t dev_num;
  tBDAddr bdaddr[MAX_NUM_OF_DEVICES];
} nonConnDevices_t;

/* Exported variables --------------------------------------------------------*/
/* Exported function prototypes ----------------------------------------------*/
uint8_t CentralDevice_Init        (void);
void    Init_Saved_Devices        (void);
void    Init_NonConn_Devices      (void);
void    Start_Scanning            (void);
void    Save_Found_Device         (tBDAddr addr, uint8_t* addr_type,
                                   uint8_t data_length, uint8_t* data, uint8_t dev_idx);
void    Save_NonConn_Device       (tBDAddr bdaddr);
void    Print_Device_Info         (uint8_t dev_idx);
void    Print_NonConn_Device      (tBDAddr addr, uint8_t* addr_type, uint8_t data_length,
                                   uint8_t* data_value);
void    Print_Service_Info        (uint8_t dev_idx, uint8_t serv_idx);
void    Print_Characteristic_Info (uint8_t dev_idx, uint8_t serv_idx, uint8_t char_idx);
void    Print_HRF_Value           (uint8_t data_length, uint8_t* value);
uint8_t Is_Device_Saved           (tBDAddr addr);
uint8_t Is_Device_Scanned         (tBDAddr addr);
uint8_t Get_Index                 (uint8_t console_ch);
void    Close_Connection          (void);
void    Start_Connection          (uint8_t dev_idx);
void    Discover_Services         (uint8_t dev_idx);
void    Discover_Characteristics  (uint8_t dev_idx, uint8_t serv_idx);
void    Update_Characteristic     (uint8_t dev_idx, uint8_t serv_idx, uint8_t char_idx,
                                   uint8_t prop_idx);
void    Set_Notifications         (uint8_t dev_idx, uint8_t serv_idx, uint8_t char_idx,
                                   uint8_t status);
void    APP_UserEvtRx             (void *pData);

#ifdef __cplusplus
}
#endif

#endif /* CENTRAL_H */
