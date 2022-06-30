/**
  ******************************************************************************
  * @file    app_bluenrg_2.c
  * @author  SRA Application Team
  * @brief   BlueNRG-2 initialization and applicative code
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
#include "app_bluenrg_2.h"

#include "bluenrg_conf.h"
#include "central.h"
#include "console.h"

#include "hci.h"
#include "hci_tl.h"
#include "bluenrg1_events.h"
#include "bluenrg1_gap_aci.h"

#include "stm32l4xx_nucleo.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* USER CODE BEGIN Defines */

/* USER CODE END Defines */

/* Private variables ---------------------------------------------------------*/
/**
 * @brief Properties mask
 */
static uint8_t props_mask[] = {
  0x01,
  0x02,
  0x04,
  0x08,
  0x10,
  0x20,
  0x40,
  0x80
};

static volatile uint8_t allow_console = FALSE;

extern savedDevices_t   saved_devices;
extern nonConnDevices_t non_conn_devices;
extern serviceInfo_t    serv_info[MAX_NUM_OF_SERVICES];
extern centralStatus_t  central_status;

/* Primary Service UUID expected from peripherals */
uint8_t GENERIC_ACCESS_PROFILE_UUID[]    = {0x00, 0x18};
uint8_t GENERIC_ATTRIBUTE_PROFILE_UUID[] = {0x01, 0x18};

/* ST Custom Services UUID */
uint8_t ST_HARDWARE_SERVICE_UUID[] = {0x1b,0xc5,0xd5,0xa5,0x02,0x00,0xb4,0x9a,0xe1,0x11,0x01,0x00,0x00,0x00,0x00,0x00};
uint8_t ST_CONFIG_SERVICE_UUID[]   = {0x1b,0xc5,0xd5,0xa5,0x02,0x00,0xb4,0x9a,0xe1,0x11,0x0F,0x00,0x00,0x00,0x00,0x00};
uint8_t ST_SW_SENS_SERVICE_UUID[]  = {0x1b,0xc5,0xd5,0xa5,0x02,0x00,0xb4,0x9a,0xe1,0x11,0x02,0x00,0x00,0x00,0x00,0x00};
uint8_t ST_CONSOLE_SERVICE_UUID[]  = {0x1b,0xc5,0xd5,0xa5,0x02,0x00,0xb4,0x9a,0xe1,0x11,0x0E,0x00,0x00,0x00,0x00,0x00};

/* ST Custom Characteristics UUID */
/* ENVIRONMENTAL */
uint8_t ST_ENVIRONMENTAL_CHAR_UUID[]      = {0x1b,0xc5,0xd5,0xa5,0x02,0x00,0x36,0xac,0xe1,0x11,0x01,0x00,0x00,0x00,0x1d,0x00};
/* ENVIRONMENTAL for Sensor Tile */
uint8_t ST_ENVIRONMENTAL_ST_CHAR_UUID[]   = {0x1b,0xc5,0xd5,0xa5,0x02,0x00,0x36,0xac,0xe1,0x11,0x01,0x00,0x00,0x00,0x14,0x00};
/* PRESSURE */
uint8_t ST_PRESSURE_CHAR_UUID[]           = {0x1b,0xc5,0xd5,0xa5,0x02,0x00,0x36,0xac,0xe1,0x11,0x01,0x00,0x00,0x00,0x10,0x00};
/* HUMIDITY */
uint8_t ST_HUMIDITY_CHAR_UUID[]           = {0x1b,0xc5,0xd5,0xa5,0x02,0x00,0x36,0xac,0xe1,0x11,0x01,0x00,0x00,0x00,0x08,0x00};
/* TEMPERATURE */
uint8_t ST_TEMPERATURE_CHAR_UUID[]        = {0x1b,0xc5,0xd5,0xa5,0x02,0x00,0x36,0xac,0xe1,0x11,0x01,0x00,0x00,0x00,0x04,0x00};
/* LED */
uint8_t ST_LED_CHAR_UUID[]                = {0x1b,0xc5,0xd5,0xa5,0x02,0x00,0x36,0xac,0xe1,0x11,0x01,0x00,0x00,0x00,0x00,0x20};
/* ACCELERATION EVENT */
uint8_t ST_ACC_EVENT_CHAR_UUID[]          = {0x1b,0xc5,0xd5,0xa5,0x02,0x00,0x36,0xac,0xe1,0x11,0x01,0x00,0x00,0x04,0x00,0x00};
/* MICROPHONE */
uint8_t ST_MIC_EVENT_CHAR_UUID[]          = {0x1b,0xc5,0xd5,0xa5,0x02,0x00,0x36,0xac,0xe1,0x11,0x01,0x00,0x00,0x00,0x00,0x04};
/* PROXIMITY */
uint8_t ST_PROXIMITY_CHAR_UUID[]          = {0x1b,0xc5,0xd5,0xa5,0x02,0x00,0x36,0xac,0xe1,0x11,0x01,0x00,0x00,0x00,0x00,0x02};
/* LUX */
uint8_t ST_LUX_CHAR_UUID[]                = {0x1b,0xc5,0xd5,0xa5,0x02,0x00,0x36,0xac,0xe1,0x11,0x01,0x00,0x00,0x00,0x00,0x01};
/* CONFIGURATION */
uint8_t ST_CONFIG_CHAR_UUID[]             = {0x1b,0xc5,0xd5,0xa5,0x02,0x00,0x36,0xac,0xe1,0x11,0x0F,0x00,0x02,0x00,0x00,0x00};
/* ACCELEROMETER, GYROSCOPE, MAGNETOMETER */
uint8_t ST_ACC_GYRO_MAG_CHAR_UUID[]       = {0x1b,0xc5,0xd5,0xa5,0x02,0x00,0x36,0xac,0xe1,0x11,0x01,0x00,0x00,0x00,0xE0,0x00};
/* QUATERNIONS */
uint8_t ST_QUATERNIONS_CHAR_UUID[]        = {0x1b,0xc5,0xd5,0xa5,0x02,0x00,0x36,0xac,0xe1,0x11,0x01,0x00,0x00,0x01,0x00,0x00};
/* E-COMPASS */
uint8_t ST_ECOMPASS_CHAR_UUID[]           = {0x1b,0xc5,0xd5,0xa5,0x02,0x00,0x36,0xac,0xe1,0x11,0x01,0x00,0x40,0x00,0x00,0x00};
/* Activity Recognition */
uint8_t ST_ACTIVITY_REC_CHAR_UUID[]       = {0x1b,0xc5,0xd5,0xa5,0x02,0x00,0x36,0xac,0xe1,0x11,0x01,0x00,0x10,0x00,0x00,0x00};
/* Carry Position recognition */
uint8_t ST_CARRY_POSITION_REC_CHAR_UUID[] = {0x1b,0xc5,0xd5,0xa5,0x02,0x00,0x36,0xac,0xe1,0x11,0x01,0x00,0x08,0x00,0x00,0x00};
/* Gesture Recognition */
uint8_t ST_GESTURE_REC_CHAR_UUID[]        = {0x1b,0xc5,0xd5,0xa5,0x02,0x00,0x36,0xac,0xe1,0x11,0x01,0x00,0x02,0x00,0x00,0x00};
/* Pedometer */
uint8_t ST_ACC_PEDO_CHAR_UUID[]           = {0x1b,0xc5,0xd5,0xa5,0x02,0x00,0x36,0xac,0xe1,0x11,0x01,0x00,0x01,0x00,0x00,0x00};
/* Intensity Detection */
uint8_t ST_INTENSITY_DET_CHAR_UUID[]      = {0x1b,0xc5,0xd5,0xa5,0x02,0x00,0x36,0xac,0xe1,0x11,0x01,0x00,0x20,0x00,0x00,0x00};
/* Terminal */
uint8_t ST_TERM_CHAR_UUID[]               = {0x1b,0xc5,0xd5,0xa5,0x02,0x00,0x36,0xac,0xe1,0x11,0x0E,0x00,0x01,0x00,0x00,0x00};
/* Standard Error */
uint8_t ST_STDERR_CHAR_UUID[]             = {0x1b,0xc5,0xd5,0xa5,0x02,0x00,0x36,0xac,0xe1,0x11,0x0E,0x00,0x02,0x00,0x00,0x00};

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
static void User_Init(void);
static void User_Process(void);
static void Get_Action(void);
static void Central_Process(void);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

#if PRINT_CSV_FORMAT
extern volatile uint32_t ms_counter;
/**
 * @brief  This function is a utility to print the log time
 *         in the format HH:MM:SS:MSS (ST BlueNRG GUI time format)
 * @param  None
 * @retval None
 */
void print_csv_time(void){
  uint32_t ms = HAL_GetTick();
  PRINT_CSV("%02ld:%02ld:%02ld.%03ld", (long)(ms/(60*60*1000)%24), (long)(ms/(60*1000)%60), (long)((ms/1000)%60), (long)(ms%1000));
}
#endif

void MX_BlueNRG_2_Init(void)
{
  /* USER CODE BEGIN SV */

  /* USER CODE END SV */

  /* USER CODE BEGIN BlueNRG_2_Init_PreTreatment */

  /* USER CODE END BlueNRG_2_Init_PreTreatment */

  /* Initialize the peripherals and the BLE Stack */
  uint8_t ret;

  User_Init();

  BSP_LED_On(LED2);

  hci_init(APP_UserEvtRx, NULL);

  PRINT_DBG("BlueNRG-2 Central Device\r\n");

  /* Initialize the Central Device */
  ret = CentralDevice_Init();
  if (ret != BLE_STATUS_SUCCESS)
  {
    PRINT_DBG("CentralDevice_Init()--> Failed 0x%02x\r\n", ret);
    message("/* **************** Init failed!!! ***************** */\r\n\n");
    while(1);
  }

  PRINT_DBG("\nBLE Stack Initialized & Central Device Configured\r\n");

  allow_console = TRUE;
  BSP_LED_Off(LED2);

  /* USER CODE BEGIN BlueNRG_2_Init_PostTreatment */

  /* USER CODE END BlueNRG_2_Init_PostTreatment */
}

/*
 * BlueNRG-2 background task
 */
void MX_BlueNRG_2_Process(void)
{
  /* USER CODE BEGIN BlueNRG_2_Process_PreTreatment */

  /* USER CODE END BlueNRG_2_Process_PreTreatment */

  hci_user_evt_proc();
  User_Process();

  /* USER CODE BEGIN BlueNRG_2_Process_PostTreatment */

  /* USER CODE END BlueNRG_2_Process_PostTreatment */
}

/**
 * @brief  Initialize User process.
 *
 * @param  None
 * @retval None
 */
static void User_Init(void)
{
  BSP_LED_Init(LED2);
  BSP_COM_Init(COM1);
}

/**
 * @brief  User Process
 *
 * @param  None
 * @retval None
 */
static void User_Process(void)
{
  if (allow_console) {
    Main_Menu();
    Get_Action();
  }
  Central_Process();
}

/**
 * @brief  Ask the User an action (scan the network, connect to a device,
 *         ...)
 * @param  None
 * @retval None
 */
void Get_Action(void)
{
  uint8_t console_ch[10] = {0,0,0,0,0,0,0,0,0,0};
  uint8_t i = 0;
  uint8_t dev_idx = saved_devices.dev_idx;
  uint8_t serv_idx;
  uint8_t char_idx;
  uint8_t prop_idx;

  switch (central_status)
  {
  case (SELECT_CHARACTERISTIC):
  case (SELECT_ANOTHER_CHARACTERISTIC):
    while (i < 5) {
      console_ch[i] = Uart_Receive_Char();
      if ((console_ch[0] == SCAN_CH_UPPER) ||
          (console_ch[0] == SCAN_CH_LOWER))
      {
        console_ch[0] = NOT_ALLOWED_CH; /* set to not allowed character */
      }
      if ((i == 0) &&
          ((console_ch[i] == CLOSE_CONNECTION_CH_UPPER)  ||
           (console_ch[i] == CLOSE_CONNECTION_CH_LOWER)  ||
           (console_ch[i] == PRINT_DEVICE_INFO_CH_UPPER) ||
           (console_ch[i] == PRINT_DEVICE_INFO_CH_LOWER)))
      {
        break;
      }
      i++;
    }
    printf("\r\n\n");
    break;
  case (RECEIVE_NOTIFICATIONS):
    console_ch[0] = Uart_Receive_Char_Timeout(VERY_SHORT_TIMEOUT);
    break;
  default:
    console_ch[0] = Uart_Receive_Char();
    printf("\r\n\n");
    break;
  }

  allow_console = FALSE;

  switch (console_ch[0])
  {
  case SCAN_CH_UPPER:
  case SCAN_CH_LOWER:
    if (saved_devices.connected) {
      printf("\r To re-scan the network, close the connection!\n");
      allow_console = TRUE;
    }
    else {
      Init_Saved_Devices();
      Init_NonConn_Devices();
      central_status = START_SCANNING;
    }
    break;
  case CLOSE_CONNECTION_CH_UPPER:
  case CLOSE_CONNECTION_CH_LOWER:
    if (saved_devices.connected) {
      Close_Connection();
      central_status = CLOSE_CONNECTION;
    }
    else {
      PRINT_DBG("\r No device connected!\n");
      allow_console = TRUE;
    }
    break;
  case PRINT_DEVICE_INFO_CH_UPPER:
  case PRINT_DEVICE_INFO_CH_LOWER:
    if (saved_devices.connected) {
      central_status = PRINT_DEVICE_INFO;
    }
    else {
      PRINT_DBG("\r No device connected!\n");
      allow_console = TRUE;
    }
    break;
  default:
    switch (central_status)
    {
    case (SELECT_DEVICE):
      saved_devices.dev_idx = Get_Index(console_ch[0]);
      PRINT_DBG("device_index %d\r\n", saved_devices.dev_idx);
      if (saved_devices.dev_idx != TYPED_ERROR_VALUE)
      {
        central_status = START_CONNECTION;
      }
      else {
        allow_console = TRUE;
      }
      break;
    case (SELECT_CHARACTERISTIC):
    case (SELECT_ANOTHER_CHARACTERISTIC):
      dev_idx = saved_devices.dev_idx;
      serv_idx = Get_Index(console_ch[0]);
      char_idx = Get_Index(console_ch[2]);
      prop_idx = Get_Index(console_ch[4]);

      if ((serv_idx == TYPED_ERROR_VALUE) || (console_ch[1] != '.') ||
          (char_idx == TYPED_ERROR_VALUE) || (console_ch[3] != '.') ||
          (prop_idx == TYPED_ERROR_VALUE))
      {
        printf("\r Undefined input!\r\n");
        central_status = SELECT_ANOTHER_CHARACTERISTIC;
      }
      else
      {
        saved_devices.dev_info[dev_idx].serv_idx = serv_idx;
        saved_devices.dev_info[dev_idx].serv_info[serv_idx].char_idx = char_idx;
        saved_devices.dev_info[dev_idx].serv_info[serv_idx].char_info[char_idx].prop_idx = prop_idx;

        PRINT_DBG(" SELECT_CHARACTERISTIC dev_idx %d, serv_idx %d, char_idx %d, prop_idx %d\n",
                  saved_devices.dev_idx,
                  saved_devices.dev_info[dev_idx].serv_idx,
                  saved_devices.dev_info[dev_idx].serv_info[serv_idx].char_idx,
                  saved_devices.dev_info[dev_idx].serv_info[serv_idx].char_info[char_idx].prop_idx);

        central_status = UPDATE_CHARACTERISTIC;
      }
      break;
    case RECEIVE_NOTIFICATIONS:
      if ((console_ch[0] == DISABLE_NOTIFICATIONS_CH_UPPER) ||
          (console_ch[0] == DISABLE_NOTIFICATIONS_CH_LOWER))
      {
        central_status = DISABLE_NOTIFICATIONS;
      }
      break;
    default:
      printf("\r\n Type your choice..... ");
      allow_console = TRUE;
      break;
    }
    break;
  }

  return;
}

/**
 * @brief  Manage all the BlueNRG actions (scanning, connection, services/
 *         characteristics discovery, ...) a Central device can run
 * @param  None
 * @retval None
 */
void Central_Process(void)
{
  uint8_t dev_idx  = saved_devices.dev_idx;
  uint8_t serv_idx = saved_devices.dev_info[dev_idx].serv_idx;
  uint8_t char_idx;
  uint8_t prop_idx;

  switch (central_status)
  {
  case (START_SCANNING):
    printf("\n -------------------- START_SCANNING --------------------\r\n");
    central_status = SCANNING_STARTED;
    Start_Scanning();
    break;
  case (START_CONNECTION):
    printf("\n ------------------- START_CONNECTION -------------------\r\n");
    central_status = CONNECTION_STARTED;
    Start_Connection(dev_idx);
    break;
  case (CONNECTION_COMPLETE):
    PRINT_DBG("\n CONNECTION_COMPLETE\n");
    central_status = SERVICE_DISCOVERY;
    break;
  case (SERVICE_DISCOVERY):
    PRINT_DBG("\n SERVICE_DISCOVERY\n");
    central_status = SERVICE_DISCOVERY_STARTED;
    Discover_Services(dev_idx);
    break;
  case (CHARACTERISTIC_DISCOVERY):
    Print_Service_Info(dev_idx, serv_idx);
    PRINT_DBG("\n    CHARACTERISTIC_DISCOVERY\n");
    central_status = CHARACTERISTIC_DISCOVERY_STARTED;
    Discover_Characteristics(dev_idx, serv_idx);
    break;
  case (DISCONNECTION_COMPLETE):
    printf("\n ---------------- DISCONNECTION_COMPLETE ----------------\r\n");
    central_status = INIT_STATUS;
    allow_console = TRUE;
    break;
  case (UPDATE_CHARACTERISTIC):
    PRINT_DBG("\n    UPDATE_CHARACTERISTIC\n");
    central_status = WAITING_UPDATE_CHARACTERISTIC;
    char_idx = saved_devices.dev_info[dev_idx].serv_info[serv_idx].char_idx;
    prop_idx = saved_devices.dev_info[dev_idx].serv_info[serv_idx].char_info[char_idx].prop_idx;
    Update_Characteristic(dev_idx, serv_idx, char_idx, prop_idx);
    break;
  case (SELECT_CHARACTERISTIC):
  case (SELECT_ANOTHER_CHARACTERISTIC):
    allow_console = TRUE;
    break;
  case (RECEIVE_NOTIFICATIONS):
    allow_console = TRUE;
    break;
  case (DISABLE_NOTIFICATIONS):
    char_idx = saved_devices.dev_info[dev_idx].serv_info[serv_idx].char_idx;
    Set_Notifications(dev_idx, serv_idx, char_idx, 0x00);
    central_status = WAITING_DISABLE_NOTIFICATIONS;
    break;
  case (PRINT_DEVICE_INFO):
    Print_Device_Info(dev_idx);
    printf("\r\n");
    for (uint8_t i=0; i<saved_devices.dev_info[dev_idx].serv_num; i++) {
      Print_Service_Info(dev_idx, i);
      printf("\r\n");
      for (uint8_t j=0; j<saved_devices.dev_info[dev_idx].serv_info[i].char_num; j++) {
        Print_Characteristic_Info(dev_idx, i, j);
        printf("\r\n");
      }
      printf("\r\n");
    }
    printf(" --------------------------------------------------------\r\n");
    central_status = SELECT_CHARACTERISTIC;
    break;
  case (WAITING_UPDATE_CHARACTERISTIC):
  case (WAITING_DISABLE_NOTIFICATIONS):
  case (SCANNING_STARTED):
  case (SELECT_DEVICE):
  case (CONNECTION_STARTED):
  case (SERVICE_DISCOVERY_STARTED):
  default:
    // PRINT_DBG("Central_Process: default (DO NOTHING)\n");
    break;
  }
}

/* ***************** BlueNRG-1 Stack Callbacks ********************************/
/*******************************************************************************
 * Function Name  : aci_gap_proc_complete_event.
 * Description    : This event indicates the end of a GAP procedure.
 * Input          : See file bluenrg1_events.h
 * Output         : See file bluenrg1_events.h
 * Return         : See file bluenrg1_events.h
 *******************************************************************************/
void aci_gap_proc_complete_event(uint8_t Procedure_Code,
                                 uint8_t Status,
                                 uint8_t Data_Length,
                                 uint8_t Data[])
{
  if (Procedure_Code == GAP_GENERAL_DISCOVERY_PROC)
  {
    PRINT_DBG("\n SCANNING_STOPPED\r\n");
    printf("\n Number of connectable devices: %d\r\n", saved_devices.dev_num);
    printf(" --------------------------------------------------------\r\n\n");

    if (saved_devices.dev_num > 0) {
      central_status = SELECT_DEVICE;
    }
    else {
      central_status = INIT_STATUS;
    }

    allow_console = TRUE;
  }
}

/*******************************************************************************
 * Function Name  : aci_gatt_proc_complete_event.
 * Description    : This event indicates the end of a GATT procedure.
 * Input          : See file bluenrg1_events.h
 * Output         : See file bluenrg1_events.h
 * Return         : See file bluenrg1_events.h
 *******************************************************************************/
void aci_gatt_proc_complete_event(uint16_t Connection_Handle,
                                  uint8_t Error_Code)
{
  uint8_t dev_idx  = saved_devices.dev_idx;

  if (Error_Code != BLE_STATUS_SUCCESS) {
    printf(" GATT Procedure completed with error 0x%02x (0x%04x)\n",
            Error_Code, Connection_Handle);
  }

  switch(central_status)
  {
  case SERVICE_DISCOVERY_STARTED:
    PRINT_DBG("\n SERVICE_DISCOVERY_COMPLETE\r\n\n");
    printf(" Number of services: %d\r\n",
           saved_devices.dev_info[dev_idx].serv_num);
    PRINT_DBG(" --------------------------------------------------------\r\n\n");
    printf("\r\n\n");
    saved_devices.dev_info[dev_idx].serv_idx = 0;
    central_status = CHARACTERISTIC_DISCOVERY;
    break;
  case CHARACTERISTIC_DISCOVERY_STARTED:
    if (saved_devices.dev_info[dev_idx].serv_idx == saved_devices.dev_info[dev_idx].serv_num-1) {
      PRINT_DBG("\n CHARACTERISTIC_DISCOVERY_COMPLETE\r\n");
      printf("\n --------------------------------------------------------\r\n\n");
      central_status = SELECT_CHARACTERISTIC;
      allow_console = TRUE;
    }
    else {
      PRINT_DBG("    Number of characteristics for service %d: %d",
                saved_devices.dev_info[dev_idx].serv_idx,
                saved_devices.dev_info[dev_idx].serv_info[saved_devices.dev_info[dev_idx].serv_idx].char_num);
      printf("\r\n\n");
      saved_devices.dev_info[saved_devices.dev_idx].serv_idx++;
      central_status = CHARACTERISTIC_DISCOVERY;
    }
    break;
  case WAITING_UPDATE_CHARACTERISTIC:
    if (Error_Code != BLE_STATUS_SUCCESS) {
      central_status = SELECT_ANOTHER_CHARACTERISTIC;
    }
    else {
      central_status = RECEIVE_NOTIFICATIONS;
    }
    break;
  case WRITING_CHARACTERISTIC_VALUE:
  case WAITING_DISABLE_NOTIFICATIONS:
    allow_console = TRUE;
    central_status = SELECT_ANOTHER_CHARACTERISTIC;
    break;
  default:
    break;
  }

}

/*******************************************************************************
 * Function Name  : hci_le_connection_complete_event.
 * Description    : This event indicates the end of a connection procedure.
 * Input          : See file bluenrg1_events.h
 * Output         : See file bluenrg1_events.h
 * Return         : See file bluenrg1_events.h
 *******************************************************************************/
void hci_le_connection_complete_event(uint8_t Status,
                                      uint16_t Connection_Handle,
                                      uint8_t Role,
                                      uint8_t Peer_Address_Type,
                                      uint8_t Peer_Address[6],
                                      uint16_t Conn_Interval,
                                      uint16_t Conn_Latency,
                                      uint16_t Supervision_Timeout,
                                      uint8_t Master_Clock_Accuracy)
{
  saved_devices.connected = TRUE;
  saved_devices.dev_info[saved_devices.dev_idx].conn_handle = Connection_Handle;
  saved_devices.dev_info[saved_devices.dev_idx].serv_info = serv_info;

  printf("\n Device %d connected \r\n", saved_devices.dev_idx);
  printf(" - address ");
  printf("%02x:%02x:%02x:%02x:%02x:%02x\r\n",
         saved_devices.dev_info[saved_devices.dev_idx].bdaddr[5], saved_devices.dev_info[saved_devices.dev_idx].bdaddr[4],
         saved_devices.dev_info[saved_devices.dev_idx].bdaddr[3], saved_devices.dev_info[saved_devices.dev_idx].bdaddr[2],
         saved_devices.dev_info[saved_devices.dev_idx].bdaddr[1], saved_devices.dev_info[saved_devices.dev_idx].bdaddr[0]);
  printf(" - connection handle 0x%04x\n", saved_devices.dev_info[saved_devices.dev_idx].conn_handle);

  central_status = CONNECTION_COMPLETE;

  //PRINT_DBG("\n CONNECTION_COMPLETE\r\n");
  //PRINT_DBG(" --------------------------------------------------------\r\n\n");

} /* hci_le_connection_complete_event() */

/*******************************************************************************
 * Function Name  : hci_disconnection_complete_event.
 * Description    : This event indicates the end of a disconnection procedure.
 * Input          : See file bluenrg1_events.h
 * Output         : See file bluenrg1_events.h
 * Return         : See file bluenrg1_events.h
 *******************************************************************************/
void hci_disconnection_complete_event(uint8_t Status,
                                      uint16_t Connection_Handle,
                                      uint8_t Reason)
{
  uint8_t dev_idx = saved_devices.dev_idx;

  if (Status != BLE_STATUS_SUCCESS) {
    printf("hci_disconnection_complete_event() error code: 0x%02x\n", Status);
  }
  /* print a warning message only if the disconnection is not caused neither by
   * the local host nor by the remote user */
  if ((Reason != BLE_ERROR_TERMINATED_LOCAL_HOST) &&
      (Reason != BLE_ERROR_TERMINATED_REMOTE_USER)) {
    PRINT_DBG("\n hci_disconnection_complete_event() unexpected reason: 0x%02x\n", Reason);
  }
  if (Connection_Handle != saved_devices.dev_info[dev_idx].conn_handle) {
    PRINT_DBG("\n hci_disconnection_complete_event() unexpected conn handle: 0x%04x\n",
           Connection_Handle);
  }

  saved_devices.connected = FALSE;
  /**
   * In case the disconnection occurs when notifications are enabled, we need
   * to disable the console. It is then enabled in the central process when
   * passing from DISCONNECTION_COMPLETE to INIT_STATUS.
   */
  allow_console = FALSE;
  central_status = DISCONNECTION_COMPLETE;

  PRINT_DBG("CONNECTION_TERMINATED\n");

} /* hci_disconnection_complete_event() */

/*******************************************************************************
 * Function Name  : hci_le_advertising_report_event.
 * Description    : An advertising report is received.
 * Input          : See file bluenrg1_events.h
 * Output         : See file bluenrg1_events.h
 * Return         : See file bluenrg1_events.h
 *******************************************************************************/
void hci_le_advertising_report_event(uint8_t Num_Reports,
                                     Advertising_Report_t Advertising_Report[])
{
  /* Advertising_Report contains all the expected parameters */
  uint8_t evt_type    = Advertising_Report[0].Event_Type;
  uint8_t data_length = Advertising_Report[0].Length_Data;
  uint8_t bdaddr_type = Advertising_Report[0].Address_Type;
  uint8_t bdaddr[6];

  uint8_t index = saved_devices.dev_idx;

  BLUENRG_memcpy(bdaddr, Advertising_Report[0].Address, 6);
  PRINT_DBG("Advert. device %02x:%02x:%02x:%02x:%02x:%02x (evt_type 0x%02x, data_len %d)\n",
            Advertising_Report[0].Address[5], Advertising_Report[0].Address[4],
            Advertising_Report[0].Address[3], Advertising_Report[0].Address[2],
            Advertising_Report[0].Address[1], Advertising_Report[0].Address[0],
            evt_type, data_length);
  PRINT_DBG("Advert. data ");
  for (uint8_t i=0; i<data_length; i++) {
    PRINT_DBG("%02x ", Advertising_Report[0].Data[i]);
  }
  PRINT_DBG("\n");
  PRINT_DBG("Current device %02x:%02x:%02x:%02x:%02x:%02x \n",
            bdaddr[5], bdaddr[4], bdaddr[3], bdaddr[2], bdaddr[1], bdaddr[0]);

  /* save current device found */
  if (evt_type == ADV_IND)
  {
    if ((saved_devices.dev_idx < MAX_NUM_OF_DEVICES) && (Is_Device_Saved(bdaddr) == FALSE))
    {
      Save_Found_Device(bdaddr, &bdaddr_type, data_length, Advertising_Report[0].Data,
                        index);
      Print_Device_Info(index);

      saved_devices.dev_num++;
      saved_devices.dev_idx++;
    }
  }
  else if ((evt_type == ADV_NONCONN_IND) || (evt_type == ADV_SCAN_IND))
  {
    if ((non_conn_devices.dev_idx < MAX_NUM_OF_DEVICES) && (Is_Device_Scanned(bdaddr) == FALSE))
    {
      Save_NonConn_Device(bdaddr);
      Print_NonConn_Device(bdaddr, &bdaddr_type, data_length, Advertising_Report[0].Data);
    }
  }

} /* hci_le_advertising_report_event() */

/*******************************************************************************
 * Function Name  : aci_att_read_by_group_type_resp_event.
 * Description    : A response event is received.
 * Input          : See file bluenrg1_events.h
 * Output         : See file bluenrg1_events.h
 * Return         : See file bluenrg1_events.h
 *******************************************************************************/
void aci_att_read_by_group_type_resp_event(uint16_t Connection_Handle,
                                           uint8_t  Attribute_Data_Length,
                                           uint8_t  Data_Length,
                                           uint8_t  Attribute_Data_List[])
{
  uint8_t  *uuid, *uuid_length, *name, *name_length;
  serviceType_t *serv_type;
  uint8_t  i, offset, num_attr;
  uint8_t  dev_idx = saved_devices.dev_idx;
  uint8_t  serv_idx;
  uint16_t *start_handle, *end_handle;
  char const* serv_name;

  num_attr = (Data_Length / Attribute_Data_Length);
  offset = 0;

  for (i=0; i<num_attr; i++) {
    serv_idx     = saved_devices.dev_info[dev_idx].serv_idx;
    serv_type    = &(saved_devices.dev_info[dev_idx].serv_info[serv_idx].serv_type);
    uuid         = saved_devices.dev_info[dev_idx].serv_info[serv_idx].uuid;
    uuid_length  = &(saved_devices.dev_info[dev_idx].serv_info[serv_idx].uuid_length);
    name         = saved_devices.dev_info[dev_idx].serv_info[serv_idx].name;
    name_length  = &(saved_devices.dev_info[dev_idx].serv_info[serv_idx].name_length);
    start_handle = &(saved_devices.dev_info[dev_idx].serv_info[serv_idx].start_handle);
    end_handle   = &(saved_devices.dev_info[dev_idx].serv_info[serv_idx].end_handle);

    *start_handle = Attribute_Data_List[offset];
    *end_handle   = Attribute_Data_List[offset+2];

    printf("\r\n");
    PRINT_DBG(" aci_att_read_by_group_type_resp_event\r\n");
    PRINT_DBG(" connection handle 0x%04x\r\n", Connection_Handle);
    PRINT_DBG(" data length %d\r\n", Data_Length);
    PRINT_DBG(" attribute data length %d\r\n", Attribute_Data_Length);

    *serv_type = CUSTOM_SERVICE_TYPE;
    serv_name = CUSTOM_SERVICE_NAME;

    //UUID Type
    if (Attribute_Data_Length == 6) {
      PRINT_DBG("UUID_TYPE_16\n");
      uuid[0] = Attribute_Data_List[offset+4];
      uuid[1] = Attribute_Data_List[offset+5];
      *uuid_length = UUID_MIN_LENGTH;
      if (BLUENRG_memcmp(uuid, GENERIC_ACCESS_PROFILE_UUID, (Attribute_Data_Length-4)) == 0) {
        *serv_type = GENERIC_ACCESS_PROFILE_TYPE;
        serv_name = GENERIC_ACCESS_PROFILE_NAME;
      }
      else if (BLUENRG_memcmp (uuid, GENERIC_ATTRIBUTE_PROFILE_UUID, (Attribute_Data_Length-4)) == 0) {
        *serv_type = GENERIC_ATTRIBUTE_PROFILE_TYPE;
        serv_name = GENERIC_ATTRIBUTE_PROFILE_NAME;
      }
    }
    else {
      PRINT_DBG("UUID_TYPE_128\n");
      *uuid_length = UUID_MAX_LENGTH;
      BLUENRG_memcpy(uuid, Attribute_Data_List+offset+4, *uuid_length);
      if (BLUENRG_memcmp (uuid, ST_HARDWARE_SERVICE_UUID, (Attribute_Data_Length-4)) == 0) {
        *serv_type = ST_HARDWARE_SERVICE_TYPE;
        serv_name = ST_HARDWARE_SERVICE_NAME;
      }
      else if (BLUENRG_memcmp (uuid, ST_CONFIG_SERVICE_UUID, (Attribute_Data_Length-4)) == 0) {
        *serv_type = ST_CONFIG_SERVICE_TYPE;
        serv_name = ST_CONFIG_SERVICE_NAME;
      }
      else if (BLUENRG_memcmp (uuid, ST_SW_SENS_SERVICE_UUID, (Attribute_Data_Length-4)) == 0) {
        *serv_type = ST_SW_SENS_SERVICE_TYPE;
        serv_name = ST_SW_SENS_SERVICE_NAME;
      }
      else if (BLUENRG_memcmp (uuid, ST_CONSOLE_SERVICE_UUID, (Attribute_Data_Length-4)) == 0) {
        *serv_type = ST_CONSOLE_SERVICE_TYPE;
        serv_name = ST_CONSOLE_SERVICE_NAME;
      }
    }

    *name_length = (strlen(serv_name) > MAX_NAME_LENGTH) ? MAX_NAME_LENGTH : strlen(serv_name);
    BLUENRG_memcpy(name, serv_name, *name_length);

    saved_devices.dev_info[dev_idx].serv_idx++;
    offset += Attribute_Data_Length;
  }

  saved_devices.dev_info[dev_idx].serv_num += num_attr;
}

/*******************************************************************************
 * Function Name  : aci_att_read_by_type_resp_event.
 * Description    : A response event is received.
 * Input          : See file bluenrg1_events.h
 * Output         : See file bluenrg1_events.h
 * Return         : See file bluenrg1_events.h
 *******************************************************************************/
void aci_att_read_by_type_resp_event(uint16_t Connection_Handle,
                                     uint8_t Handle_Value_Pair_Length,
                                     uint8_t Data_Length,
                                     uint8_t Handle_Value_Pair_Data[])
{
  uint8_t  *uuid, *uuid_length, *name, *name_length;
  uint16_t *decl_handle, *value_handle;
  uint8_t  *broadcast, *read, *write_wo_resp, *write, *notify, *indicate, *auth_signed_write;
  uint8_t  char_idx;
  characteristicType_t *char_type;
  char const* char_name;
  uint8_t  offset = 0;

  uint8_t dev_idx  = saved_devices.dev_idx;
  uint8_t serv_idx = saved_devices.dev_info[dev_idx].serv_idx;
  char_idx = saved_devices.dev_info[dev_idx].serv_info[serv_idx].char_idx;

  printf("\n");
  PRINT_DBG(" aci_att_read_by_type_resp_event\r\n");
  PRINT_DBG(" connection handle 0x%04x\r\n", Connection_Handle);
  PRINT_DBG(" data length %d\r\n", Data_Length);
  PRINT_DBG(" handle value pair length %d\r\n", Handle_Value_Pair_Length);

  char_type    = &(saved_devices.dev_info[dev_idx].serv_info[serv_idx].char_info[char_idx].char_type);
  uuid         = saved_devices.dev_info[dev_idx].serv_info[serv_idx].char_info[char_idx].uuid;
  uuid_length  = &(saved_devices.dev_info[dev_idx].serv_info[serv_idx].char_info[char_idx].uuid_length);
  name         = saved_devices.dev_info[dev_idx].serv_info[serv_idx].char_info[char_idx].name;
  name_length  = &(saved_devices.dev_info[dev_idx].serv_info[serv_idx].char_info[char_idx].name_length);
  decl_handle  = &(saved_devices.dev_info[dev_idx].serv_info[serv_idx].char_info[char_idx].decl_handle);
  value_handle = &(saved_devices.dev_info[dev_idx].serv_info[serv_idx].char_info[char_idx].value_handle);
  broadcast         = &(saved_devices.dev_info[dev_idx].serv_info[serv_idx].char_info[char_idx].broadcast);
  read              = &(saved_devices.dev_info[dev_idx].serv_info[serv_idx].char_info[char_idx].read);
  write_wo_resp     = &(saved_devices.dev_info[dev_idx].serv_info[serv_idx].char_info[char_idx].write_wo_resp);
  write             = &(saved_devices.dev_info[dev_idx].serv_info[serv_idx].char_info[char_idx].write);
  notify            = &(saved_devices.dev_info[dev_idx].serv_info[serv_idx].char_info[char_idx].notify);
  indicate          = &(saved_devices.dev_info[dev_idx].serv_info[serv_idx].char_info[char_idx].indicate);
  auth_signed_write = &(saved_devices.dev_info[dev_idx].serv_info[serv_idx].char_info[char_idx].auth_signed_write);

  // UUID Type
  if (Handle_Value_Pair_Length == 7) {
    PRINT_DBG("Char UUID_TYPE_16\n");
    *uuid_length = UUID_MIN_LENGTH;
    uuid[0] = Handle_Value_Pair_Data[offset+5];
    uuid[1] = Handle_Value_Pair_Data[offset+6];
    PRINT_DBG("C UUID-%02x%02x (len %d)\n", uuid[0], uuid[1], *uuid_length);
  } else {
    PRINT_DBG("Char UUID_TYPE_128\n\r");
    *uuid_length = UUID_MAX_LENGTH;
    BLUENRG_memcpy(uuid, Handle_Value_Pair_Data+offset+5, *uuid_length);
    PRINT_DBG("C UUID-");
    for (uint8_t j = 0; j < *uuid_length; j++) {
      PRINT_DBG("%02x", uuid[j]);
    }
    PRINT_DBG(" (len %d)\n", *uuid_length);
  }

  /**
  * Handles
  */
  *decl_handle  = Handle_Value_Pair_Data[offset];
  *value_handle = Handle_Value_Pair_Data[offset+3];
  /**
  * Properties
  */
  *broadcast         = (props_mask[0] & Handle_Value_Pair_Data[offset+2]);
  *read              = (props_mask[1] & Handle_Value_Pair_Data[offset+2])>>1;
  *write_wo_resp     = (props_mask[2] & Handle_Value_Pair_Data[offset+2])>>2;
  *write             = (props_mask[3] & Handle_Value_Pair_Data[offset+2])>>3;
  *notify            = (props_mask[4] & Handle_Value_Pair_Data[offset+2])>>4;
  *indicate          = (props_mask[5] & Handle_Value_Pair_Data[offset+2])>>5;
  *auth_signed_write = (props_mask[6] & Handle_Value_Pair_Data[offset+2])>>6;

  if ((BLUENRG_memcmp (uuid, ST_ENVIRONMENTAL_CHAR_UUID, (Handle_Value_Pair_Length-5))    == 0) ||
      (BLUENRG_memcmp (uuid, ST_ENVIRONMENTAL_ST_CHAR_UUID, (Handle_Value_Pair_Length-5)) == 0)) {
        *char_type = ST_ENVIRONMENTAL_CHAR_TYPE;
        char_name  = ST_ENVIRONMENTAL_CHAR_NAME;
  }
  else if (BLUENRG_memcmp (uuid, ST_PRESSURE_CHAR_UUID, (Handle_Value_Pair_Length-5)) == 0) {
    *char_type = ST_PRESSURE_CHAR_TYPE;
    char_name  = ST_PRESSURE_CHAR_NAME;
  }
  else if (BLUENRG_memcmp (uuid, ST_HUMIDITY_CHAR_UUID, (Handle_Value_Pair_Length-5)) == 0) {
    *char_type = ST_HUMIDITY_CHAR_TYPE;
    char_name  = ST_HUMIDITY_CHAR_NAME;
  }
  else if (BLUENRG_memcmp (uuid, ST_TEMPERATURE_CHAR_UUID, (Handle_Value_Pair_Length-5)) == 0) {
    *char_type = ST_TEMPERATURE_CHAR_TYPE;
    char_name  = ST_TEMPERATURE_CHAR_NAME;
  }
  else if (BLUENRG_memcmp (uuid, ST_LED_CHAR_UUID, (Handle_Value_Pair_Length-5)) == 0) {
    *char_type = ST_LED_CHAR_TYPE;
    char_name  = ST_LED_CHAR_NAME;
  }
  else if (BLUENRG_memcmp (uuid, ST_CONFIG_CHAR_UUID, (Handle_Value_Pair_Length-5)) == 0) {
    *char_type = ST_CONFIG_CHAR_TYPE;
    char_name  = ST_CONFIG_CHAR_NAME;
  }
  else if (BLUENRG_memcmp (uuid, ST_ACC_EVENT_CHAR_UUID, (Handle_Value_Pair_Length-5)) == 0) {
    *char_type = ST_ACC_EVENT_CHAR_TYPE;
    char_name  = ST_ACC_EVENT_CHAR_NAME;
  }
  else if (BLUENRG_memcmp (uuid, ST_MIC_EVENT_CHAR_UUID, (Handle_Value_Pair_Length-5)) == 0) {
    *char_type = ST_MIC_EVENT_CHAR_TYPE;
    char_name  = ST_MIC_EVENT_CHAR_NAME;
  }
  else if (BLUENRG_memcmp (uuid, ST_PROXIMITY_CHAR_UUID, (Handle_Value_Pair_Length-5)) == 0) {
    *char_type = ST_PROXIMITY_CHAR_TYPE;
    char_name  = ST_PROXIMITY_CHAR_NAME;
  }
  else if (BLUENRG_memcmp (uuid, ST_LUX_CHAR_UUID, (Handle_Value_Pair_Length-5)) == 0) {
    *char_type = ST_LUX_CHAR_TYPE;
    char_name  = ST_LUX_CHAR_NAME;
  }
  else if (BLUENRG_memcmp (uuid, ST_ACC_GYRO_MAG_CHAR_UUID, (Handle_Value_Pair_Length-5)) == 0) {
    *char_type = ST_ACC_GYRO_MAG_CHAR_TYPE;
    char_name  = ST_ACC_GYRO_MAG_CHAR_NAME;
  }
  else if (BLUENRG_memcmp (uuid, ST_QUATERNIONS_CHAR_UUID, (Handle_Value_Pair_Length-5)) == 0) {
    *char_type = ST_QUATERNIONS_CHAR_TYPE;
    char_name  = ST_QUATERNIONS_CHAR_NAME;
  }
  else if (BLUENRG_memcmp (uuid, ST_ECOMPASS_CHAR_UUID, (Handle_Value_Pair_Length-5)) == 0) {
    *char_type = ST_ECOMPASS_CHAR_TYPE;
    char_name  = ST_ECOMPASS_CHAR_NAME;
  }
  else if (BLUENRG_memcmp (uuid, ST_ACTIVITY_REC_CHAR_UUID, (Handle_Value_Pair_Length-5)) == 0) {
    *char_type = ST_ACTIVITY_REC_CHAR_TYPE;
    char_name  = ST_ACTIVITY_REC_CHAR_NAME;
  }
  else if (BLUENRG_memcmp (uuid, ST_CARRY_POSITION_REC_CHAR_UUID, (Handle_Value_Pair_Length-5)) == 0) {
    *char_type = ST_CARRY_POSITION_REC_CHAR_TYPE;
    char_name  = ST_CARRY_POSITION_REC_CHAR_NAME;
  }
  else if (BLUENRG_memcmp (uuid, ST_GESTURE_REC_CHAR_UUID, (Handle_Value_Pair_Length-5)) == 0) {
    *char_type = ST_GESTURE_REC_CHAR_TYPE;
    char_name  = ST_GESTURE_REC_CHAR_NAME;
  }
  else if (BLUENRG_memcmp (uuid, ST_ACC_PEDO_CHAR_UUID, (Handle_Value_Pair_Length-5)) == 0) {
    *char_type = ST_ACC_PEDO_CHAR_TYPE;
    char_name  = ST_ACC_PEDO_CHAR_NAME;
  }
  else if (BLUENRG_memcmp (uuid, ST_INTENSITY_DET_CHAR_UUID, (Handle_Value_Pair_Length-5)) == 0) {
    *char_type = ST_INTENSITY_DET_CHAR_TYPE;
    char_name  = ST_INTENSITY_DET_CHAR_NAME;
  }
  else if (BLUENRG_memcmp (uuid, ST_TERM_CHAR_UUID, (Handle_Value_Pair_Length-5)) == 0) {
    *char_type = ST_TERM_CHAR_TYPE;
    char_name  = ST_TERM_CHAR_NAME;
  }
  else if (BLUENRG_memcmp (uuid, ST_STDERR_CHAR_UUID, (Handle_Value_Pair_Length-5)) == 0) {
    *char_type = ST_STDERR_CHAR_TYPE;
    char_name  = ST_STDERR_CHAR_NAME;
  }
  else {
    *char_type = CUSTOM_CHAR_TYPE;
    char_name  = CUSTOM_CHAR_NAME;
  }
  PRINT_DBG("%s \n", char_name);

  *name_length = (strlen(char_name) > MAX_NAME_LENGTH) ? MAX_NAME_LENGTH : strlen(char_name);
  BLUENRG_memcpy(name, char_name, *name_length);

  Print_Characteristic_Info(dev_idx, serv_idx, char_idx);

  saved_devices.dev_info[dev_idx].serv_info[serv_idx].char_idx++;
  saved_devices.dev_info[dev_idx].serv_info[serv_idx].char_num++;

  PRINT_DBG(" ----------------------------------------------\r\n");
}

/*******************************************************************************
 * Function Name  : aci_att_read_resp_event.
 * Description    : A response event is received.
 * Input          : See file bluenrg1_events.h
 * Output         : See file bluenrg1_events.h
 * Return         : See file bluenrg1_events.h
 ******************************************************************************/
void aci_att_read_resp_event(uint16_t Connection_Handle, uint8_t Event_Data_Length,
                             uint8_t Attribute_Value[])
{
  uint8_t i;

  printf(" Value (HEX): %s", Event_Data_Length > 0 ? "0x" : "No value!");
  for (i=0; i<Event_Data_Length; i++) {
    printf("%02x", Attribute_Value[i]);
  }
  printf("\r\n");

  /* Print data in human readable format */
  if (Event_Data_Length > 0) {
    Print_HRF_Value(Event_Data_Length, Attribute_Value);
  }

  central_status = SELECT_ANOTHER_CHARACTERISTIC;
}

/*******************************************************************************
 * Function Name  : aci_gatt_notification_event.
 * Description    : A response event is received.
 * Input          : See file bluenrg1_events.h
 * Output         : See file bluenrg1_events.h
 * Return         : See file bluenrg1_events.h
 ******************************************************************************/
void aci_gatt_notification_event(uint16_t Connection_Handle,
                                 uint16_t Attribute_Handle,
                                 uint8_t Attribute_Value_Length,
                                 uint8_t Attribute_Value[])
{
  uint8_t i;

  printf(" Value (HEX): %s", Attribute_Value_Length > 0 ? "0x" : "No value!");
  for (i=0; i<Attribute_Value_Length; i++) {
    printf("%02x", Attribute_Value[i]);
  }
  printf("\r\n");

  /* Print data in human readable format */
  if (Attribute_Value_Length > 0) {
    Print_HRF_Value(Attribute_Value_Length, Attribute_Value);
  }
}
