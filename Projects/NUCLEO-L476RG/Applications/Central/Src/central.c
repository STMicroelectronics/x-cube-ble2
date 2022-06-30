/**
  ******************************************************************************
  * @file    central.c
  * @author  SRA Application Team
  * @brief   Central device init and state machine implementation
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
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "central.h"
#include "console.h"
#include "st_data_parser.h"
#include "bluenrg_conf.h"
#include "bluenrg1_hal_aci.h"
#include "bluenrg1_gap.h"
#include "bluenrg1_gap_aci.h"
#include "bluenrg1_hci_le.h"
#include "hci_const.h"
#include "bluenrg1_gatt_aci.h"

/* Private typedef -----------------------------------------------------------*/
/* Private defines -----------------------------------------------------------*/
#define BDADDR_SIZE        6

/* Private macros ------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
centralStatus_t  central_status = INIT_STATUS;
savedDevices_t   saved_devices;
nonConnDevices_t non_conn_devices;
serviceInfo_t    serv_info[MAX_NUM_OF_SERVICES];

/* Private function prototypes -----------------------------------------------*/
static uint8_t Get_BLEFirmware_Details(void);

/* Private functions ---------------------------------------------------------*/
/**
 * @brief  Get BlueNRG-2 hardware and firmware details
 *
 * @param  None
 * @retval Status
 */
uint8_t Get_BLEFirmware_Details(void)
{
  uint8_t status;

  uint8_t  DTM_version_major, DTM_version_minor, DTM_version_patch, DTM_variant;
  uint16_t DTM_Build_Number;
  uint8_t  BTLE_Stack_version_major, BTLE_Stack_version_minor, BTLE_Stack_version_patch,
           BTLE_Stack_development;
  uint16_t BTLE_Stack_variant, BTLE_Stack_Build_Number;
  uint8_t  BTLE_sv_patch, DTM_v_patch;
  uint8_t  alphabet[]={' ', 'a', 'b', 'c', 'd', 'e', 'f', 'g', 'h', 'i', 'j', 'k', 'l', 'm', 'n', 'o', 'p', 'q', 'r', 's', 't', 'u', 'v', 'w', 'x', 'y', 'z'};

  status = aci_hal_get_firmware_details(&DTM_version_major, &DTM_version_minor,
                                        &DTM_version_patch, &DTM_variant, &DTM_Build_Number,
                                        &BTLE_Stack_version_major, &BTLE_Stack_version_minor,
                                        &BTLE_Stack_version_patch, &BTLE_Stack_development,
                                        &BTLE_Stack_variant, &BTLE_Stack_Build_Number);

  BTLE_sv_patch = alphabet[BTLE_Stack_version_patch];
  DTM_v_patch = alphabet[DTM_version_patch];

  message(ANSI_COLOR_RESET);    /* reset foreground color */
  message(ANSI_CLEAR_SCREEN);   /* serial console clear screen */
  message(ANSI_CURSOR_TO_HOME); /* serial console cursor to home */
  message("\r\n");
  message(" --------------------------------------------------------\r\n\n");
  message(" BLE2 Universal Central v%d.%d.%d \r\n",
          CENTRAL_MAJOR_VERSION, CENTRAL_MINOR_VERSION, CENTRAL_PATCH_VERSION);
  message("\r\n");

  if (status == BLE_STATUS_SUCCESS) {
    message(" - BlueNRG-2 FW v%d.%d%c \r\n",
            BTLE_Stack_version_major, BTLE_Stack_version_minor, BTLE_sv_patch);
    message(" - DTM %s v%d.%d%c \r\n",
            DTM_variant == 0x01 ? "UART" : (DTM_variant == 0x02 ? "SPI" : "unknown"),
            DTM_version_major, DTM_version_minor, DTM_v_patch);
    message("\r\n");
  }

  return status;
}

/* Exported functions --------------------------------------------------------*/
/**
 * @brief  Init the Central device
 * @note
 * @param  None
 * @retval None
 */
uint8_t CentralDevice_Init(void)
{
  uint8_t ret;
  uint16_t service_handle, dev_name_char_handle, appearance_char_handle;
  uint8_t device_name[] = {'B','L','E','2','_','C','e','n','t','r','a','l'};
  uint8_t bdaddr[BDADDR_SIZE];
  uint8_t bdaddr_len_out;
  uint8_t config_data_stored_static_random_address = 0x80; /* Offset of the static random address stored in NVM */

  /* Sw reset of the device */
  hci_reset();
  /**
   *  To support both the BlueNRG-2 and the BlueNRG-2N a minimum delay of 2000ms is required at device boot
   */
  HAL_Delay(2000);

  /* get the BlueNRG HW and FW versions */
  ret = Get_BLEFirmware_Details();
  if (ret != BLE_STATUS_SUCCESS) {
    PRINT_DBG("Get_BLEFirmware_Details() --> Failed 0x%02x\r\n", ret);
    return ret;
  }

  ret = aci_hal_read_config_data(config_data_stored_static_random_address,
                                 &bdaddr_len_out, bdaddr);

  if (ret != BLE_STATUS_SUCCESS) {
    PRINT_DBG("aci_hal_read_config_data() --> Read Static Random address failed 0x%02x\r\n", ret);
    return ret;
  }
  if ((bdaddr[5] & 0xC0) != 0xC0) {
    PRINT_DBG("Static Random address not well formed.\r\n");
    while(1);
  }

  /* Set the TX power -2 dBm */
  aci_hal_set_tx_power_level(1, 4);
  if (ret != BLE_STATUS_SUCCESS)
  {
    PRINT_DBG("aci_hal_set_tx_power_level() --> Failed 0x%04x\r\n", ret);
    return ret;
  }

  /* GATT Init */
  ret = aci_gatt_init();
  if (ret != BLE_STATUS_SUCCESS)
  {
    PRINT_DBG("aci_gatt_init() failed: 0x%02x\r\n", ret);
    return ret;
  }

  /* GAP Init */
  ret = aci_gap_init(GAP_CENTRAL_ROLE, 0, sizeof(device_name), &service_handle, &dev_name_char_handle,
                     &appearance_char_handle);
  if (ret != BLE_STATUS_SUCCESS)
  {
    PRINT_DBG("aci_gap_init() --> Failed 0x%02x\r\n", ret);
    return ret;
  }

  /* Update device name */
  ret = aci_gatt_update_char_value(service_handle, dev_name_char_handle, 0, sizeof(device_name),
                                   device_name);
  if (ret != BLE_STATUS_SUCCESS)
  {
    PRINT_DBG("aci_gatt_update_char_value() --> Failed 0x%02x\r\n", ret);
    return ret;
  }

  return ret;
}

/**
 * @brief  Init struct containing all saved devices
 * @param  None
 * @retval None
 */
void Init_Saved_Devices(void)
{
  uint8_t i, j, t;

  for (i=0; i<MAX_NUM_OF_DEVICES; i++) {
    BLUENRG_memset(saved_devices.dev_info[i].bdaddr, 0 , sizeof(saved_devices.dev_info[i].bdaddr));
    saved_devices.dev_info[i].addr_type   = 0;
    saved_devices.dev_info[i].name_length = 0;
    BLUENRG_memset(saved_devices.dev_info[i].name, 0, MAX_NAME_LENGTH);
    saved_devices.dev_info[i].serv_idx    = 0;
    saved_devices.dev_info[i].serv_num    = 0;
    saved_devices.dev_info[i].conn_handle = 0;
    saved_devices.dev_info[i].serv_info   = NULL;
  }
  for (j=0; j<MAX_NUM_OF_SERVICES; j++) {
    serv_info[j].char_idx     = 0;
    serv_info[j].char_num     = 0;
    serv_info[j].start_handle = 0;
    serv_info[j].end_handle   = 0;
    serv_info[j].name_length  = 0;
    BLUENRG_memset(serv_info[j].name, 0, MAX_NAME_LENGTH);
    serv_info[j].serv_type    = NO_SERVICE_TYPE;
    serv_info[j].uuid_length  = 0;
    BLUENRG_memset(serv_info[j].uuid, 0, UUID_MAX_LENGTH);
    for (t=0; t<MAX_NUM_OF_CHARS; t++) {
      serv_info[j].char_info[t].uuid_length       = 0;
      BLUENRG_memset(serv_info[j].char_info[t].uuid, 0, UUID_MAX_LENGTH);
      serv_info[j].char_info[t].name_length       = 0;
      BLUENRG_memset(serv_info[j].char_info[t].name, 0, MAX_NAME_LENGTH);
      serv_info[j].char_info[t].uuid_length       = 0;
      serv_info[j].char_info[t].decl_handle       = 0;
      serv_info[j].char_info[t].value_handle      = 0;
      serv_info[j].char_info[t].prop_idx          = 0;
      serv_info[j].char_info[t].broadcast         = 0;
      serv_info[j].char_info[t].read              = 0;
      serv_info[j].char_info[t].write_wo_resp     = 0;
      serv_info[j].char_info[t].write             = 0;
      serv_info[j].char_info[t].notify            = 0;
      serv_info[j].char_info[t].indicate          = 0;
      serv_info[j].char_info[t].auth_signed_write = 0;
      serv_info[j].char_info[t].char_type = NO_CHARACTERISTIC_TYPE;
    }
  }

  saved_devices.dev_idx   = 0;
  saved_devices.connected = 0;
  saved_devices.dev_num   = 0;
}

/**
 * @brief  Init struct containing all non connectable devices
 * @param  None
 * @retval None
 */
void Init_NonConn_Devices()
{
  uint8_t i;

  for (i=0; i<MAX_NUM_OF_DEVICES; i++) {
    BLUENRG_memset(non_conn_devices.bdaddr[i], 0 , sizeof(non_conn_devices.bdaddr[i]));
  }

  non_conn_devices.dev_idx=0;
  non_conn_devices.dev_num=0;
}

/**
 * @brief  Close the connection with the peripheral device
 * @param  None
 * @retval None
 */
void Close_Connection(void)
{
  uint8_t i = saved_devices.dev_idx;
  int ret;

  ret = aci_gap_terminate(saved_devices.dev_info[i].conn_handle, BLE_ERROR_TERMINATED_LOCAL_HOST);

  if (ret != BLE_STATUS_SUCCESS){
    printf("aci_gap_terminate() failed: %02X\n",ret);
  }

  HAL_Delay(100); /* see comment @file bluenrg1_gap_aci.h, procedure aci_gap_terminate() */
}

/**
 * @brief  Start searching for a peripheral device
 * @param  None
 * @retval None
 */
void Start_Scanning(void)
{
  int ret;

  /* scanInterval, scanWindow, own_address_type, filterDuplicates */
  ret = aci_gap_start_general_discovery_proc(SCAN_P, SCAN_L, PUBLIC_ADDR, 1);

  if (ret != BLE_STATUS_SUCCESS){
    printf("aci_gap_start_general_discovery_proc() failed: %02X\n",ret);
  }
}

/**
 * @brief  Start connection with a peripheral device
 * @param  Device index to connect
 * @retval None
 */
void Start_Connection(uint8_t dev_index)
{
  uint8_t ret;

  ret = aci_gap_create_connection(SCAN_P, SCAN_L,
                                  saved_devices.dev_info[dev_index].addr_type,
                                  saved_devices.dev_info[dev_index].bdaddr,
                                  PUBLIC_ADDR, 40, 40, 0, 60, 2000 , 2000);

  if (ret != BLE_STATUS_SUCCESS) {
    printf("aci_gap_create_connection() failed: 0x%02x\n", ret);
  }
  else {
    PRINT_DBG("aci_gap_create_connection() OK\n");
  }
}

/**
 * @brief  Services discovery
 * @param  uint8_t The device index
 * @retval None
 */
void Discover_Services(uint8_t dev_index)
{
  uint8_t ret;

  PRINT_DBG(" connection handle 0x%04x\n", saved_devices.dev_info[dev_index].conn_handle);
  ret = aci_gatt_disc_all_primary_services(saved_devices.dev_info[dev_index].conn_handle);

  if (ret != BLE_STATUS_SUCCESS) {
    printf("aci_gatt_disc_all_primary_services() failed: 0x%02x\n", ret);
  }
  else {
    PRINT_DBG("aci_gatt_disc_all_primary_services() OK\n");
  }
}

/**
 * @brief  Discover for each service all characteristics
 * @param  uint8_t The device index
 * @param  uint8_t The service index
 * @retval None
 */
void Discover_Characteristics(uint8_t dev_idx, uint8_t serv_idx)
{
  uint16_t conn_handle  = saved_devices.dev_info[dev_idx].conn_handle;
  uint16_t start_handle = saved_devices.dev_info[dev_idx].serv_info[serv_idx].start_handle;
  uint16_t end_handle   = saved_devices.dev_info[dev_idx].serv_info[serv_idx].end_handle;
  uint8_t  ret;

  ret = aci_gatt_disc_all_char_of_service(conn_handle, start_handle, end_handle);
  if (ret != BLE_STATUS_SUCCESS) {
    printf("aci_gatt_disc_all_char_of_service() failed: 0x%02x\n", ret);
  }
  else {
    PRINT_DBG("aci_gatt_disc_all_char_of_service() OK\n");
  }
}

/**
 * @brief  Update a characteristic property
 * @param  uint8_t The device index
 * @param  uint8_t The service index
 * @param  uint8_t The characteristic index
 * @param  uint8_t The property index
 * @retval None
 */
void Update_Characteristic (uint8_t dev_idx, uint8_t serv_idx, uint8_t char_idx,
                            uint8_t prop_idx)
{
  uint16_t connection_handle = saved_devices.dev_info[dev_idx].conn_handle;
  uint16_t attr_handle;

  uint8_t ret;

  if (serv_idx > (saved_devices.dev_info[dev_idx].serv_num - 1))
  {
    printf(" The service x=%d does not exist!\r\n", serv_idx);
    central_status = SELECT_ANOTHER_CHARACTERISTIC;
    return;
  }
  if ((saved_devices.dev_info[dev_idx].serv_info[serv_idx].serv_type == GENERIC_ACCESS_PROFILE_TYPE) ||
      (saved_devices.dev_info[dev_idx].serv_info[serv_idx].serv_type == GENERIC_ATTRIBUTE_PROFILE_TYPE))
  {
    printf(" Services:\n - %s\n - %s\n are not supported!\r\n",
           GENERIC_ACCESS_PROFILE_NAME, GENERIC_ATTRIBUTE_PROFILE_NAME);
    central_status = SELECT_ANOTHER_CHARACTERISTIC;
    return;
  }
  if (char_idx > (saved_devices.dev_info[dev_idx].serv_info[serv_idx].char_num - 1))
  {
    printf(" The characteristic y=%d does not exist!\r\n", char_idx);
    central_status = SELECT_ANOTHER_CHARACTERISTIC;
    return;
  }

  switch (prop_idx)
  {
  case (1):
    if (saved_devices.dev_info[dev_idx].serv_info[serv_idx].char_info[char_idx].read) {
      attr_handle = saved_devices.dev_info[dev_idx].serv_info[serv_idx].char_info[char_idx].decl_handle + 1;
      ret = aci_gatt_read_char_value(connection_handle, attr_handle);
      if (ret != BLE_STATUS_SUCCESS) {
        printf(" Unable to read data from device %d (err 0x%02x, 0x%04x - 0x%04x)\r\n",
               dev_idx, ret, connection_handle, attr_handle);
        central_status = SELECT_ANOTHER_CHARACTERISTIC;
      }
    }
    else {
      printf(" The selected characteristic is not readable!\r\n");
      central_status = SELECT_ANOTHER_CHARACTERISTIC;
    }
    break;
  case (2):
  case (3):
    if (saved_devices.dev_info[dev_idx].serv_info[serv_idx].char_info[char_idx].write) {
      /**
       * With an ATT write request (or notification), 3 bytes are
       * used by command type and attribute ID, 20 bytes are left
       * for the attribute data.
       */
      uint8_t attribute_val_length;
      uint8_t attribute_val[MAX_STRING_LENGTH];

      printf("\n Type the value (max %d characters, 'ENTER' to send): ", MAX_STRING_LENGTH);
      printf("\n\n ");
      attribute_val_length = Get_Value(attribute_val);
      printf("\n");

      central_status = WRITING_CHARACTERISTIC_VALUE;

      attr_handle = saved_devices.dev_info[dev_idx].serv_info[serv_idx].char_info[char_idx].value_handle;
      ret = aci_gatt_write_char_value(connection_handle, attr_handle,
                                      attribute_val_length, attribute_val);
      if (ret != BLE_STATUS_SUCCESS) {
        printf(" Unable to write characteristic %d (dev %d, serv %d, err 0x%02x, 0x%04x - 0x%04x)\r\n",
               char_idx, dev_idx, serv_idx, ret, connection_handle, attr_handle);
        central_status = SELECT_ANOTHER_CHARACTERISTIC;
      }
    }
    else {
      printf(" The selected characteristic is not writable!\r\n");
      central_status = SELECT_ANOTHER_CHARACTERISTIC;
    }
    break;
  case (4):
    if (saved_devices.dev_info[dev_idx].serv_info[serv_idx].char_info[char_idx].notify) {
      Set_Notifications(dev_idx, serv_idx, char_idx, 0x01); /* enable notifications */
    }
    else {
      printf(" The notify property can't be enabled for the selected characteristic!\r\n");
      central_status = SELECT_ANOTHER_CHARACTERISTIC;
    }
    break;
  case (5):
    if (saved_devices.dev_info[dev_idx].serv_info[serv_idx].char_info[char_idx].indicate) {
      Set_Notifications(dev_idx, serv_idx, char_idx, 0x02); /* enable indications */
    }
    else {
      printf(" The indicate property can't be enabled for the selected characteristic!\r\n");
      central_status = SELECT_ANOTHER_CHARACTERISTIC;
    }
    break;
  case (0):
  case (6):
    printf(" The property z=%d is not supported!\r\n", prop_idx);
    central_status = SELECT_ANOTHER_CHARACTERISTIC;
    break;
  default:
    printf(" The property z=%d does not exist!\r\n", prop_idx);
    central_status = SELECT_ANOTHER_CHARACTERISTIC;
    break;
  }

}

/**
 * @brief  Enable/Disable notifications and indications
 * @param  uint8_t The device index
 * @param  uint8_t The service index
 * @param  uint8_t The characteristic index
 * @param  uint8_t The new status:
 *                 0x00 disable both notifications and indications
 *                 0x01 enable notifications
 *                 0x02 enable indications
 *                 0x03 enable both notifications and indications
 * @param  uint8_t The property index
 * @retval None
 */
void Set_Notifications(uint8_t dev_idx, uint8_t serv_idx, uint8_t char_idx, uint8_t status)
{
  uint16_t connection_handle = saved_devices.dev_info[dev_idx].conn_handle;
  uint8_t  attribute_val_length = 2;
  /**
   * status = 0x00 disable both notifications and indications
   * status = 0x01 enable notifications
   * status = 0x02 enable indications
   * status = 0x03 enable both notifications and indications
   */
  uint8_t attribute_val[] = {status, 0x00};
  uint8_t ret;

  uint16_t attr_handle = saved_devices.dev_info[dev_idx].serv_info[serv_idx].char_info[char_idx].decl_handle + 2;
  ret = aci_gatt_write_char_desc(connection_handle, attr_handle,
                                 attribute_val_length, attribute_val);
  if (ret != BLE_STATUS_SUCCESS) {
    printf(" Unable to change notification/indication property on device %d (err 0x%02x, 0x%04x - 0x%04x)\r\n",
           dev_idx, ret, connection_handle, attr_handle);
    central_status = SELECT_ANOTHER_CHARACTERISTIC;
  }
}

/**
 * @brief  Save the found device in the struct containing all the found devices
 * @param  addr
 * @param  address type (0x00 Public Device Address, 0x01 Random Device Address)
 * @param  data_length
 * @param  data_value
 * @param  position in the struct
 * @retval None
 */
void Save_Found_Device(tBDAddr addr, uint8_t* addr_type, uint8_t data_length,
                       uint8_t* data_value, uint8_t index)
{
  uint8_t i = 0;

  BLUENRG_memcpy(saved_devices.dev_info[index].bdaddr, addr, 6);
  saved_devices.dev_info[index].addr_type   = *addr_type;
  saved_devices.dev_info[index].name_length = strlen("Unknown");
  BLUENRG_memcpy(saved_devices.dev_info[index].name, "Unknown",
                 (saved_devices.dev_info[index].name_length));

  while (i < data_length)
  {
    /* Advertising data fields: len, type, values */
    /* Check if field is a complete or a short local name */
    if ((data_value[i+1] == AD_TYPE_COMPLETE_LOCAL_NAME) ||
        (data_value[i+1] == AD_TYPE_SHORTENED_LOCAL_NAME))
    {
      saved_devices.dev_info[index].name_length = (data_value[i]-1);
      BLUENRG_memcpy(saved_devices.dev_info[index].name, &data_value[i+2],
                     (saved_devices.dev_info[index].name_length));
      break;
    }
    else
    {
      /* move to next advertising field */
      i += (data_value[i] + 1);
    }
  }

  return;
}

/**
 * @brief  Save the found device in the struct containing all the non connectable
 *         devices
 * @param  addr
 * @retval None
 */
void Save_NonConn_Device(tBDAddr addr)
{
  uint8_t i = non_conn_devices.dev_idx;

  BLUENRG_memcpy(non_conn_devices.bdaddr[i], addr, 6);

  non_conn_devices.dev_num++;
  non_conn_devices.dev_idx++;
}

/**
 * @brief  Print all information related to a saved device
 * @param  device position in the saved devices structure
 * @retval None
 */
void Print_Device_Info(uint8_t device_index)
{
  uint8_t i = device_index;

  /* print the address */
  printf("\r\n");
  printf(" %d. BLE device: address %02x:%02x:%02x:%02x:%02x:%02x\r\n",
         i, saved_devices.dev_info[i].bdaddr[5], saved_devices.dev_info[i].bdaddr[4],
         saved_devices.dev_info[i].bdaddr[3], saved_devices.dev_info[i].bdaddr[2],
         saved_devices.dev_info[i].bdaddr[1], saved_devices.dev_info[i].bdaddr[0]);

  /* the device is connectable */
  printf("                connectable\n");

  /* print the address type */
  printf("                %s address type\n",
         (saved_devices.dev_info[i].addr_type==0x00 ? "public" : (saved_devices.dev_info[i].addr_type==0x01 ? "private" : "unknown")));

  /* print the name */
  printf("                name ");
  printf("%s", saved_devices.dev_info[i].name);
  printf("\n");

  if (saved_devices.dev_info[i].conn_handle != 0) {
    printf("                connection handle 0x%04x\n", saved_devices.dev_info[i].conn_handle);
  }

  return;
}

/**
 * @brief  Print info about non connectable devices (e.g. Beacon)
 * @param  addr
 * @param  address type (0x00 Public Device Address, 0x01 Random Device Address)
 * @param  data_length
 * @param  data_value
 * @retval None
 */
void Print_NonConn_Device(tBDAddr addr, uint8_t* addr_type, uint8_t data_length,
                          uint8_t* data_value)
{
  uint8_t i = 0;
  uint8_t name_length;
  uint8_t name[MAX_NAME_LENGTH];

  printf(ANSI_COLOR_WHITE); /* to print logs in matt white (like a gray) */

  /* print the address */
  printf("\r\n");
  printf("    BLE device: address %02x:%02x:%02x:%02x:%02x:%02x\r\n",
         addr[5], addr[4], addr[3], addr[2], addr[1], addr[0]);
  printf("                non-connectable\n");

  /* print the address type */
  printf("                %s address type\n",
         (*addr_type==0x00 ? "public" : (*addr_type==0x01 ? "private" : "unknown")));

  name_length = strlen("Unknown");
  BLUENRG_memcpy(name, "Unknown", name_length);
  while (i < data_length)
  {
    /* Advertising data fields: len, type, values */
    /* Check if field is a complete or a short local name */
    if ((data_value[i+1] == AD_TYPE_COMPLETE_LOCAL_NAME) ||
        (data_value[i+1] == AD_TYPE_SHORTENED_LOCAL_NAME))
    {
      name_length = (data_value[i]-1);
      BLUENRG_memcpy(name, &data_value[i+2], name_length);
      break;
    }
    else
    {
      /* move to next advertising field */
      i += (data_value[i] + 1);
    }
  }

  /* print the name */
  printf("                name ");
  printf("%s\n", name);

  if (data_value[0]==0x02) { /* the AD Type Flags are at the beginning of the Advertising packet */
    i=3;
  }
  else { /* no AD Type Flags at the beginning of the Advertising packet */
    i=0;
  }
  /**
   * iBeacon
   *
   * (byte pos)   0   1   2   3   4    5-6   7   8      9-24  25-26  27-28     29
   *             FL  FT  FT  FL  FT  Manuf  st  FL  Loc-UUID    Maj    Min  SigPW
   * (hex)       02  01  06  1a  FF  30 00  02  15
   *
   * FL = Field Length
   * FT = Field Type
   * Manuf = Manufacturer ID (0x0030 STMicroelectronics)
   * st = SubType (0x02 iBeacon)
   * Loc-UUID = Location UUID
   * Maj = Major number
   * Min = Minor number
   * SigPW = Signal Power
  */
  if ((data_value[1+i]==AD_TYPE_MANUFACTURER_SPECIFIC_DATA) && (data_value[4+i]==0x02)) {
    printf("                iBeacon ");
    if ((data_value[1+i]==0xFF) &&
        (data_value[2+i]==0x30) &&
          (data_value[3+i]==0x00)) {//Company identifier code (Default is 0x0030 - STMicroelectronics))
            printf("[Manuf.: ST]");
          }
    else {
      printf("[Manuf.: Unknown]");
    }
    printf("\n");
  }
  /**
   * Eddystone Beacon Advertising Packet
   * (byte pos)  0   1   2   3   4     5-6     7     8    9-10
   *            FL  FT  fl  FL  CL  E-UUID    FL    SD  E-UUID
   * (hex)      02  01  06  03  03   AA FE  0xnn  0x16   AA FE
   *
   * FL = Field Length
   * FT = Field Type
   * fl = LE and BR/EDR flag
   * CL = Complete list of service UUID
   * SD = Service Data
   * E-UUID = Eddystone UUID (0xFEAA)
   */
  else if ((data_value[5+i]==AD_TYPE_SERVICE_DATA) && (data_value[6+i]==0xAA) && (data_value[7+i]==0xFE)) {
    printf("                Eddystone Beacon ");
    if (data_value[8+i]==0x00) {
      printf("[UID]");
    }
    else if (data_value[8+i]==0x10) {
      printf("[URL]");
    }
    else if (data_value[8+i]==0x20) {
      printf("[TLM]");
    }
    printf("\n");
  }

  printf(ANSI_COLOR_RESET); /* reset color to default value for log print */

  return;
}

/**
 * @brief  Print all information related to a device service
 * @param  device position in the saved devices structure
 * @param  service position in the saved services structure
 * @retval None
 */
void Print_Service_Info(uint8_t dev_idx, uint8_t serv_idx)
{
  uint8_t i;
  uint8_t len;

  len = saved_devices.dev_info[dev_idx].serv_info[serv_idx].name_length;

  printf(ANSI_COLOR_RESET); /* reset color to default value for log print */
  if ((saved_devices.dev_info[dev_idx].serv_info[serv_idx].serv_type == GENERIC_ACCESS_PROFILE_TYPE) ||
      (saved_devices.dev_info[dev_idx].serv_info[serv_idx].serv_type == GENERIC_ATTRIBUTE_PROFILE_TYPE))
  {
    printf(ANSI_COLOR_WHITE); /* to print logs in matt white (like a gray) */
  }

  printf(" %d. ", serv_idx);
  for (i=0; i<len; i++) {
    printf("%c", saved_devices.dev_info[dev_idx].serv_info[serv_idx].name[i]);
  }
  printf("\n");

  len = saved_devices.dev_info[dev_idx].serv_info[serv_idx].uuid_length;
  printf("    - UUID ");
  for (i=len; i>0; i--) {
    printf("%02x", saved_devices.dev_info[dev_idx].serv_info[serv_idx].uuid[i-1]);
    if ((i==13) || (i==11) || (i==9) || (i==7)) {
      printf("-");
    }
  }
  printf("\n");
  printf("    - attribute handles 0x%04x - 0x%04x\n",
         saved_devices.dev_info[dev_idx].serv_info[serv_idx].start_handle,
         saved_devices.dev_info[dev_idx].serv_info[serv_idx].end_handle);
}

/**
 * @brief  Print all information related to a service characteristic
 * @param  device position in the saved device struct
 * @param  service position in the saved service struct
 * @param  characteristic position in the saved characteristic struct
 * @retval None
 */
void Print_Characteristic_Info(uint8_t dev_idx, uint8_t serv_idx, uint8_t char_idx)
{
  uint8_t  i, len;

  len = saved_devices.dev_info[dev_idx].serv_info[serv_idx].char_info[char_idx].name_length;
  /* Char name */
  printf("    %d. ", char_idx);
  for (i=0; i<len; i++) {
    printf("%c", saved_devices.dev_info[dev_idx].serv_info[serv_idx].char_info[char_idx].name[i]);
  }
  printf("\n");

  len = saved_devices.dev_info[dev_idx].serv_info[serv_idx].char_info[char_idx].uuid_length;

  /* Char UUID */
  printf("       -. UUID ");
  for (i=len; i>0; i--) {
    printf("%02x", saved_devices.dev_info[dev_idx].serv_info[serv_idx].char_info[char_idx].uuid[i-1]);
    if ((i==13) || (i==11) || (i==9) || (i==7)) {
      printf("-");
    }
  }
  printf("\n");

  /* Char declaration and value handles */
  printf("       -. declaration_handle = 0x%04x\n",
         saved_devices.dev_info[dev_idx].serv_info[serv_idx].char_info[char_idx].decl_handle);
  printf("       -. value_handle       = 0x%04x\n",
         saved_devices.dev_info[dev_idx].serv_info[serv_idx].char_info[char_idx].value_handle);

  /* Char properties */
  printf("       0. broadcast          = %d\n",
         saved_devices.dev_info[dev_idx].serv_info[serv_idx].char_info[char_idx].broadcast);
  printf("       1. read               = %d\n",
         saved_devices.dev_info[dev_idx].serv_info[serv_idx].char_info[char_idx].read);
  printf("       2. writeWoResp        = %d\n",
         saved_devices.dev_info[dev_idx].serv_info[serv_idx].char_info[char_idx].write_wo_resp);
  printf("       3. write              = %d\n",
         saved_devices.dev_info[dev_idx].serv_info[serv_idx].char_info[char_idx].write);
  printf("       4. notify             = %d\n",
         saved_devices.dev_info[dev_idx].serv_info[serv_idx].char_info[char_idx].notify);
  printf("       5. indicate           = %d\n",
         saved_devices.dev_info[dev_idx].serv_info[serv_idx].char_info[char_idx].indicate);
  printf("       6. authSignedWrite    = %d\n",
         saved_devices.dev_info[dev_idx].serv_info[serv_idx].char_info[char_idx].auth_signed_write);

  printf(ANSI_COLOR_RESET);    /* reset previous color for log print */

}

/**
 * @brief  Recognize a data value from a ST characteristic and print it in
 *         human readable format
 * @param  The data length
 * @param  The received data
 * @retval None
 */
void Print_HRF_Value(uint8_t data_length, uint8_t* value)
{
  uint16_t tmp_ui16 = 0;
  uint32_t tmp_ui32 = 0;
  uint8_t  dev_idx  = saved_devices.dev_idx;
  uint8_t  serv_idx = saved_devices.dev_info[dev_idx].serv_idx;
  uint8_t  char_idx = saved_devices.dev_info[dev_idx].serv_info[serv_idx].char_idx;

  if ((saved_devices.dev_info[dev_idx].serv_info[serv_idx].serv_type != NO_SERVICE_TYPE) &&
      (saved_devices.dev_info[dev_idx].serv_info[serv_idx].serv_type != CUSTOM_SERVICE_TYPE))
  {
    if ((saved_devices.dev_info[dev_idx].serv_info[serv_idx].char_info[char_idx].char_type != NO_CHARACTERISTIC_TYPE) &&
        (saved_devices.dev_info[dev_idx].serv_info[serv_idx].char_info[char_idx].char_type != CUSTOM_CHAR_TYPE))
    {
      printf(" Value (HRF): ");
      /* Timestamp */
      printf("TStamp %05d ", Get_Timestamp(data_length, value));

      switch (saved_devices.dev_info[dev_idx].serv_info[serv_idx].char_info[char_idx].char_type)
      {
      case (ST_ENVIRONMENTAL_CHAR_TYPE):
        /* Pressure */
        tmp_ui32 = Get_Pressure(data_length, value);
        printf("Press[mBar] %d.%d ", (uint16_t)(tmp_ui32/100), (uint16_t)(tmp_ui32%100));
        if (data_length == ENV_DATA_LEN_LONG) {
          /* Humidity */
          tmp_ui16 = Get_Humidity(data_length, value);
          printf("Hum[%c] %d.%d ", '%', tmp_ui16/10, tmp_ui16%10);
          /* Temperature 2 */
          tmp_ui16 = Get_Temperature(data_length, value, 2);
          printf("Temp2[C] %d.%d ", tmp_ui16/10, tmp_ui16%10);
          /* Temperature 1 */
          tmp_ui16 = Get_Temperature(data_length, value, 1);
          printf("Temp1[C] %d.%d \r\n", tmp_ui16/10, tmp_ui16%10);
        }
        else if (data_length == ENV_DATA_LEN_SHORT) {
          /* Temperature 1 */
          tmp_ui16 = Get_Temperature(data_length, value, 1);
          printf("Temp1[C] %d.%d \r\n", tmp_ui16/10, tmp_ui16%10);
        }
        break;
      case (ST_PRESSURE_CHAR_TYPE):
        /* Pressure */
        tmp_ui32 = Get_Pressure(data_length, value);
        printf("Press[mBar] %d.%d \r\n", (uint16_t)(tmp_ui32/100), (uint16_t)(tmp_ui32%100));
        break;
      case (ST_HUMIDITY_CHAR_TYPE):
        /* Humidity */
        tmp_ui16 = Get_Humidity(data_length, value);
        printf("Hum[%c] %d.%d \r\n", '%', tmp_ui16/10, tmp_ui16%10);
        break;
      case (ST_TEMPERATURE_CHAR_TYPE):
        /* Temperature 1 */
        tmp_ui16 = Get_Temperature(data_length, value, 1);
        printf("Temp1[C] %d.%d \r\n", tmp_ui16/10, tmp_ui16%10);
        break;
      case (ST_CO_CHAR_TYPE):
        /* CO */
        tmp_ui32 = Get_CO(data_length, value);
        printf("CO[%c] %d.%d", '%', (uint16_t)(tmp_ui32/100), (uint16_t)(tmp_ui32%100));
        break;
      case (ST_LED_CHAR_TYPE):
        {
          /* LED status (0=OFF, 1=ON) */
          uint8_t tmp_ui8 = Get_LED_Status(data_length, value);
          printf("LED %s \r\n", ((tmp_ui8 == 0) ? "OFF" : "ON"));
        }
        break;
      case (ST_CONFIG_CHAR_TYPE):
        printf(" HRF not supported! \r\n");
        break;
      case (ST_ACC_EVENT_CHAR_TYPE):
        printf(" Acc. Event: %s \r\n", Get_Acc_Event(data_length, value).type_name);
      break;
      case (ST_MIC_EVENT_CHAR_TYPE):
        /* Microphone [db] */
        printf("Mic[db] %d \r\n", Get_Mic_Audio_Level(data_length, value));
        break;
      case (ST_PROXIMITY_CHAR_TYPE):
        /* Proximity [mm] */
        printf("PRX[mm] %d \r\n", Get_Proximity(data_length, value));
        break;
      case (ST_LUX_CHAR_TYPE):
        /* Brightness */
        printf("LUX %d \r\n", Get_Lux_Level(data_length, value));
        break;
      case (ST_ACC_GYRO_MAG_CHAR_TYPE):
        {
          motionData_t motion = Get_Motion_Data(data_length, value);
          printf("ACC[mg]  X %s%d Y %s%d Z %s%d ",
                 (motion.acc.x.is_neg == 1) ? "-" : "", motion.acc.x.val,
                 (motion.acc.y.is_neg == 1) ? "-" : "", motion.acc.y.val,
                 (motion.acc.z.is_neg == 1) ? "-" : "", motion.acc.z.val);
          printf("\n                           GYR[dps] ");
          printf("X %s%d.%d Y %s%d.%d Z %s%d.%d ",
                 (motion.gyr.x.is_neg == 1) ? "-" : "", motion.gyr.x.val/10, motion.gyr.x.val%10,
                 (motion.gyr.y.is_neg == 1) ? "-" : "", motion.gyr.y.val/10, motion.gyr.y.val%10,
                 (motion.gyr.z.is_neg == 1) ? "-" : "", motion.gyr.z.val/10, motion.gyr.z.val%10);
          printf("\n                           MAG[mGa] ");
          printf("X %s%d Y %s%d Z %s%d ",
                 (motion.mag.x.is_neg == 1) ? "-" : "", motion.mag.x.val,
                 (motion.mag.y.is_neg == 1) ? "-" : "", motion.mag.y.val,
                 (motion.mag.z.is_neg == 1) ? "-" : "", motion.mag.z.val);
          printf("\r\n");
        }
        break;
      case (ST_QUATERNIONS_CHAR_TYPE):
        {
          quatData_t quat = Get_Quaternions_Data(data_length, value);
          printf("QUAT1 ");
          printf("X %s%d Y %s%d Z %s%d ",
                 (quat.q1.x.is_neg == 1) ? "-" : "", quat.q1.x.val,
                 (quat.q1.y.is_neg == 1) ? "-" : "", quat.q1.y.val,
                 (quat.q1.z.is_neg == 1) ? "-" : "", quat.q1.z.val);
          if ((data_length == QUATERNIONS_2_DATA_LEN) || (data_length == QUATERNIONS_3_DATA_LEN)) {
            printf("\n                           QUAT2 ");
            printf("X %s%d Y %s%d Z %s%d ",
                   (quat.q2.x.is_neg == 1) ? "-" : "", quat.q2.x.val,
                   (quat.q2.y.is_neg == 1) ? "-" : "", quat.q2.y.val,
                   (quat.q2.z.is_neg == 1) ? "-" : "", quat.q2.z.val);
          }
          if (data_length == QUATERNIONS_3_DATA_LEN) {
            printf("\n                           QUAT3 ");
            printf("X %s%d Y %s%d Z %s%d ",
                   (quat.q3.x.is_neg == 1) ? "-" : "", quat.q3.x.val,
                   (quat.q3.y.is_neg == 1) ? "-" : "", quat.q3.y.val,
                   (quat.q3.z.is_neg == 1) ? "-" : "", quat.q3.z.val);
          }
          printf("\r\n");
        }
        break;
      case (ST_ECOMPASS_CHAR_TYPE):
        /* E-Compass (angle to Magnetic North in cents of degree [0.00 -> 359,99]) */
        tmp_ui16 = Get_ECompass(data_length, value);
        printf("E-Compass[degree]: %d.%d \r\n", tmp_ui16/100, tmp_ui16%100);
        break;
      case (ST_ACTIVITY_REC_CHAR_TYPE):
        /* Activity Recognition */
        printf("Activity Rec.: %s \r\n", Get_Activity_Recognition(data_length, value).type_name);
        break;
      case (ST_CARRY_POSITION_REC_CHAR_TYPE):
        /* Carry Position Recognition */
        printf("Carry Pos. Rec.: %s \r\n", Get_Carry_Pos_Recognition(data_length, value).type_name);
        break;
      case (ST_GESTURE_REC_CHAR_TYPE):
        /* Gesture Recognition */
        printf("Gest. Rec.: %s \r\n", Get_Gesture_Recognition(data_length, value).type_name);
        break;
      case (ST_ACC_PEDO_CHAR_TYPE):
        /* Pedometer */
        printf("Pedometer: ");
        printf("steps %d ", (uint16_t)(Get_Pedometer_Info(data_length, value).steps));
        printf("steps/min %d \r\n", Get_Pedometer_Info(data_length, value).steps_min);
        break;
      case (ST_INTENSITY_DET_CHAR_TYPE):
        /* Intensity Detection */
        printf("Int. Det.: %s \r\n", Get_Motion_Intensity(data_length, value).type_name);
        break;
      default:
        printf(" \r\n");
        break;
      }
    }
  }

}

/**
 * @brief  Search for an already saved device
 * @param  address of the device to search for
 * @retval TRUE if the device has been already saved, FALSE otherwise
 */
uint8_t Is_Device_Saved(tBDAddr addr)
{
  uint8_t i;

  for (i=0; i<saved_devices.dev_num; i++) {
    PRINT_DBG("Saved device %d: %02x:%02x:%02x:%02x:%02x:%02x\n", i,
           saved_devices.dev_info[i].bdaddr[5], saved_devices.dev_info[i].bdaddr[4],
           saved_devices.dev_info[i].bdaddr[3], saved_devices.dev_info[i].bdaddr[2],
           saved_devices.dev_info[i].bdaddr[1], saved_devices.dev_info[i].bdaddr[0]);
    if (BLUENRG_memcmp(addr, saved_devices.dev_info[i].bdaddr, sizeof(saved_devices.dev_info[i].bdaddr)) == 0)
    {
      PRINT_DBG("Current device %02x:%02x:%02x:%02x:%02x:%02x already saved at pos %d\n",
                 addr[5], addr[4], addr[3], addr[2], addr[1], addr[0], i);
      return TRUE;
    }
  }
  PRINT_DBG("Current device %02x:%02x:%02x:%02x:%02x:%02x not yet saved (%d)\n",
         addr[5], addr[4], addr[3], addr[2], addr[1], addr[0],i);
  return FALSE;
}

/**
 * @brief  Search for an already scanned device
 * @param  address of the device to search for
 * @retval TRUE if the device has been already scanned, FALSE otherwise
 */
uint8_t Is_Device_Scanned(tBDAddr addr)
{
  uint8_t i;

  for (i=0; i<non_conn_devices.dev_num; i++) {
    PRINT_DBG("Scanned device %d: %02x:%02x:%02x:%02x:%02x:%02x\n", i,
           non_conn_devices.bdaddr[i][5], non_conn_devices.bdaddr[i][4],
           non_conn_devices.bdaddr[i][3], non_conn_devices.bdaddr[i][2],
           non_conn_devices.bdaddr[i][1], non_conn_devices.bdaddr[i][0]);
    if (BLUENRG_memcmp(addr, non_conn_devices.bdaddr[i], sizeof(non_conn_devices.bdaddr[i])) == 0)
    {
      PRINT_DBG("Current device %02x:%02x:%02x:%02x:%02x:%02x already scanned (at pos %d in the scanned device array)\n",
                 addr[5], addr[4], addr[3], addr[2], addr[1], addr[0], i);
      return TRUE;
    }
  }
  PRINT_DBG("Current device %02x:%02x:%02x:%02x:%02x:%02x not yet scanned (%d)\n",
         addr[5], addr[4], addr[3], addr[2], addr[1], addr[0],i);
  return FALSE;
}

/**
 * @brief  Translate the console input into an integer
 * @param  The console input
 * @retval The integer
 */
uint8_t Get_Index(uint8_t console_ch)
{
  uint8_t index = TYPED_ERROR_VALUE;

  switch (console_ch)
  {
  case ('0'):
    index = 0;
    break;
  case ('1'):
    index = 1;
    break;
  case ('2'):
    index = 2;
    break;
  case ('3'):
    index = 3;
    break;
  case ('4'):
    index = 4;
    break;
  case ('5'):
    index = 5;
    break;
  case ('6'):
    index = 6;
    break;
  case ('7'):
    index = 7;
    break;
  case ('8'):
    index = 8;
    break;
  case ('9'):
    index = 9;
    break;
  default:
    PRINT_DBG("\r Undefined input!\r\n");
    break;
  }
  return index;
}

/**
 * @brief  Callback processing the ACI events
 * @note   Inside this function each event must be identified and correctly
 *         parsed
 * @param  void* Pointer to the ACI packet
 * @retval None
 */
void APP_UserEvtRx(void *pData)
{
  uint32_t i;

  hci_spi_pckt *hci_pckt = (hci_spi_pckt *)pData;

  if(hci_pckt->type == HCI_EVENT_PKT)
  {
    hci_event_pckt *event_pckt = (hci_event_pckt*)hci_pckt->data;

    if(event_pckt->evt == EVT_LE_META_EVENT)
    {
      evt_le_meta_event *evt = (void *)event_pckt->data;

      for (i = 0; i < (sizeof(hci_le_meta_events_table)/sizeof(hci_le_meta_events_table_type)); i++)
      {
        if (evt->subevent == hci_le_meta_events_table[i].evt_code)
        {
          hci_le_meta_events_table[i].process((void *)evt->data);
        }
      }
    }
    else if(event_pckt->evt == EVT_VENDOR)
    {
      evt_blue_aci *blue_evt = (void*)event_pckt->data;

      for (i = 0; i < (sizeof(hci_vendor_specific_events_table)/sizeof(hci_vendor_specific_events_table_type)); i++)
      {
        if (blue_evt->ecode == hci_vendor_specific_events_table[i].evt_code)
        {
          hci_vendor_specific_events_table[i].process((void *)blue_evt->data);
        }
      }
    }
    else
    {
      for (i = 0; i < (sizeof(hci_events_table)/sizeof(hci_events_table_type)); i++)
      {
        if (event_pckt->evt == hci_events_table[i].evt_code)
        {
          hci_events_table[i].process((void *)event_pckt->data);
        }
      }
    }
  }
}
