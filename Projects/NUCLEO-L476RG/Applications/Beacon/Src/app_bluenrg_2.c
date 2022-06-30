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

#include "hci.h"
#include "bluenrg1_types.h"
#include "bluenrg1_gap.h"
#include "bluenrg1_aci.h"
#include "bluenrg1_hci_le.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/
/* Set to 1 for enabling Flags AD Type position at the beginning
   of the advertising packet */
#define ENABLE_FLAGS_AD_TYPE_AT_BEGINNING 1

#if (ADV_DATA_TYPE != ADV_NONCONN_IND)
   #warning "The advertising data type should be of 'non-connectable and no scan response' type."
#endif
#if (ADV_INTERV_MIN > ADV_INTERV_MAX)
   #error "The min advertising interval cannot be greater than the max advertising interval."
#endif
#if ((ADV_INTERV_MIN <= 0) || (ADV_INTERV_MIN > 40959))
  #error "Invalid min advertising interval! Please select a value between 0 and 40959 ms."
#endif
#if ((ADV_INTERV_MAX <= 0) || (ADV_INTERV_MAX > 40959))
  #error "Invalid max advertising interval! Please select a value between 0 and 40959 ms."
#endif

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
static void User_Init(void);
static void User_Process(void);
static void Device_Init(void);
static void Start_Beaconing(void);

void EnterStopMode(void);

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
  User_Init();

  hci_init(NULL, NULL);

  /* Init the Device */
  Device_Init();

  /* Start Beacon Non Connectable Mode*/
  Start_Beaconing();

  PRINT_DBG("BlueNRG-2 BLE Beacon Application\r\n");

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
 * @brief  User application process
 * @param  None
 * @retval None
 */
static void User_Process(void)
{
  EnterStopMode();
}

static void Device_Init(void)
{
  uint8_t ret;
  uint16_t service_handle;
  uint16_t dev_name_char_handle;
  uint16_t appearance_char_handle;
  uint8_t bdaddr[] = {0xf5, 0x00, 0x00, 0xE1, 0x80, 0x02};

  /* Sw reset of the device */
  hci_reset();
  /**
   *  To support both the BlueNRG-2 and the BlueNRG-2N a minimum delay of 2000ms is required at device boot
   */
  HAL_Delay(2000);

  /* Set the TX Power to -2 dBm */
  ret = aci_hal_set_tx_power_level(1,4);
  if(ret != 0) {
    PRINT_DBG ("Error in aci_hal_set_tx_power_level() 0x%04x\r\n", ret);
    while(1);
  }
  else
    PRINT_DBG ("aci_hal_set_tx_power_level() --> SUCCESS\r\n");

  /* Set the public address */
  ret = aci_hal_write_config_data(CONFIG_DATA_PUBADDR_OFFSET, CONFIG_DATA_PUBADDR_LEN, bdaddr);
  if(ret != 0) {
    PRINT_DBG ("Error in aci_hal_write_config_data() 0x%04x\r\n", ret);
    while(1);
  }
  else
    PRINT_DBG ("aci_hal_write_config_data() --> SUCCESS\r\n");

  /* Init the GATT */
  ret = aci_gatt_init();
  if(ret != 0) {
    PRINT_DBG ("Error in aci_gatt_init() 0x%04x\r\n", ret);
    while(1);
  }
  else
    PRINT_DBG ("aci_gatt_init() --> SUCCESS\r\n");

  /* Init the GAP */
  ret = aci_gap_init(GAP_PERIPHERAL_ROLE, 0, 0x08, &service_handle, &dev_name_char_handle, &appearance_char_handle);
  if(ret != 0) {
    PRINT_DBG ("Error in aci_gap_init() 0x%04x\r\n", ret);
    while(1);
  }
  else
    PRINT_DBG ("aci_gap_init() --> SUCCESS\r\n");
}

/**
* @brief  Start beaconing
* @param  None
* @retval None
*/
static void Start_Beaconing(void)
{
  uint8_t ret = BLE_STATUS_SUCCESS;

#if ENABLE_FLAGS_AD_TYPE_AT_BEGINNING
  /* Set AD Type Flags at beginning on Advertising packet  */
  const uint8_t adv_data[] = {
    /* Advertising data: Flags AD Type */
    0x02,
    0x01,
    0x06,
    /* Advertising data: manufacturer specific data */
    26, AD_TYPE_MANUFACTURER_SPECIFIC_DATA,
    0x30, 0x00, //Company identifier code (Default is 0x0030 - STMicroelectronics: To be customized for specific identifier)
    0x02,       // ID
    0x15,       //Length of the remaining payload
    0xE2, 0x0A, 0x39, 0xF4, 0x73, 0xF5, 0x4B, 0xC4, //Location UUID
    0xA1, 0x2F, 0x17, 0xD1, 0xAD, 0x07, 0xA9, 0x61,
    0x00, 0x00, // Major number
    0x00, 0x00, // Minor number
    0xC8        //2's complement of the Tx power (-56dB)};
  };
#else
  const uint8_t manuf_data[] = {26, AD_TYPE_MANUFACTURER_SPECIFIC_DATA,
  0x30, 0x00, //Company identifier code (Default is 0x0030 - STMicroelectronics: To be customized for specific identifier)
  0x02,       // ID
  0x15,       //Length of the remaining payload
  0xE2, 0x0A, 0x39, 0xF4, 0x73, 0xF5, 0x4B, 0xC4, //Location UUID
  0xA1, 0x2F, 0x17, 0xD1, 0xAD, 0x07, 0xA9, 0x61,
  0x00, 0x00, // Major number
  0x00, 0x00, // Minor number
  0xC8        //2's complement of the Tx power (-56dB)};
  };
#endif

  /* disable scan response */
  ret = hci_le_set_scan_response_data(0,NULL);
  if (ret != BLE_STATUS_SUCCESS) {
    PRINT_DBG ("Error in hci_le_set_scan_resp_data() 0x%04x\r\n", ret);
    while(1);
  }
  else
    PRINT_DBG ("hci_le_set_scan_resp_data() --> SUCCESS\r\n");

  /* put device in non connectable mode */
  ret = aci_gap_set_discoverable(ADV_DATA_TYPE,              /*< Advertise as non-connectable, undirected. */
                                 ADV_INTERV_MIN,             /*< Set the min advertising interval (0.625 us increment). */
                                 ADV_INTERV_MAX,             /*< Set the max advertising interval (0.625 us increment). */
                                 PUBLIC_ADDR, NO_WHITE_LIST_USE, /*< Use the public address, with no white list. */
                                 0, NULL,                    /*< Do not use a local name. */
                                 0, NULL,                    /*< Do not include the service UUID list. */
                                 0, 0);                      /*< Do not set a slave connection interval. */

  if (ret != BLE_STATUS_SUCCESS) {
    PRINT_DBG ("Error in aci_gap_set_discoverable() 0x%04x\r\n", ret);
    while(1);
  }
  else
    PRINT_DBG ("aci_gap_set_discoverable() --> SUCCESS\r\n");

#if ENABLE_FLAGS_AD_TYPE_AT_BEGINNING
  /* Set the  ADV data with the Flags AD Type at beginning of the
  advertsing packet,  followed by the beacon manufacturer specific data */
  ret = hci_le_set_advertising_data (sizeof(adv_data), (uint8_t *)adv_data);
  if (ret != BLE_STATUS_SUCCESS) {
    PRINT_DBG ("Error in hci_le_set_advertising_data() 0x%04x\r\n", ret);
    while(1);
  }
  else
    PRINT_DBG ("hci_le_set_advertising_data() --> SUCCESS\r\n");
#else
  ret = aci_gap_delete_ad_type(AD_TYPE_TX_POWER_LEVEL);
  if (ret != BLE_STATUS_SUCCESS) {
    PRINT_DBG ("Error in aci_gap_delete_ad_type() 0x%04x\r\n", ret);
    while(1);
  }
  else
    PRINT_DBG ("aci_gap_delete_ad_type() --> SUCCESS\r\n");

  ret = aci_gap_update_adv_data(sizeof(manuf_data), (uint8_t *)manuf_data);
  if (ret != BLE_STATUS_SUCCESS) {
    PRINT_DBG ("Error in aci_gap_update_adv_data() 0x%04x\r\n", ret);
    while(1);
  }
  else
    PRINT_DBG ("aci_gap_update_adv_data() --> SUCCESS\r\n");
#endif
}

/**
 * @brief  Enter the STOP mode
 * @param  None
 * @retval None
 */
void EnterStopMode(void)
{
  __HAL_PWR_CLEAR_FLAG(PWR_FLAG_WU);
  HAL_PWR_EnterSTOPMode(PWR_LOWPOWERREGULATOR_ON, PWR_STOPENTRY_WFI);
}
