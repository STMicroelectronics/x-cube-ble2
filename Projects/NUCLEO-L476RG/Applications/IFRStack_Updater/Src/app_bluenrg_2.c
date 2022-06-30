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

#include <stdlib.h>
#include <stdio.h>
#include "bluenrg1_aci.h"
#include "bluenrg1_hci_le.h"
#include "bluenrg1_events.h"
#include "hci_tl.h"
#include "hci.h"
#include "bluenrg_utils.h"
#include "stm32l4xx_nucleo.h"

#ifdef BLUENRG_STACK_UPDATER_YMODEM
  #include "ymodem.h"
#endif /* BLUENRG_STACK_UPDATER_YMODEM */

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private defines -----------------------------------------------------------*/
/* Private macros ------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
extern const devConfig_t deviceConfig;
#ifdef BLUENRG_STACK_UPDATER
  extern const unsigned char FW_IMAGE[];
  extern unsigned int FW_IMAGE_SIZE;
#endif

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
static void User_Process(void);
static void User_Init(void);
void APP_UserEvtRx(void *pData);
#ifdef BLUENRG_STACK_UPDATER_YMODEM
  extern int32_t program_device_UART(void);
  extern void processInputData(uint8_t* data_buffer, uint16_t Nb_bytes);
#endif /* BLUENRG_STACK_UPDATER_YMODEM */

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

  /* LED ON for initial status */
  BSP_LED_On(LED2);

  hci_init(APP_UserEvtRx, NULL);

  printf("BlueNRG-2 IFR/Stack Updater.\r\n");

#if defined (BLUENRG_DEV_CONFIG_UPDATER)
  /* Verify if the DEV_CONFIG memory and then update it if needed */
  printf("Verify IFR configuration...\r\n");
  ret = verify_DEV_CONFIG(&deviceConfig);

  if (ret) {
    printf("Start IFR updater...\r\n");
    ret = program_DEV_CONFIG(&deviceConfig);
    if (ret) {
      printf("... error.\r\n\n");
      while(1) {
        BSP_LED_Toggle(LED2);
        HAL_Delay(3000);
      }
    }
  }
#endif /* BLUENRG_DEV_CONFIG_UPDATER */

#if defined (BLUENRG_STACK_UPDATER)
  /* Update the DTM image using the FW_IMAGE provided in the file update_fw_image.c */
  printf("Start Stack updater...\r\n");
  ret = program_device(FW_IMAGE, FW_IMAGE_SIZE);
  if (ret) {
    printf("... error.\r\n\n");
    while(1) {
      BSP_LED_Toggle(LED2);
      HAL_Delay(3000);
    }
  }
#endif /* BLUENRG_STACK_UPDATER */

#if defined (BLUENRG_STACK_UPDATER_YMODEM)
  /* Update the BlueNRG-2 image through PC using serial terminal application */
  printf("Start Stack updater using YMODEM...\r\n");

  /* Enable IRQ for reception  */
  __HAL_UART_ENABLE_IT(&hcom_uart[COM1], UART_IT_RXNE);

  Ymodem_Init();

  ret = program_device_UART();
  if (ret) {
    printf("... error.\r\n\n");
    while(1) {
      BSP_LED_Toggle(LED2);
      HAL_Delay(3000);
    }
  }
#endif /* BLUENRG_STACK_UPDATER_YMODEM */

  printf("... end.\r\n\n");

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

  User_Process();

  /* USER CODE BEGIN BlueNRG_2_Process_PostTreatment */

  /* USER CODE END BlueNRG_2_Process_PostTreatment */
}

/**
 * @brief  Initialize User process
 *
 * @param  None
 * @retval None
 */
static void User_Init(void)
{
  BSP_COM_Init(COM1);

  BSP_LED_Init(LED2);
}

/**
 * @brief  User Process
 *
 * @param  None
 * @retval None
 */
static void User_Process(void)
{
  BSP_LED_Toggle(LED2);
  HAL_Delay(500);
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

#ifdef BLUENRG_STACK_UPDATER_YMODEM
/**
  * @brief This function handles USART2 global interrupt.
  */
void USART2_IRQHandler(void)
{
  uint8_t tmp_rx_data = 0;

  if (__HAL_UART_GET_FLAG(&hcom_uart[COM1], UART_FLAG_RXNE) == 1) {

    HAL_UART_Receive(&hcom_uart[COM1], &tmp_rx_data, 1, 1000);

    processInputData(&tmp_rx_data, 1);
  }
  if (__HAL_UART_GET_FLAG(&hcom_uart[COM1], UART_FLAG_ORE) == 1) {
    __HAL_UART_CLEAR_OREFLAG(&hcom_uart[COM1]);
  }
}
#endif /* BLUENRG_STACK_UPDATER_YMODEM */
