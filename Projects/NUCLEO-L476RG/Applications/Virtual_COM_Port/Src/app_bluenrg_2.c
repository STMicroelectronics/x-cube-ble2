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

#include "hci.h"
#include "hci_tl.h"
#include "stm32l4xx_nucleo.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private defines -----------------------------------------------------------*/
/* UART timeout values */
#define BLE_UART_SHORT_TIMEOUT      30
#define BLE_UART_LONG_TIMEOUT       300

/* HCI Packet types */
#define HCI_COMMAND_PKT             0x01
#define HCI_ACLDATA_PKT             0x02
#define HCI_SCODATA_PKT             0x03
#define HCI_EVENT_PKT               0x04
#define HCI_VENDOR_PKT              0xff

#define HCI_TYPE_OFFSET             0
#define HCI_VENDOR_CMDCODE_OFFSET   1
#define HCI_VENDOR_LEN_OFFSET       2
#define HCI_VENDOR_PARAM_OFFSET     4

#define FW_VERSION_MAJOR            1
#define FW_VERSION_MINOR            6

/* Commands */
#define VERSION                     0x01
#define EEPROM_READ                 0x02
#define EEPROM_WRITE                0x03
#define BLUENRG_RESET               0x04
#define HW_BOOTLOADER               0x05

#define MAX_RESP_SIZE               255

#define RESP_VENDOR_CODE_OFFSET     1
#define RESP_LEN_OFFSET_LSB         2
#define RESP_LEN_OFFSET_MSB         3
#define RESP_CMDCODE_OFFSET         4
#define RESP_STATUS_OFFSET          5
#define RESP_PARAM_OFFSET           6

/* Types of vendor codes */
#define ERROR                       0
/* Error codes */
#define UNKNOWN_COMMAND	            0x01
#define INVALID_PARAMETERS          0x12

#define RESPONSE                    1
/* end of vendor codes */

/* Size of Reception buffer */
#define UARTHEADERSIZE              4
#define RXBUFFERSIZE                255

/* Private macros ------------------------------------------------------------*/
#define PACK_2_BYTE_PARAMETER(ptr, param)  do{\
                *((uint8_t *)ptr) = (uint8_t)(param);   \
                *((uint8_t *)ptr+1) = (uint8_t)(param)>>8; \
                }while(0)

/* Private variables ---------------------------------------------------------*/
extern UART_HandleTypeDef hcom_uart[COMn];
/* Buffer used for reception */
uint8_t uart_header[UARTHEADERSIZE];
uint8_t aRxBuffer[RXBUFFERSIZE];

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void Hal_Write_Serial(const void* data1, const void* data2, int32_t n_bytes1,
                      int32_t n_bytes2);
void BlueNRG_HW_Bootloader(void);
void handle_vendor_command(uint8_t* cmd, uint8_t cmd_len);
static void User_Init(void);
static void User_Process(void);

static void Enable_SPI_IRQ(void);
static void Disable_SPI_IRQ(void);
static void set_irq_as_input(void);
static void set_irq_as_output(void);

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

  User_Init();

  hci_init(NULL, NULL);

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
 * @brief  Initialize User process.
 *
 * @param  None
 * @retval None
 */
static void User_Init(void)
{
  BSP_COM_Init(COM1);
}

/**
 * @brief  Manage the communication with the BlueNRG2 via UART and SPI.
 *
 * @param  None
 * @retval None
 */
static void User_Process(void)
{
  HAL_StatusTypeDef status;
  uint8_t len;
  int i;

  /* Read uart header */
  status = HAL_UART_Receive(&hcom_uart[COM1], (uint8_t *)uart_header, UARTHEADERSIZE, BLE_UART_SHORT_TIMEOUT);
  if (status != HAL_OK && status != HAL_TIMEOUT)
  {
    Error_Handler();
  }

  if (status == HAL_OK) {
    //We actually received data from the GUI
    PRINT_DBG("From GUI ");

    len = uart_header[UARTHEADERSIZE-1];

    if (len > 0) {
      /*## Put UART peripheral in reception process ###########################*/
      /* Any data received will be stored "aRxBuffer" buffer */
      if(HAL_UART_Receive(&hcom_uart[COM1], (uint8_t *)aRxBuffer, len, BLE_UART_LONG_TIMEOUT) != HAL_OK)
      {
        Error_Handler();
      }
    }

#if PRINT_CSV_FORMAT
    print_csv_time();
    for (i = 0; i < UARTHEADERSIZE; i++) {
      PRINT_CSV(" %02x", uart_header[i]);
    }
    if (len > 0) {
      for (i = 0; i < len; i++) {
        PRINT_CSV(" %02x", aRxBuffer[i]);
      }
    }
    PRINT_CSV("\n");
#endif

    /* write data received from the vcom to the BlueNRG chip via SPI */
    if (uart_header[HCI_TYPE_OFFSET] == HCI_COMMAND_PKT) {
      //This is an HCI command so pass it to BlueNRG via SPI
      Hal_Write_Serial(uart_header, aRxBuffer, UARTHEADERSIZE, len);
    } else {
      //Process the command locally without bothering with the BlueNRG chip
      handle_vendor_command(uart_header, UARTHEADERSIZE);
    }
  }

  while (HAL_GPIO_ReadPin(HCI_TL_SPI_EXTI_PORT, HCI_TL_SPI_EXTI_PIN) == GPIO_PIN_SET) {
    uint8_t rx_buffer[255];
    uint8_t rx_bytes;

    /* Data are available from BlueNRG: read them through SPI */
    rx_bytes = HCI_TL_SPI_Receive(rx_buffer, sizeof(rx_buffer));

    /* Check if there is data is so, send it to VCOM */
    if (rx_bytes > 0) {

      PRINT_DBG("From BlueNRG to GUI ");

#if PRINT_CSV_FORMAT
      print_csv_time();
      for (i = 0; i < rx_bytes; i++) {
        PRINT_CSV(" %02x", rx_buffer[i]);
      }
      PRINT_CSV("\n");
#endif

      for (i = 0; i < rx_bytes; i++) {
        if(HAL_UART_Transmit(&hcom_uart[COM1], (uint8_t*)&rx_buffer[i], 1, BLE_UART_LONG_TIMEOUT)!= HAL_OK) {
          Error_Handler();
        }
      }
    }
  }
}

/**
 * @brief  Handle vendor command
 * @param  cmd: the command
 * @param  cmd_len: the length of the command
 * @return None
 */
void handle_vendor_command(uint8_t* cmd, uint8_t cmd_len)
{
  int unsupported = 0;
  uint8_t len = 0;
  uint8_t response[MAX_RESP_SIZE];

  response[0] = HCI_VENDOR_PKT;
  response[RESP_VENDOR_CODE_OFFSET] = RESPONSE;
  response[RESP_CMDCODE_OFFSET] = cmd[HCI_VENDOR_CMDCODE_OFFSET];
  response[RESP_STATUS_OFFSET] = 0;

  if (cmd[HCI_TYPE_OFFSET] == HCI_VENDOR_PKT) {
    switch (cmd[HCI_VENDOR_CMDCODE_OFFSET]) {
    case VERSION:
      response[RESP_PARAM_OFFSET] = FW_VERSION_MAJOR;
      response[RESP_PARAM_OFFSET+1] = FW_VERSION_MINOR;
      len = 2;
      break;

    case BLUENRG_RESET:
      HCI_TL_SPI_Reset();
      break;

    case HW_BOOTLOADER:
      BlueNRG_HW_Bootloader();
      break;

    default:
      unsupported = 1;
    }
  } else {
    unsupported = 1;
  }

  if (unsupported) {
    response[RESP_STATUS_OFFSET] = UNKNOWN_COMMAND;
  }

  len += 2; //Status and Command code
  PACK_2_BYTE_PARAMETER(response + RESP_LEN_OFFSET_LSB, len);
  len += RESP_CMDCODE_OFFSET;

  PRINT_DBG("From Nucleo to GUI ");
#if PRINT_CSV_FORMAT
  print_csv_time();
  for (int i = 0; i < len; i++) {
    PRINT_CSV(" %02x", response[i]);
  }
  PRINT_CSV("\n");
#endif

  if(HAL_UART_Transmit(&hcom_uart[COM1], (uint8_t*)response, len, BLE_UART_LONG_TIMEOUT) != HAL_OK) {
    Error_Handler();
  }
}

/**
 * @brief  Writes data to a serial interface.
 * @param  data1   :  1st buffer
 * @param  data2   :  2nd buffer
 * @param  n_bytes1: number of bytes in 1st buffer
 * @param  n_bytes2: number of bytes in 2nd buffer
 * @retval None
 */
void Hal_Write_Serial(const void* data1, const void* data2, int32_t n_bytes1,
                      int32_t n_bytes2)
{
  uint8_t buff[UARTHEADERSIZE + RXBUFFERSIZE];
  uint32_t tickstart;

  BLUENRG_memcpy(buff, (uint8_t *)data1, n_bytes1);
  BLUENRG_memcpy(buff + n_bytes1, (uint8_t *)data2, n_bytes2);

  tickstart = HAL_GetTick();
  while(1){
    /* Data are available for the BlueNRG: write them through SPI */
    if (HCI_TL_SPI_Send(buff, n_bytes1+n_bytes2) == 0) break;
    if ((HAL_GetTick() - tickstart) > (HCI_DEFAULT_TIMEOUT_MS/10)) break;
  }
}

/**
 * @brief  Activate internal bootloader using pin.
 * @param  None
 * @retval None
 */
void BlueNRG_HW_Bootloader(void)
{
  Disable_SPI_IRQ();
  set_irq_as_output();

  HCI_TL_SPI_Reset();

  set_irq_as_input();
  Enable_SPI_IRQ();
}

/**
 * @brief  Enable SPI IRQ.
 * @param  None
 * @retval None
 */
void Enable_SPI_IRQ(void)
{
  HAL_NVIC_EnableIRQ(HCI_TL_SPI_EXTI_IRQn);
}

/**
 * @brief  Disable SPI IRQ.
 * @param  None
 * @retval None
 */
void Disable_SPI_IRQ(void)
{
  HAL_NVIC_DisableIRQ(HCI_TL_SPI_EXTI_IRQn);
}

/**
 * @brief  Set in Output mode the IRQ.
 * @param  None
 * @retval None
 */
void set_irq_as_output(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;

  /* Pull IRQ high */
  GPIO_InitStructure.Pin   = HCI_TL_SPI_IRQ_PIN;
  GPIO_InitStructure.Mode  = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStructure.Pull  = GPIO_NOPULL;
  HAL_GPIO_Init(HCI_TL_SPI_IRQ_PORT, &GPIO_InitStructure);
  HAL_GPIO_WritePin(HCI_TL_SPI_IRQ_PORT, HCI_TL_SPI_IRQ_PIN, GPIO_PIN_SET);
}

/**
 * @brief  Set the IRQ in input mode.
 * @param  None
 * @retval None
 */
void set_irq_as_input(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;

  /* IRQ input */
  GPIO_InitStructure.Pin       = HCI_TL_SPI_IRQ_PIN;
  GPIO_InitStructure.Mode      = GPIO_MODE_IT_RISING;
  GPIO_InitStructure.Pull      = GPIO_PULLDOWN;
  GPIO_InitStructure.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
#ifdef IS_GPIO_AF
  GPIO_InitStructure.Alternate = 0;
#endif
  HAL_GPIO_Init(HCI_TL_SPI_IRQ_PORT, &GPIO_InitStructure);

  GPIO_InitStructure.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(HCI_TL_SPI_IRQ_PORT, &GPIO_InitStructure);
}

