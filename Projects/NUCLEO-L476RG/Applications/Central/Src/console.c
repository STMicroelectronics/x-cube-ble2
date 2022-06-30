/**
  ******************************************************************************
  * @file    console.c
  * @author  SRA Application Team
  * @brief   Manage the console
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
#include "console.h"
#include "central.h"
#include "stm32l4xx_nucleo.h"

/* Private typedef -----------------------------------------------------------*/
/* Private defines -----------------------------------------------------------*/
#define UartHandle hcom_uart[COM1]

/* Private macros ------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
extern savedDevices_t  saved_devices;
extern centralStatus_t central_status;

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/
/** @brief  Receives a character from serial port
 *  @param  None
 *  @retval Character received
 */
int Uart_Receive_Char(void)
{
  uint8_t ch;
  HAL_UART_Receive(&UartHandle, &ch, 1, HAL_MAX_DELAY);

  /* Echo character back to console */
  HAL_UART_Transmit(&UartHandle, &ch, 1, HAL_MAX_DELAY);

  /* And cope with Windows */
  if(ch == '\r'){
    uint8_t ret = '\n';
    HAL_UART_Transmit(&UartHandle, &ret, 1, HAL_MAX_DELAY);
  }

  return ch;
}

/** @brief  Receives a character from serial port
 *  @param  Timeout
 *  @retval Character received
 */
int Uart_Receive_Char_Timeout(int timeout)
{
  uint8_t ch;
  HAL_UART_Receive(&UartHandle, &ch, 1, timeout);

  /* And cope with Windows */
  if(ch == '\r'){
    uint8_t ret = '\n';
    HAL_UART_Transmit(&UartHandle, &ret, 1, HAL_MAX_DELAY);
  }

  return ch;
}

/**
 * @brief  Print available key options
 * @param  None
 * @retval None
 */
void Main_Menu(void)
{
  if ((central_status != SELECT_ANOTHER_CHARACTERISTIC) &&
      (central_status != RECEIVE_NOTIFICATIONS)) {
    printf("\n * *********************** MENU *********************** *\r\n");
    printf(" *                                                      *\r\n");

    switch (central_status)
    {
    case INIT_STATUS:
      printf(" * [S/s]   Scan the network                             *\n");
      break;
    case SELECT_DEVICE:
      printf(" * [S/s]   Scan the network                             *\n");
      printf(" * [0 - %d] Connect to a device                          *\n",
             saved_devices.dev_num-1);
      break;
    case SELECT_CHARACTERISTIC:
      printf(" * [x.y.z] Update the characteristic properties         *\n");
      printf(" *         - x = service index                          *\n");
      printf(" *         - y = characteristic index                   *\n");
      printf(" *         - z = characteristic property index          *\n");
      printf(" *         [D/d] Disable notifications                  *\n");
      printf(" * [P/p]   Print device info                            *\n");
      printf(" * [C/c]   Close connection                             *\n");
      break;
    default:
      break;
    }

    printf(" *                                                      *\r\n");
    printf(" * *********************** ---- *********************** *\r\n");
  }

  if (central_status != RECEIVE_NOTIFICATIONS) {
    printf("\r\n Type your choice..... ");
  }
  fflush(stdout);
}

/**
 * @brief  Get a string from console
 * @param  The string buffer
 * @retval The string length
 */
uint8_t Get_Value(uint8_t* console_ch)
{
  uint8_t i = 0;

  while (i < MAX_STRING_LENGTH) {
    console_ch[i] = Uart_Receive_Char();
    if (console_ch[i] == 0x0D) { /* hex for carriage return */
      break;
    }
    i++;
  }
  return i;
}
