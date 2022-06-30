/**
  ******************************************************************************
  * @file    console.h
  * @author  SRA Application Team
  * @brief   Header file for console.c
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

#ifndef CONSOLE_H
#define CONSOLE_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>

/* Exported defines ----------------------------------------------------------*/
#define VERY_SHORT_TIMEOUT         100

#define ANSI_COLOR_RED      "\033[31m"
#define ANSI_COLOR_GREEN    "\033[32m"
#define ANSI_COLOR_YELLOW   "\033[33m"
#define ANSI_COLOR_BLUE     "\033[34m"
#define ANSI_COLOR_MAGENTA  "\033[35m"
#define ANSI_COLOR_CYAN     "\033[36m"
#define ANSI_COLOR_WHITE    "\033[37m"
#define ANSI_COLOR_RESET    "\033[0m"

#define ANSI_CLEAR_SCREEN   "\033[2J"
#define ANSI_CURSOR_TO_HOME "\033[H"

#define NOT_ALLOWED_CH                  'n'
#define SCAN_CH_UPPER                   'S'
#define SCAN_CH_LOWER                   's'
#define CLOSE_CONNECTION_CH_UPPER       'C'
#define CLOSE_CONNECTION_CH_LOWER       'c'
#define DISABLE_NOTIFICATIONS_CH_UPPER  'D'
#define DISABLE_NOTIFICATIONS_CH_LOWER  'd'
#define PRINT_DEVICE_INFO_CH_UPPER      'P'
#define PRINT_DEVICE_INFO_CH_LOWER      'p'

/* Exported types ------------------------------------------------------------*/
/* Exported function prototypes ----------------------------------------------*/
void    Main_Menu(void);
uint8_t Get_Value(uint8_t* console_ch);
int     Uart_Receive_Char(void);
int     Uart_Receive_Char_Timeout(int timeout);

#ifdef __cplusplus
}
#endif

#endif /* CONSOLE_H */
