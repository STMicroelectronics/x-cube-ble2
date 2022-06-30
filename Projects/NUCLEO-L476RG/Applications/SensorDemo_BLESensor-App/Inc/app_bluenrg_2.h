/**
  ******************************************************************************
  * @file    app_bluenrg_2.h
  * @author  SRA Application Team
  * @brief   Header file for app_bluenrg_2.c
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef APP_BLUENRG_2_H
#define APP_BLUENRG_2_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32l4xx_nucleo.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported Defines ----------------------------------------------------------*/
/*************** Don't Change the following defines *************/

/* Package Version only numbers 0->9 */
#define PACK_VERSION_MAJOR '3'
#define PACK_VERSION_MINOR '2'
#define PACK_VERSION_PATCH '2'

/* Define the application Name (MUST be 7 char long) */
#define APP_NAME 'S','D','E','M',PACK_VERSION_MAJOR,PACK_VERSION_MINOR,PACK_VERSION_PATCH

/* Package Name */
#define BLUENRG_PACKAGENAME "X-CUBE-BLE2"

/* USER CODE BEGIN ED */

/* USER CODE END ED */

/* Exported Variables --------------------------------------------------------*/

/* USER CODE BEGIN EV */

/* USER CODE END EV */

/* Exported Functions Prototypes ---------------------------------------------*/
void MX_BlueNRG_2_Init(void);
void MX_BlueNRG_2_Process(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

#ifdef __cplusplus
}
#endif
#endif /* APP_BLUENRG_2_H */
