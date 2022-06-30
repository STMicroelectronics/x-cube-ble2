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
#define BLUENRG2_DEVICE

/**
 * Enable this #define if you want to change the BlueNRG IFR configuration.
 * IFR parameters can be edited in file Middlewares\ST\BlueNRG-2\hci\bluenrg1_devConfig.c
 * Disable this define if you want to update the BlueNRG fw stack.
*/
//#define BLUENRG_DEV_CONFIG_UPDATER

#ifndef BLUENRG_DEV_CONFIG_UPDATER
  /**
   * Update the DTM image using the FW_IMAGE provided in the file update_fw_image.c.
   * Disable this define if you want to update the BlueNRG fw stack through YMODEM
   * You can use this mode when using STM32 MCUs with small flash size (i.e.
   * when you cannot use the BLUENRG_STACK_UPDATER mode).
   */
  //#define BLUENRG_STACK_UPDATER

  #ifndef BLUENRG_STACK_UPDATER
    /**
     * Update the DTM image using a serial communication SW on PC
     * (as HyperTerminal or TeraTerm).
     * The DTM image to transfer, usually named DTM_SPI_NOUPDATER.bin, can be found
     * in the ST BlueNRG GUI installation folder
     * (usually C:\Users\<user name>\ST\BlueNRG GUI X.Y.Z\)
     * at location Firmware\BlueNRG2\DTM.
     */
    #define BLUENRG_STACK_UPDATER_YMODEM
  #endif

#endif

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
