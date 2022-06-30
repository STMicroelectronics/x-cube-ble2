/**
  ******************************************************************************
  * @file    target_platform.h
  * @author  SRA Application Team
  * @brief   Header file for target_platform.c
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
#ifndef TARGET_PLATFORM_H
#define TARGET_PLATFORM_H

#ifdef __cplusplus
 extern "C" {
#endif

#ifdef STM32L476xx

/* Includes ------------------------------------------------------------------*/
#include <stdio.h>
#include <stdlib.h>

#include "stm32l4xx_hal.h"
#include "stm32l4xx_nucleo.h"
#include "stm32l4xx_hal_conf.h"

/* Exported defines ----------------------------------------------------------*/
/* Exported macros -----------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/
uint32_t GetPage(uint32_t Address);
uint32_t GetBank(uint32_t Address);

#endif /* STM32L476xx */

#ifdef __cplusplus
}
#endif

#endif /* TARGET_PLATFORM_H */
