/**
  @page readme.txt
  
  @verbatim
  ******************************************************************************
  * @file    readme.txt
  * @author  SRA Application Team
  * @brief   Bootloader for STM32L476RG and script files for binary flashing.
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
  @endverbatim

Folders included in BootLoader.zip archive

 -------------------------
| GenSensorDemoBin_Script |
 -------------------------
This folder contains the script files (.bat/.sh), for all the supported IDEs (EWARM, MDK-ARM 
and STM32CubeIDE), for installing the BootLoader and flashing the SensorDemo_BLESensor-App on a 
STM32L476RG MCU.
These scripts also create a *_BL.bin file for STM32L476RG MCU composed by the BootLoader and the 
SensorDemo_BLESensor-App.

To easily use these scripts, extract the BootLoader.zip archive and copy the BootLoader folder in 
the root folder of your SensorDemo_BLESensor-App sample application.
Before launching the script for your IDE, open it and set the correct paths and filenames.


 ----------------
| STM32L476RG_BL |
 ----------------
The firmware contained in this folder is the Boot Loader that must be flashed at the 
Flash start address (0x08000000) and has the purpose to apply and run the Firmware-Over-The-Air 
(FOTA) receveid. 
It runs the program normally if no update is received.

For STM32L476RG (Nucleo or SensorTile [STEVAL-STLCS01V1]) the 1Mbytes-Flash is divided in 
2 512Kbytes-banks. Each bank is split in 256 2Kbytes-pages.

|Page 0->7  | Page 4->255       | Page 256->511 |
|16K        | 504K              | 512K          |
|0x08000000 |0x08004000         |0x08080000     |
|-----------------------------------------------|
|  Reg1     |       Reg2        |    Reg3       |
|BootLoader | Running Program   |    FOTA       |


This Boot Loader must be Loaded on Sector 0. It checks if there is a FOTA stored (0x08080000 address):
- If YES:
  - it erases the necessary space on Region 2 (Reg2)
  - it copies the FOTA on that sectors
  - it erases the FOTA region (Reg3) after the update.
- if NOT:
  - it works like a trampoline for executing the normal program stored from Region 2 (0x08004000 address)

The FOTA must be less than 504Kbytes

The "Running Program" and the FOTA are compiled for running from 0x08004000 address. 
If they are placed at the beginning of the FLASH they doesn't work.
Also, without the BootLoader that programs could not be executed.


-------------------------------------------------------------------------------------------------------------
Hereafter some information related to other STM32 series are provided (even if not supported in this package)
-------------------------------------------------------------------------------------------------------------
- For STM32F401RE (Nucleo) or STM32F446RE (Nucleo or BlueCoin) the 512Kbyte-Flash are split in 8 sectors:

  |sector 0   | sector 1  | sector 2  | sector 3  | sector 4  | sector 5  | sector 6  | sector 7  |
  |16K        | 16K       | 16K       | 16K       | 64K       | 128K      | 128K      | 128K      |
  |0x08000000 |0x08004000 |0x08008000 |0x0800C000 |0x08010000 |0x08020000 |0x08040000 |0x08060000 |
  |-----------------------------------------------------------------------------------------------|
  |BootLoader | Running Program                                           |  FOTA                 |

  This Boot Loader must be Loaded in Sector 0. It checks if there is a FOTA stored in Sector 6 (0x08040000 address):
  - If YES:
    - it erases the sectors from 1 to 5
    - it copies the FOTA on that sectors
    - it erases the FOTA region after the update.
  - if NOT:
    - it works like a trampoline for executing the normal program stored from sector1 (0x08004000 address)

  The FOTA must be less than 240Kbytes

- For STM32L4R9ZI the 2Mbytes-Flash is Divided in 2 1Mbytes-banks. Each bank is split in 256 4Kbytes-pages.

  |Page 0->7  | Page 4->255       | Page 256->511 |
  |16K        | 1008K             | 1024K         |
  |0x08000000 |0x08004000         |0x08100000     |
  |-----------------------------------------------|
  |  Reg1     |       Reg2        |    Reg3       |
  |BootLoader | Running Program   |    FOTA       |

  This Boot Loader must be Loaded in Sector 0. It checks if there is a FOTA stored (0x08100000 address):
  - If YES:
    - it erases the necessary space on Region 2 (Reg2)
    - it copies the FOTA on that sectors
    - it erases the FOTA region (Reg3) after the update.
  - if NOT:
    - it works like a trampoline for executing the normal program stored from Region 2 (0x08004000 address)

  The FOTA must be less than 1008Kbytes
