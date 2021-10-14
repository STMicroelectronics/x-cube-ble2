#!/bin/bash
#/**
#  ******************************************************************************
#  * @file    GenSensorDemoBin_STM32CubeIDE_L476.sh
#  * @author  SRA Application Team
#  * @brief   Script for adding the bootloader part to a binary file
#  ******************************************************************************
#  * @attention
#  *
#  * Copyright (c) 2020 STMicroelectronics.
#  * All rights reserved.
#  *
#  * This software is licensed under terms that can be found in the LICENSE file
#  * in the root directory of this software component.
#  * If no LICENSE file comes with this software, it is provided AS-IS.
#  *
#  ******************************************************************************
#  */

######## Modify this Section:
# 1) Set the Installation path for OpenOCD
# example:
#OpenOCD_DIR="C:/ST/STM32CubeIDE_1.4.0/STM32CubeIDE/plugins/com.st.stm32cube.ide.mcu.externaltools.openocd.win32_1.4.0.202007081208/tools/"
OpenOCD_DIR=""

# 2) Set the installation path for stm32 OpenOCD scritps
# example:
#OpenOCD_CFC="C:/ST/STM32CubeIDE_1.4.0/STM32CubeIDE/plugins/com.st.stm32cube.ide.mcu.debug.openocd_1.4.2.202008061104/resources/openocd/st_scripts"
OpenOCD_CFC=""

# 3) Only for Linux/iOS add openocd library path to _LIBRARY_PATH:
# For iOS example:
#export DYLD_LIBRARY_PATH=${DYLD_LIBRARY_PATH}:${OpenOCD_DIR}"lib/"

# For Linux example:
#export LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:${OpenOCD_DIR}"lib/"

# 4) Set the application name
# example:
#APPNAME="SensorDemo_BLESensor-App"
APPNAME="SensorDemo_BLESensor-App"

######## Don't change the following part

## Control Section

if [[ ! $OpenOCD_DIR ]];  then
	echo "Please add the rigth path to OpenOCD_DIR Variable"
	exit
fi

if [[ ! $OpenOCD_CFC ]];  then
	echo "Please add the rigth path to OpenOCD_CFC Variable"
	exit
fi


## Run section

# Board type
BOARDNAME="nucleo_l476rg"

# OpenOCD command
OpenOCD_CMD="${OpenOCD_DIR}/bin/openocd -s ${OpenOCD_CFC} -f nucleo_l476rg.cfg"


echo "/******************************************/"
echo "       Clean SensorDemo_BLESensor-App"
echo "/******************************************/"
echo "             Full Chip Erase"
echo "/******************************************/"
${OpenOCD_CMD} -c "init" -c "reset halt" -c "flash erase_sector 0 511 511" -c "shutdown"
echo "/******************************************/"
echo "              Install BootLoader"
echo "/******************************************/"
${OpenOCD_CMD} -c "init" -c "reset halt" -c "flash write_image erase ../../STM32L476RG_BL/BootLoaderL4.bin 0x08000000 bin" -c "shutdown"
echo "/******************************************/"
echo "      Install SensorDemo_BLESensor-App"
echo "/******************************************/"
${OpenOCD_CMD} -c "init" -c "reset halt" -c "flash write_image erase ../../../STM32CubeIDE/Release/${APPNAME}.bin 0x08004000 bin" -c "shutdown"
echo "/******************************************/"
echo " Dump SensorDemo_BLESensor-App + BootLoader"
echo "/******************************************/"

SizeBinBL=`ls -l ../../../STM32CubeIDE/Release/${APPNAME}.bin | awk '{print $5+0x4000};'`
${OpenOCD_CMD} -c "init" \
			   -c "reset halt" \
			   -c "dump_image ../../../STM32CubeIDE/Release/${APPNAME}_BL.bin 0x08000000 ${SizeBinBL}" \
			   -c "shutdown"

