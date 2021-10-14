---
pagetitle: Release Notes for X-CUBE-BLE2 Package
lang: en
header-includes: <link rel="icon" type="image/x-icon" href="_htmresc/favicon.png" />
---

::: {.row}
::: {.col-sm-12 .col-lg-4}

<center>
# Release Notes for <mark>X-CUBE-BLE2</mark>
Copyright &copy; 2021 STMicroelectronics\
    
[![ST logo](_htmresc/st_logo_2020.png)](https://www.st.com){.logo}
</center>

# License

This software package is
licensed by ST under ST license SLA0055, the "License"; You may not use this component except in compliance
with the License. You may obtain a copy of the License at:
[www.st.com/SLA0055](https://www.st.com/SLA0055)

# Purpose

The <mark>X-CUBE-BLE2</mark> is an expansion software package for STM32Cube. This software provides drivers running on STM32 for STM's BlueNRG-2 Bluetooth Low Energy device. This software package is built on top of STM32Cube software technology that ease portability across different STM32 microcontrollers. 

X-CUBE-BLE2 software features:

- Complete middleware to build applications using BlueNRG-2 network processor.
- Easy portability across different MCU families thanks to STM32Cube.
- Sample applications that the developer can use to start experimenting with the code.
- References to free Android and iOS Apps that can be used along with the sample applications.
- Free, user-friendly license terms.
- Examples implementation available for X-NUCLEO-BNRG2A1 STM32 expansion board plugged on top of one NUCLEO-L476RG.

The figure below shows the overall architecture.

[![X-CUBE-BLE2 Block Diagram](_htmresc/X-CUBE-BLE2_BlockDiagram_2020.png)]()

- At the bottom layer there are the HW components: the STM32 MCU and the BlueNRG-2 network processor.  
- The drivers abstract low level details of the hardware and allow the middleware software to provide Bluetooth LE features in a hardware independent fashion.
- The applications provide examples of how to use the code.

Related information and documentation:

- [UM2666](https://www.st.com/resource/en/user_manual/dm00672214-getting-started-with-the-xcubeble2-bluetooth-low-energy-software-expansion-for-stm32cube-stmicroelectronics.pdf): Getting started with the X-CUBE-BLE2 Bluetooth Low Energy software expansion for STM32Cube
- [STM32Cube](http://www.st.com/stm32cube)
- [STM32 Nucleo boards](http://www.st.com/stm32nucleo)
- [How to use the X-CUBE-BLE2 pack in the STM32CubeMX tool](https://www.youtube.com/watch?v=BzKIT2T5Q4c)

:::

::: {.col-sm-12 .col-lg-8}
# Update History

::: {.collapse}
<input type="checkbox" id="collapse-section7" checked aria-hidden="true">
<label for="collapse-section7" aria-hidden="true">V3.2.2 / 18-Oct-2021</label>
<div>			

## Main Changes

### Maintenance release

  Headline
  ----------------------------------------------------------
  Align to STMicroelectronics.X-CUBE-BLE2.3.2.2.pack for STM32CubeMX (minimum required version V6.3.0)
  Publish code on GitHub

## Contents
The X-CUBE-BLE2 expansion software package comes with a rich set of examples running on STMicroelectronics boards, organized by board and provided with preconfigured projects for the main supported toolchains.

<small>The components flagged by "[]{.icon-st-update}" have changed since the
previous release. "[]{.icon-st-add}" are new.</small>

Documentation

  Name                                                        Version                                           License                                                                                                       Document
  ----------------------------------------------------------- ------------------------------------------------- ------------------------------------------------------------------------------------------------------------- ------------------------------------------------------------------------------------------------------------------------------------------------
  X-CUBE-BLE2 API Description                                 V3.2.2 []{.icon-st-update}                        [SLA0055](https://www.st.com/SLA0055)                                                                         [CHM](Documentation\X-CUBE-BLE2.chm)
  Getting Started with X-CUBE-BLE2 pack for STM32CubeMX       V3.2.2 []{.icon-st-update}                        [SLA0055](https://www.st.com/SLA0055)                                                                         [PDF](Documentation\STMicroelectronics.X-CUBE-BLE2_GettingStarted.pdf)

Drivers

  Name                                                        Version                                           License                                                                                                       Release note
  ----------------------------------------------------------- ------------------------------------------------- ------------------------------------------------------------------------------------------------------------- ------------------------------------------------------------------------------------------------------------------------------------------------
  STM32L4xx CMSIS                                             V1.7.1                                            [Apache License v2.0](https://opensource.org/licenses/Apache-2.0)                                             [release notes](Drivers\CMSIS\Device\ST\STM32L4xx\Release_Notes.html)
  STM32L4xx HAL                                               V1.13.0                                           [BSD-3-Clause](https://opensource.org/licenses/BSD-3-Clause)                                                  [release notes](Drivers\STM32L4xx_HAL_Driver\Release_Notes.html)
  BSP STM32L4xx_Nucleo                                        V2.1.7                                            [BSD-3-Clause](https://opensource.org/licenses/BSD-3-Clause)                                                  [release notes](Drivers\BSP\STM32L4xx_Nucleo\Release_Notes.html)

Middlewares
  
  Name                                                        Version                                           License                                                                                                       Release note
  ----------------------------------------------------------- ------------------------------------------------- ------------------------------------------------------------------------------------------------------------- ------------------------------------------------------------------------------------------------------------------------------------------------
  BlueNRG-2                                                    V3.2.2 []{.icon-st-update}                       [SLA0055](https://www.st.com/SLA0055)                                                                         [release notes](Middlewares\ST\BlueNRG-2\Release_Notes.html)

Projects
  
  Name                                                        Version                                           License                                                                                                       Release note
  ----------------------------------------------------------- ------------------------------------------------- ------------------------------------------------------------------------------------------------------------- ------------------------------------------------------------------------------------------------------------------------------------------------
  Projects                                                    V3.2.2 []{.icon-st-update}                        [SLA0055](https://www.st.com/SLA0055)                                                                         [release notes](Projects\Release_Notes.html)

## Known Limitations

- To correctly set the RESET on pin D7 of the X-NUCLEO-BNRG2A1 expansion board, a 0 Ohm resistor must be soldiered on R117. Alternatively, the D7 pin of the Arduino connector and the pin #5 of the J12 must be bridged
- On dual-core STM32 series this expansion software can be used on both cores but exclusively 

## Development Toolchains and Compilers

- IAR Embedded Workbench for ARM (EWARM) toolchain V8.50.9
- RealView Microcontroller Development Kit (MDK-ARM) toolchain V5.32.0
- STM32CubeIDE V1.7.0

## Supported Devices and Boards

- STM32L476RG devices and STM32L476RG-Nucleo board RevB
- X-NUCLEO-BNRG2A1
- BlueNRG-2 and BlueNRG-2N Bluetooth® Low Energy devices

## Dependencies

None

</div>
:::

::: {.collapse}
<input type="checkbox" id="collapse-section6" aria-hidden="true">
<label for="collapse-section6" aria-hidden="true">V3.2.1 / 05-July-2021</label>
<div>			

## Main Changes

### Product update

  Headline
  ----------------------------------------------------------
  Align to STMicroelectronics.X-CUBE-BLE2.3.2.1.pack for STM32CubeMX (minimum required version V6.3.0)
  Fix SPI read and write operations in HCI TL Interface
  Fix Beacon sample application for STM32U5 series

## Contents
The X-CUBE-BLE2 expansion software package comes with a rich set of examples running on STMicroelectronics boards, organized by board and provided with preconfigured projects for the main supported toolchains.

<small>The components flagged by "[]{.icon-st-update}" have changed since the
previous release. "[]{.icon-st-add}" are new.</small>

Documentation

  Name                                                        Version                                           License                                                                                                       Document
  ----------------------------------------------------------- ------------------------------------------------- ------------------------------------------------------------------------------------------------------------- ------------------------------------------------------------------------------------------------------------------------------------------------
  X-CUBE-BLE2 API Description                                 V3.2.1 []{.icon-st-update}                        [SLA0055](https://www.st.com/SLA0055)                                                                         [CHM](Documentation\X-CUBE-BLE2.chm)
  Getting Started with X-CUBE-BLE2 pack for STM32CubeMX       V3.2.1 []{.icon-st-update}                        [SLA0055](https://www.st.com/SLA0055)                                                                         [PDF](Documentation\STMicroelectronics.X-CUBE-BLE2_GettingStarted.pdf)

Drivers

  Name                                                        Version                                           License                                                                                                       Release note
  ----------------------------------------------------------- ------------------------------------------------- ------------------------------------------------------------------------------------------------------------- ------------------------------------------------------------------------------------------------------------------------------------------------
  STM32L4xx CMSIS                                             V1.7.1                                            [Apache License v2.0](https://opensource.org/licenses/Apache-2.0)                                             [release notes](Drivers\CMSIS\Device\ST\STM32L4xx\Release_Notes.html)
  STM32L4xx HAL                                               V1.13.0                                           [BSD-3-Clause](https://opensource.org/licenses/BSD-3-Clause)                                                  [release notes](Drivers\STM32L4xx_HAL_Driver\Release_Notes.html)
  BSP STM32L4xx_Nucleo                                        V2.1.7                                            [BSD-3-Clause](https://opensource.org/licenses/BSD-3-Clause)                                                  [release notes](Drivers\BSP\STM32L4xx_Nucleo\Release_Notes.html)

Middlewares
  
  Name                                                        Version                                           License                                                                                                       Release note
  ----------------------------------------------------------- ------------------------------------------------- ------------------------------------------------------------------------------------------------------------- ------------------------------------------------------------------------------------------------------------------------------------------------
  BlueNRG-2                                                    V3.2.1 []{.icon-st-update}                       [SLA0055](https://www.st.com/SLA0055)                                                                         [release notes](Middlewares\ST\BlueNRG-2\Release_Notes.html)

Projects
  
  Name                                                        Version                                           License                                                                                                       Release note
  ----------------------------------------------------------- ------------------------------------------------- ------------------------------------------------------------------------------------------------------------- ------------------------------------------------------------------------------------------------------------------------------------------------
  Projects                                                    V3.2.1 []{.icon-st-update}                        [SLA0055](https://www.st.com/SLA0055)                                                                         [release notes](Projects\Release_Notes.html)

## Known Limitations

- To correctly set the RESET on pin D7 of the X-NUCLEO-BNRG2A1 expansion board, a 0 Ohm resistor must be soldiered on R117. Alternatively, the D7 pin of the Arduino connector and the pin #5 of the J12 must be bridged
- On dual-core STM32 series this expansion software can be used on both cores but exclusively 

## Development Toolchains and Compilers

- IAR Embedded Workbench for ARM (EWARM) toolchain V8.50.9
- RealView Microcontroller Development Kit (MDK-ARM) toolchain V5.32.0
- STM32CubeIDE V1.6.1

## Supported Devices and Boards

- STM32L476RG devices and STM32L476RG-Nucleo board RevB
- X-NUCLEO-BNRG2A1
- BlueNRG-2 and BlueNRG-2N Bluetooth® Low Energy devices

## Dependencies

None

</div>
:::

::: {.collapse}
<input type="checkbox" id="collapse-section5" aria-hidden="true">
<label for="collapse-section5" aria-hidden="true">V3.2.0 / 01-February-2021</label>
<div>			

## Main Changes

### Product update

  Headline
  ----------------------------------------------------------
  Align to STMicroelectronics.X-CUBE-BLE2.3.2.0.pack for STM32CubeMX (minimum required version V6.2.0)
  Add support to Example Selector in STM32CubeMX

## Contents
The X-CUBE-BLE2 expansion software package comes with a rich set of examples running on STMicroelectronics boards, organized by board and provided with preconfigured projects for the main supported toolchains.

<small>The components flagged by "[]{.icon-st-update}" have changed since the
previous release. "[]{.icon-st-add}" are new.</small>

Documentation

  Name                                                        Version                                           License                                                                                                       Document
  ----------------------------------------------------------- ------------------------------------------------- ------------------------------------------------------------------------------------------------------------- ------------------------------------------------------------------------------------------------------------------------------------------------
  X-CUBE-BLE2 API Description                                 V3.2.0 []{.icon-st-update}                        [SLA0055](https://www.st.com/SLA0055)                                                                         [CHM](Documentation\X-CUBE-BLE2.chm)
  Getting Started with X-CUBE-BLE2 pack for STM32CubeMX       V3.2.0 []{.icon-st-update}                        [SLA0055](https://www.st.com/SLA0055)                                                                         [PDF](Documentation\STMicroelectronics.X-CUBE-BLE2_GettingStarted.pdf)

Drivers

  Name                                                        Version                                           License                                                                                                       Release note
  ----------------------------------------------------------- ------------------------------------------------- ------------------------------------------------------------------------------------------------------------- ------------------------------------------------------------------------------------------------------------------------------------------------
  STM32L4xx CMSIS                                             V1.7.1 []{.icon-st-update}                        [Apache License v2.0](https://opensource.org/licenses/Apache-2.0)                                             [release notes](Drivers\CMSIS\Device\ST\STM32L4xx\Release_Notes.html)
  STM32L4xx HAL                                               V1.13.0 []{.icon-st-update}                       [BSD-3-Clause](https://opensource.org/licenses/BSD-3-Clause)                                                  [release notes](Drivers\STM32L4xx_HAL_Driver\Release_Notes.html)
  BSP STM32L4xx_Nucleo                                        V2.1.7                                            [BSD-3-Clause](https://opensource.org/licenses/BSD-3-Clause)                                                  [release notes](Drivers\BSP\STM32L4xx_Nucleo\Release_Notes.html)

Middlewares
  
  Name                                                        Version                                           License                                                                                                       Release note
  ----------------------------------------------------------- ------------------------------------------------- ------------------------------------------------------------------------------------------------------------- ------------------------------------------------------------------------------------------------------------------------------------------------
  BlueNRG-2                                                    V3.2.0 []{.icon-st-update}                       [SLA0055](https://www.st.com/SLA0055)                                                                         [release notes](Middlewares\ST\BlueNRG-2\Release_Notes.html)

Projects
  
  Name                                                        Version                                           License                                                                                                       Release note
  ----------------------------------------------------------- ------------------------------------------------- ------------------------------------------------------------------------------------------------------------- ------------------------------------------------------------------------------------------------------------------------------------------------
  Projects                                                    V3.2.0 []{.icon-st-update}                        [SLA0055](https://www.st.com/SLA0055)                                                                         [release notes](Projects\Release_Notes.html)

## Known Limitations

- To correctly set the RESET on pin D7 of the X-NUCLEO-BNRG2A1 expansion board, a 0 Ohm resistor must be soldiered on R117. Alternatively, the D7 pin of the Arduino connector and the pin #5 of the J12 must be bridged
- On dual-core STM32 series this expansion software can be used on both cores but exclusively 

## Development Toolchains and Compilers

- IAR Embedded Workbench for ARM (EWARM) toolchain V8.50.5
- RealView Microcontroller Development Kit (MDK-ARM) toolchain V5.31.0
- STM32CubeIDE V1.5.1

## Supported Devices and Boards

- STM32L476RG devices and STM32L476RG-Nucleo board RevB
- X-NUCLEO-BNRG2A1
- BlueNRG-2 and BlueNRG-2N Bluetooth® Low Energy devices

## Dependencies

None

</div>
:::

::: {.collapse}
<input type="checkbox" id="collapse-section4" aria-hidden="true">
<label for="collapse-section4" aria-hidden="true">V3.1.0 / 16-November-2020</label>
<div>			

## Main Changes

### Product update

  Headline
  ----------------------------------------------------------
  Align to STMicroelectronics.X-CUBE-BLE2.3.1.0.pack for STM32CubeMX (minimum required version V6.1.0)
  Add support to FOTA (Firmware Over The Air) feature in SensorDemo_BLESensor-App sample application (only for STM32L476RG)

## Contents

<small>The components flagged by "[]{.icon-st-update}" have changed since the
previous release. "[]{.icon-st-add}" are new.</small>

Documentation

  Name                                                        Version                                           License                                                                                                       Document
  ----------------------------------------------------------- ------------------------------------------------- ------------------------------------------------------------------------------------------------------------- ------------------------------------------------------------------------------------------------------------------------------------------------
  X-CUBE-BLE2 API Description                                 V3.1.0 []{.icon-st-update}                        [SLA0055](https://www.st.com/SLA0055)                                                                         [CHM](Documentation\X-CUBE-BLE2.chm)
  Getting Started with X-CUBE-BLE2 pack for STM32CubeMX       V3.1.0 []{.icon-st-update}                        [SLA0055](https://www.st.com/SLA0055)                                                                         [PDF](Documentation\STMicroelectronics.X-CUBE-BLE2_GettingStarted.pdf)

Drivers

  Name                                                        Version                                           License                                                                                                       Release note
  ----------------------------------------------------------- ------------------------------------------------- ------------------------------------------------------------------------------------------------------------- ------------------------------------------------------------------------------------------------------------------------------------------------
  STM32L4xx CMSIS                                             V1.7.0                                            [Apache License v2.0](https://opensource.org/licenses/Apache-2.0)                                             [release notes](Drivers\CMSIS\Device\ST\STM32L4xx\Release_Notes.html)
  STM32L4xx HAL                                               V1.12.0                                           [BSD-3-Clause](https://opensource.org/licenses/BSD-3-Clause)                                                  [release notes](Drivers\STM32L4xx_HAL_Driver\Release_Notes.html)
  BSP STM32L4xx_Nucleo                                        V2.1.6                                            [BSD-3-Clause](https://opensource.org/licenses/BSD-3-Clause)                                                  [release notes](Drivers\BSP\STM32L4xx_Nucleo\Release_Notes.html)

Middlewares
  
  Name                                                        Version                                           License                                                                                                       Release note
  ----------------------------------------------------------- ------------------------------------------------- ------------------------------------------------------------------------------------------------------------- ------------------------------------------------------------------------------------------------------------------------------------------------
  BlueNRG-2                                                    V3.1.0 []{.icon-st-update}                       [SLA0055](https://www.st.com/SLA0055)                                                                         [release notes](Middlewares\ST\BlueNRG-2\Release_Notes.html)

Projects
  
  Name                                                        Version                                           License                                                                                                       Release note
  ----------------------------------------------------------- ------------------------------------------------- ------------------------------------------------------------------------------------------------------------- ------------------------------------------------------------------------------------------------------------------------------------------------
  Projects                                                    V3.1.0 []{.icon-st-update}                        [SLA0055](https://www.st.com/SLA0055)                                                                         [release notes](Projects\Release_Notes.html)

## Known Limitations

- To correctly set the RESET on pin D7 of the X-NUCLEO-BNRG2A1 expansion board, a 0 Ohm resistor must be soldiered on R117. Alternatively, the D7 pin of the Arduino connector and the pin #5 of the J12 must be bridged
- On dual-core STM32 series this expansion software can be used on both cores but exclusively 

## Development Toolchains and Compilers

- IAR Embedded Workbench for ARM (EWARM) toolchain V8.50.5
- RealView Microcontroller Development Kit (MDK-ARM) toolchain V5.31.0
- STM32CubeIDE V1.4.1

## Supported Devices and Boards

- STM32L476RG devices and STM32L476RG-Nucleo board RevB
- X-NUCLEO-BNRG2A1
- BlueNRG-2 and BlueNRG-2N Bluetooth® Low Energy devices

## Dependencies

None

</div>
:::

::: {.collapse}
<input type="checkbox" id="collapse-section3" aria-hidden="true">
<label for="collapse-section3" aria-hidden="true">V3.0.0 / 30-September-2020</label>
<div>			

## Main Changes

### Product update

  Headline
  ----------------------------------------------------------
  Align to STMicroelectronics.X-CUBE-BLE2.3.0.0.pack for STM32CubeMX (minimum required version V6.0.0)
  Add support in BlueNRG-2 middleware to device configuration for IFR and Stack Updater and other improvements
  Add IFRStack_Updater sample application
  Improve sample applications

## Contents

<small>The components flagged by "[]{.icon-st-update}" have changed since the
previous release. "[]{.icon-st-add}" are new.</small>

Documentation

  Name                                                        Version                                           License                                                                                                       Document
  ----------------------------------------------------------- ------------------------------------------------- ------------------------------------------------------------------------------------------------------------- ------------------------------------------------------------------------------------------------------------------------------------------------
  X-CUBE-BLE2 API Description                                 V3.0.0 []{.icon-st-update}                        [SLA0055](https://www.st.com/SLA0055)                                                                         [CHM](Documentation\X-CUBE-BLE2.chm)
  Getting Started with X-CUBE-BLE2 pack for STM32CubeMX       V3.0.0 []{.icon-st-update}                        [SLA0055](https://www.st.com/SLA0055)                                                                         [PDF](Documentation\STMicroelectronics.X-CUBE-BLE2_GettingStarted.pdf)

Drivers 

  Name                                                        Version                                           License                                                                                                       Release note
  ----------------------------------------------------------- ------------------------------------------------- ------------------------------------------------------------------------------------------------------------- ------------------------------------------------------------------------------------------------------------------------------------------------
  STM32L4xx CMSIS                                             V1.7.0                                            [Apache License v2.0](https://opensource.org/licenses/Apache-2.0)                                             [release notes](Drivers\CMSIS\Device\ST\STM32L4xx\Release_Notes.html)
  STM32L4xx HAL                                               V1.12.0                                           [BSD-3-Clause](https://opensource.org/licenses/BSD-3-Clause)                                                  [release notes](Drivers\STM32L4xx_HAL_Driver\Release_Notes.html)
  BSP STM32L4xx_Nucleo                                        V2.1.6                                            [BSD-3-Clause](https://opensource.org/licenses/BSD-3-Clause)                                                  [release notes](Drivers\BSP\STM32L4xx_Nucleo\Release_Notes.html)

Middlewares
  
  Name                                                        Version                                           License                                                                                                       Release note
  ----------------------------------------------------------- ------------------------------------------------- ------------------------------------------------------------------------------------------------------------- ------------------------------------------------------------------------------------------------------------------------------------------------
  BlueNRG-2                                                    V3.0.0 []{.icon-st-update}                       [SLA0055](https://www.st.com/SLA0055)                                                                         [release notes](Middlewares\ST\BlueNRG-2\Release_Notes.html)

Projects
  
  Name                                                        Version                                           License                                                                                                       Release note
  ----------------------------------------------------------- ------------------------------------------------- ------------------------------------------------------------------------------------------------------------- ------------------------------------------------------------------------------------------------------------------------------------------------
  Projects                                                    V3.0.0 []{.icon-st-update}                        [SLA0055](https://www.st.com/SLA0055)                                                                         [release notes](Projects\Release_Notes.html)

## Known Limitations

- To correctly set the RESET on pin D7 of the X-NUCLEO-BNRG2A1 expansion board, a 0 Ohm resistor must be soldiered on R117. Alternatively, the D7 pin of the Arduino connector and the pin #5 of the J12 must be bridged
- On dual-core STM32 series this expansion software can be used on both cores but exclusively 

## Development Toolchains and Compilers

- IAR Embedded Workbench for ARM (EWARM) toolchain V8.50.5
- RealView Microcontroller Development Kit (MDK-ARM) toolchain V5.31.0
- STM32CubeIDE V1.4.1

## Supported Devices and Boards

- STM32L476RG devices and STM32L476RG-Nucleo board RevB
- X-NUCLEO-BNRG2A1
- BlueNRG-2 and BlueNRG-2N Bluetooth® Low Energy devices

## Dependencies

None

</div>
:::

::: {.collapse}
<input type="checkbox" id="collapse-section2" aria-hidden="true">
<label for="collapse-section2" aria-hidden="true">V2.0.0 / 03-July-2020</label>
<div>

## Main Changes

### Product update

  Headline
  ----------------------------------------------------------
  Align to STMicroelectronics.X-CUBE-BLE2.2.0.0.pack for STM32CubeMX (minimum required version V6.0.0)
  Align middleware to BlueNRG-1_2 DK 3.2.0
  Update on STM32Cube HAL drivers
  Add Central sample application
  Add support to BLE char value length greater than 20 bytes in SampleApp sample application
  Introduce support to BlueNRG-2N device

## Contents

<small>The components flagged by "[]{.icon-st-update}" have changed since the
previous release. "[]{.icon-st-add}" are new.</small>

Documentation

  Name                                                        Version                                           License                                                                                                       Document
  ----------------------------------------------------------- ------------------------------------------------- ------------------------------------------------------------------------------------------------------------- ------------------------------------------------------------------------------------------------------------------------------------------------
  X-CUBE-BLE2 API Description                                 V2.0.0 []{.icon-st-update}                        [SLA0055](https://www.st.com/SLA0055)                                                                         [CHM](Documentation\X-CUBE-BLE2.chm)
  Getting Started with X-CUBE-BLE2 pack for STM32CubeMX       V2.0.0 []{.icon-st-update}                        [SLA0055](https://www.st.com/SLA0055)                                                                         [PDF](Documentation\STMicroelectronics.X-CUBE-BLE2_GettingStarted.pdf)

Drivers

  Name                                                        Version                                           License                                                                                                       Release note
  ----------------------------------------------------------- ------------------------------------------------- ------------------------------------------------------------------------------------------------------------- ------------------------------------------------------------------------------------------------------------------------------------------------
  STM32L4xx CMSIS                                             V1.7.0 []{.icon-st-update}                        [Apache License v2.0](https://opensource.org/licenses/Apache-2.0)                                             [release notes](Drivers\CMSIS\Device\ST\STM32L4xx\Release_Notes.html)
  STM32L4xx HAL                                               V1.12.0 []{.icon-st-update}                       [BSD-3-Clause](https://opensource.org/licenses/BSD-3-Clause)                                                  [release notes](Drivers\STM32L4xx_HAL_Driver\Release_Notes.html)
  BSP STM32L4xx_Nucleo                                        V2.1.6                                            [BSD-3-Clause](https://opensource.org/licenses/BSD-3-Clause)                                                  [release notes](Drivers\BSP\STM32L4xx_Nucleo\Release_Notes.html)

Middlewares
  
  Name                                                        Version                                           License                                                                                                       Release note
  ----------------------------------------------------------- ------------------------------------------------- ------------------------------------------------------------------------------------------------------------- ------------------------------------------------------------------------------------------------------------------------------------------------
  BlueNRG-2                                                    V2.0.0 []{.icon-st-update}                       [SLA0055](https://www.st.com/SLA0055)                                                                         [release notes](Middlewares\ST\BlueNRG-2\Release_Notes.html)

Projects
  
  Name                                                        Version                                           License                                                                                                       Release note
  ----------------------------------------------------------- ------------------------------------------------- ------------------------------------------------------------------------------------------------------------- ------------------------------------------------------------------------------------------------------------------------------------------------
  Projects                                                    V2.0.0 []{.icon-st-update}                        [SLA0055](https://www.st.com/SLA0055)                                                                         [release notes](Projects\Release_Notes.html)

## Known Limitations

- To correctly set the RESET on pin D7 of the X-NUCLEO-BNRG2A1 expansion board, a 0 Ohm resistor must be soldiered on R117. Alternatively, the D7 pin of the Arduino connector and the pin #5 of the J12 must be bridged

## Development Toolchains and Compilers

- IAR Embedded Workbench for ARM (EWARM) toolchain V8.32.3
- RealView Microcontroller Development Kit (MDK-ARM) toolchain V5.29.0
- STM32CubeIDE V1.3.1

## Supported Devices and Boards

- STM32L476RG devices and STM32L476RG-Nucleo board RevB
- X-NUCLEO-BNRG2A1
- BlueNRG-2 and BlueNRG-2N Bluetooth® Low Energy devices

## Dependencies

None

</div>
:::

::: {.collapse}
<input type="checkbox" id="collapse-section1" aria-hidden="true">
<label for="collapse-section1" aria-hidden="true">V1.0.0 / 13-December-2019</label>
<div>			

## Main Changes

### First release

## Contents

Documentation

  Name                                                        Version                                           License                                                                                                       Document
  ----------------------------------------------------------- ------------------------------------------------- ------------------------------------------------------------------------------------------------------------- ------------------------------------------------------------------------------------------------------------------------------------------------
  X-CUBE-BLE2 API Description                                 V1.0.0                                            [SLA0055](https://www.st.com/SLA0055)                                                                         [CHM](Documentation\X-CUBE-BLE2.chm)
  Getting Started with X-CUBE-BLE2 pack for STM32CubeMX       V1.0.0                                            [SLA0055](https://www.st.com/SLA0055)                                                                         [PDF](Documentation\STMicroelectronics.X-CUBE-BLE2_GettingStarted.pdf)

Drivers

  Name                                                        Version                                           License                                                                                                       Release note
  ----------------------------------------------------------- ------------------------------------------------- ------------------------------------------------------------------------------------------------------------- ------------------------------------------------------------------------------------------------------------------------------------------------
  STM32L4xx CMSIS                                             V1.6.0                                            [BSD-3-Clause](https://opensource.org/licenses/BSD-3-Clause)                                                  [release notes](Drivers\CMSIS\Device\ST\STM32L4xx\Release_Notes.html)
  STM32L4xx HAL                                               V1.11.0                                           [BSD-3-Clause](https://opensource.org/licenses/BSD-3-Clause)                                                  [release notes](Drivers\STM32L4xx_HAL_Driver\Release_Notes.html)
  BSP STM32L4xx_Nucleo                                        V2.1.6                                            [BSD-3-Clause](https://opensource.org/licenses/BSD-3-Clause)                                                  [release notes](Drivers\BSP\STM32L4xx_Nucleo\Release_Notes.html)

Middlewares
  
  Name                                                        Version                                           License                                                                                                       Release note
  ----------------------------------------------------------- ------------------------------------------------- ------------------------------------------------------------------------------------------------------------- ------------------------------------------------------------------------------------------------------------------------------------------------
  BlueNRG-2                                                    V1.0.0                                            [SLA0055](https://www.st.com/SLA0055)                                                                         [release notes](Middlewares\ST\BlueNRG-2\Release_Notes.html)

Projects

  Name                                                        Version                                           License                                                                                                       Release note
  ----------------------------------------------------------- ------------------------------------------------- ------------------------------------------------------------------------------------------------------------- ------------------------------------------------------------------------------------------------------------------------------------------------
  Projects                                                    V1.0.0                                            [SLA0055](https://www.st.com/SLA0055)                                                                         [release notes](Projects\Release_Notes.html)

## Known Limitations

- To correctly set the RESET on pin D7 of the X-NUCLEO-BNRG2A1 expansion board, a 0 Ohm resistor must be soldiered on R117. Alternatively, the D7 pin of the Arduino connector and the pin #5 of the J12 must be bridged

## Development Toolchains and Compilers

- IAR Embedded Workbench for ARM (EWARM) toolchain V8.32.3
- RealView Microcontroller Development Kit (MDK-ARM) toolchain V5.27.1
- STM32CubeIDE V1.1.0

## Supported Devices and Boards

- STM32L476RG devices and STM32L476RG-Nucleo board RevB
- X-NUCLEO-BNRG2A1
- BlueNRG-2 Bluetooth® Low Energy device

## Backward Compatibility

None

## Dependencies

None

</div>
:::

:::
:::

<footer class="sticky">
::: {.columns}
::: {.column width="95%"}
For complete documentation on **X-CUBE-BLE2** ,
visit: [www.st.com](https://www.st.com/content/st_com/en/products/embedded-software/mcu-mpu-embedded-software/stm32-embedded-software/stm32cube-expansion-packages/x-cube-ble2.html)
:::
::: {.column width="5%"}
<abbr title="Based on template cx566953 version 2.0">Info</abbr>
:::
:::
</footer>
