# X-CUBE-BLE2 Firmware Package

![latest tag](https://img.shields.io/github/v/tag/STMicroelectronics/x-cube-ble2.svg?color=brightgreen)

The X-CUBE-BLE2 is an expansion software package for STM32Cube. This software provides drivers running on STM32 for STM's BlueNRG-2 Bluetooth Low Energy device. This software package is built on top of STM32Cube software technology that ease portability across different STM32 microcontrollers.

**X-CUBE-BLE2 software features**:

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

**Related information and documentation**:

- [UM2666](https://www.st.com/resource/en/user_manual/dm00672214-getting-started-with-the-xcubeble2-bluetooth-low-energy-software-expansion-for-stm32cube-stmicroelectronics.pdf): Getting started with the X-CUBE-BLE2 Bluetooth Low Energy software expansion for STM32Cube
- [STM32Cube](http://www.st.com/stm32cube)
- [STM32 Nucleo boards](http://www.st.com/stm32nucleo)
- [How to use the X-CUBE-BLE2 pack in the STM32CubeMX tool](https://www.youtube.com/watch?v=BzKIT2T5Q4c)
