STM32 UART_IAP_Bootloader
=========================

* Device: stm32f107vct6
* Tool:   Keil MDK-ARM uvision5
* How to Start:   
step1: Download project.

step2: Open IAP project and program to chip.

step3: Open uart terminal([realterm](http://sourceforge.net/projects/realterm/)) and setting baudrate: 115200.

step4: Open APP project and compile, the binary code will be create at ./Program_APP/Project.bin

step5: Using uart terminal software select ./Program_APP/Project.bin then send it.

step6: Uart terminal will recive data of Project.bin and the led PE2,PE5 blink once. Now is in APP mode. Have fun!
  
How Simple Bootloader work?
=========================
In the First, host send data of Project.bin. Then IAP recive the data size equal to define ProgramSize. The IAP will write program to flash and jump to setting address to execute APP.

* rurume1010@gmail.com
