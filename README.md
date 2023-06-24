# STM32F103C8T6_PID_Motor
## Created 06/24/2023

This project aim is to prototype the ability to cooperate multiple different module of MCU STM32F103C8T6 Blue Pill, to control the speed and position of the DC servo motor with an attached encoder, using the PID control method. Besides, to save time for configuration of the modules, I use the HAL library and CubeMX to configure the chip.

In this project, I used the UART - TTL chip to connect the MCU to my PC and send command to control the motor, which the respectively following command
- '1': 50% maximum speed
- '2': 70% maximum speed
- '3': 100% maximum speed
- '4': turn 45 degree CCW
- '5': turn 60 degree CCW
- '6': turn 90 degree CCW
- '7': turn 180 degree CCW
- '8': turn 270 degree CCW
- '9': turn 360 degree CCW
- Press the button to stop the motor immediately

All the PID parameters to control the specific position of the motor shaft are tuned by experience, and it used to took me a huge time to get this experience during one of my previous project. However, due to that painful experience, the project this time takes me only haft of a day. I'm planning to continue to develop this project so that I can remotely control using Bluetooth, or I can configure the UART module to interract with microcomputer to run ROS, in order to control mobile robots.

Components:
- MCU STM32F103C8T6 Blue Pill
- DC servo motor with encoder (374 pulse per revolution, PWM frequency 2000Hz)
- L298N motor driver
- UART-TTL chip

![System circuit](https://github.com/HarryNguyen2023/STM32F103C8T6_PID_Motor/assets/136590151/6491fa03-c6b6-4349-b2d2-e2a1e160aeb3)

Also, I want to appreciate Mr. Khaled Magdy for creating an incredible serial monitor that helps me alot to debug during this and many other projects of mine.

![Elite Serial Monitor by Khaled Magdy](https://github.com/HarryNguyen2023/STM32F103C8T6_PID_Motor/assets/136590151/5abdb3b5-8123-47a7-a108-972198d285af)

### Update
I have successfully update the project to remotely control the motor via Bluetooth using HC-05 Bluetooth IC and Serial Bluetooth Terminal app on my mobilephone

![System with HC-05 Bluetooth IC](https://github.com/HarryNguyen2023/STM32F103C8T6_PID_Motor/assets/136590151/675168f3-6d9c-487b-b208-6748f7bf85c3)

## References
1. STMicroelectronics, STM32F10x datasheet.
2. STMicroelectronics, STM32F10x Reference mannual.
3. Khaled Magdy, STM32 ARM tutorial series, https://deepbluembedded.com/stm32-arm-programming-tutorials/.






