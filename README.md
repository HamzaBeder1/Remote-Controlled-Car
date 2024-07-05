# RC Car

## Purpose
The purpose of this project was to make a RC car. This project was made by using the following components:
* L298N motor driver: controlled the movement of the motors, allowing the car to move.
* HC05 Bluetooth module: allowed remote control of the car by using Bluetooth and UART.
* ST7735 TFT LCD module: allowed on-board diagnostics to be viewed. SPI was used to communicate with the module.
* HC-SR04 Ultrasonic sensor: prevented collisions with nearby objects.
* MPU-6050 gyroscope and accelerometer: used to measure acceleration and orientation. The MCU communicated with this module using I2C.

## Challenges:
* Connecting HC05 to laptop: The module would not appear under discoverable devices initially. Settings on my laptop needed to be changed.
* Communicating with modules using serial communication: A logic analyzer was used to debug issues with communication between the MCU and the modules.
* 
