ICM-20948 on Tiva TM4C1294
======================


This repository contains an example project for running ICM-20948 IMU module with TM4C1294 microcontroller.

## Functionality

* Communication over SPI only
* Original DMP firmware with 9DOF sensor fusion
* Reading back orientation (from 6DOf or 9DOF fusion algorithm), linear acceleration or angular velocity
* ICM-20948 class is implemented as C++ singleton (since it depends on HW interface)

#### DMP

DMP firmware was released by the InvenSense and can perform 9DOF sensor fusion to compute the quaternions from which it is possible to find commonly used Euler angles. It supports on-chip 6DOF or 9DOF sensor fusion, in addition to providing gravity vector, linear acceleration and angular velocity. Output rate of the fusion algorithm is currently set to 200Hz (can be increased to 225Hz according to documentation).

General flow when using the DMP library is to configure the sensors (accelerometer, gyro, magnetometer), flash DMP firmware, then enable the desired output and its sampling rate.


## Wiring in SPI mode

ICM-20948 can be connected to SPI2 peripheral of Tiva evaluation kit as follows:

ICM-20948       |   EK-TM4C1294XL
----------------|------------------
ICM-20948 SDA   | PD1(SPI2MOSI)
ICM-20948 SCL   | PD3(SPI2CLK)
ICM-20948 INT   | PA5(GPIO)
ICM-20948 AD0   | PD0(SPI2MOSI)
ICM-20948 NCS   | PN2(GPIO used as slave select)
ICM-20948 VCC   | 3.3V
ICM-20948 GND   | GND

Speed of SPI transfer is set to 1MHz. Additionally, this example uses power control functionality through pin PL4. It is meant to control an external n-type MOSFET to cut the power to ICM-20948. Power control signal is designed as active-high, cutting the power to ICM-20948 when it's set low.


## Example code

``main.cpp`` contains a simple example which demonstrates initialization of the sensor, and two blocks of code showing how to get orientation, or acceleration and gyroscope measurements. The remainder of the library can be found in ``icm20948/`` folder.

To use it in your project, copy folders ``icm20948/``, ``libs/``, ``HAL/`` and a file ``hwconfig.h``. To use on a different platform, refer to the section below. 


## Porting the library

Even though the library was developed and tested on TM4C1294 the functional code is fully decoupled from hardware through the use of Hardware Abstraction Layer (HAL). If you want to experiment with support for other board simply create new folder in ``HAL/``, add in the same files as in ``HAL/tm4c1294/``. Keep interface of new HAL the same as that in ``HAL/tm4c1294/``, i.e. use same function names as those in header files ``HAL/tm4c1294/*.h``. Main HAL include file, ``HAL/hal.h``, then uses macros to select the right board and load appropriate board drivers.

## Testing the IMU

Give a try to my other project which I developed while working on this library: [Data dashboard](https://github.com/vedranMv/dataDashboard). A QT-based dashboard for visualizing real-time data.

![alt tag](https://cdn.hackaday.io/images/7187981605656673038.PNG)