# AVATAR X-Prize Arm
This repo contains Code Composer Studio projects for a variety of different tasks involving the AVATAR X-PRIZE Arm motor controllers. These controllers were developed as part of the AVATAR X-PRIZE Arm capstone project in Fall 2019 at Northeastern University. The goal of this project was to create a system that allowed a person to control a robot arm using an exoskeleton that provides real-time haptic feedback from the environment that the robot arm is interacting with. Both the robotic arm and exoskeleton were custom hardware that made use of psuedo-direct drive brushless servo actuators. More info can be found on the project in [this paper.](https://www.domyam.com/Final-Report-AVATAR-Arm.pdf)

# BLDC Servo Controller

The BLDC Servo controller is built around the [TI Tiva C Series TM4C123x MCU](https://www.ti.com/lit/ds/symlink/tm4c1231h6pm.pdf), which is an ARM Cortex-M4F based MCU. We selected this MCU due to its motor control features including the avaialable number of PWM timers and quadrature decoder. 
A [TI DRV8323](https://www.ti.com/lit/ds/symlink/drv8323.pdf) is used as a gate driver IC. 
An [AMS AS5047P](https://ams.com/en/as5047p) was used to measure motor rotor position for servo control and Field Oriented control of the motor phases.
The motor controller PCB was mounted to a **T-Motor U8 Lite** drone motor to create the servo.

This project was inspired by the work done on the MIT Mini-Cheetah actuators by Ben Katz. [This project](https://dspace.mit.edu/bitstream/handle/1721.1/118671/1057343368-MIT.pdf) was used as a reference for both the development of the motor controller PCB and the firmware.

## Project Folders

The main motor driver firmware is the **freertos_motor_demo** project. This project inludes closed-loop position control of the servo and communications via RS-485. This project is untested due to the failure of accurate motor calibration.  
The **freertos_encoder_demo** project was used to run encoders only on the exoskeleton for out capstone demo day Fall 2019.  
The **flash_encoder_settings** and **flash_driver_settings** projects are simple scripts to write the proper settings to those peripherals on the Motor Driver board. They also have some testing capability.  
The **motor_calibration** project is used to find the default positive rotation direction of the motor and its electrical offset. This project is untested and needs to be verified accurate.  

## Dependencies

* [Code Composer Studio](http://www.ti.com/tool/CCSTUDIO-TM4X)

* [TivaWare C Series for TM4C123](http://www.ti.com/tool/SW-TM4C)

  * FreeRTOS - comes packaged in TivaWare

Code Composer is based on Eclipse but has a lot of the TI/ARM buid toolchain stuff built in. You can use other IDE's but CCS will save you a lot of headaches setting up a build toolchain. 

## Getting the code to run on your machine

Everything should be packaged so that you can open one of the projects in Code Composer and hit the ground running. 

In order for this to happen, TivaWare should be installed in C:/ti/. This is the default location if you use the installer from TI. It looks like a newer version of TivaWare was recently released. If you happen to download this version you will need to change the project includes. 

The safest way to do this is to right click on your project in the Project Explorer sidebar. 
Open Properties. Go to the Build menu and click on the Variables tab. 
Here you can change the SW_ROOT_VARIABLE to point to your TivaWare folder. 

Try building the project. Hopefully it returns a build successful with maybe a few errors.

# Developing the code

For a good feel of the structure of the code read through a few of the TivaWare examples. The formatting we use mostly matches those.

We also heavily rely upon FreeRTOS for the motor board controller. Read their [Getting Started](https://www.freertos.org/FreeRTOS-quick-start-guide.html#page_top) page and our code to help you figure out how that works.

The schematic for the motor controller PCB developed for the AVATAR Arm Capstone project is included in this repo. The pin assignments in the firmware are for that use case.

You can always test your code on a TI LanchPad but you might need to change some of the pinouts. Make sure when you're wiring everything up that your physical pins match your code. There should be breakout boards for all of the MCU peripherals that we used on the Motor Driver. 

GOOD LUCK!!

