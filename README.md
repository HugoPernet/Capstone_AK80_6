# Exo skeleton Capstone 2023-2024


This repository is intended to be used with the single-actuator exoskeleton prototype developed as part of UC Berkeley 2023-2024 capstone project.

## Supervisor:
- Pr. H.Kazerooni

## Contributors:
- Hugo Pernet: +1 628-236-9604, pernet.hugo@gmail.com
- Kathy Min: 
- Abhay Bhandari
- Isaiah Dillard
- Clea Rita Al Haddad
- Tong Wang

## Hardware requierments:
_Motor:_
this code was developed to work for [Tmotor AK series ](https://breakdance.github.io/breakdance/) more specifically for the [AK80-6 motor](https://store.tmotor.com/images/file/202208/251661393360838805.pdf).

_IMU:_
this code supposes the use of an IMU with an MPU-6050 chipset [MPU 6050](https://invensense.tdk.com/wp-content/uploads/2015/02/MPU-6000-Datasheet1.pdf)

_IC:_
this code was designed to be compiled for the [Adafruit Feather M4 CAN express](https://learn.adafruit.com/adafruit-feather-m4-can-express) using PlatformIO on VsCode.

> Note: For the next versions, we recommend switching to an ESP32-based platform with a separate CAN controller. This Feather board has very little documentation available on its CAN controller, making it hard to implement and troubleshoot.

## Software requierements:
All required libraries are included in this project in the _lib_ file.
