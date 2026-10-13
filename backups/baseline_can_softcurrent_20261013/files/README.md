# BLDC_FOC_G431_SENSORLESS

Sensorless FOC project for STM32G431CB using PlatformIO and CMSIS.

## Hardware

- MCU: STM32G431CB
- Motor: 2207 BLDC
- Development environment: PlatformIO
- Framework: CMSIS
- Debugger: ST-Link

## Project Goal

Develop and verify a sensorless field-oriented control system for BLDC motors.

Planned development stages:

1. PWM / SVPWM
2. ADC current sampling
3. Current offset calibration
4. Open-loop startup
5. FOC current loop
6. Sensorless observer
7. Observer lock detection
8. Open-loop to sensorless closed-loop transition
9. Speed control
10. Experimental validation
