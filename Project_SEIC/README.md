# Project SEIC

## Overview

This project contains two Arduino sketches: `Project_SEIC_1.ino` and `Project_SEIC_2.ino`.

### Project_SEIC_1.ino

- **Functionality**: 
  - Reads gas levels using an MQ2 sensor.
  - Monitors distance using an ultrasonic sensor (HCSR04).
  - Blinks an LED and sounds a buzzer based on gas detection and distance measurements.
  - Uses Blynk for remote monitoring and control.

### Project_SEIC_2.ino

- **Functionality**:
  - Reads temperature and humidity using a DHT11 sensor.
  - Reads luminosity using an LS06S sensor.
  - Controls LEDs and a buzzer based on sensor readings.
  - Determines environmental conditions and provides feedback through LEDs and buzzer.

## Files

1. **Project_SEIC_1.ino**: Monitors gas levels and distance, and provides alerts via LEDs and buzzer.
2. **Project_SEIC_2.ino**: Monitors temperature, humidity, and luminosity, and provides alerts via LEDs and buzzer.

## Setup

1. **Project_SEIC_1.ino**:
   - Connect MQ2 sensor to pin 15.
   - Connect HCSR04 ultrasonic sensor to pins 4 (Trigger) and 5 (Echo).
   - Connect LED to pin 21.
   - Connect second LED to pin 12.
   - Connect buzzer to pin 23.

2. **Project_SEIC_2.ino**:
   - Connect DHT11 sensor to pin 22.
   - Connect LS06S sensor to pin 21.
   - Connect LED to pin 18.
   - Connect second LED to pin 19.
   - Connect buzzer to pin 12.
