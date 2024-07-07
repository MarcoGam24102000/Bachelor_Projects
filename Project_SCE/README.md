# FreeRTOS-based Sensor Monitoring System

This project utilizes FreeRTOS to manage tasks for monitoring various sensors and controlling LEDs and a buzzer based on sensor readings. The system includes temperature and humidity sensing using a DHT11 sensor, luminosity sensing with an LS06S sensor, and distance measurement using an HC-SR04 sensor.

## Components

- **DHT11 Sensor**: Measures temperature and humidity.
- **LS06S Sensor**: Measures luminosity.
- **HC-SR04 Sensor**: Measures distance.
- **LCD**: Displays sensor readings.
- **Buzzer and LEDs**: Provide visual and audible alerts based on sensor thresholds.

## Requirements

- **Arduino IDE**: To compile and upload the code to the Arduino board.
- **Libraries**:
  - `DHT.h`: Library for DHT temperature and humidity sensor.
  - `Ultrasonic.h`: Library for HC-SR04 ultrasonic sensor.
  - `Wire.h`: Library for I2C communication.
  - `LiquidCrystal_I2C.h`: Library for LCD display using I2C.

## Setup

1. **Hardware Setup**:
   - Connect DHT11 sensor to pin 4.
   - Connect LS06S sensor to analog pin 32.
   - Connect HC-SR04 trigger pin to pin 12 and echo pin to pin 13.
   - Connect LEDs and buzzer to respective pins (defined in the code).
   - Connect LCD via I2C using the address 0x27.

2. **Software Setup**:
   - Install Arduino IDE if not already installed.
   - Install required libraries (`DHT.h`, `Ultrasonic.h`, `Wire.h`, `LiquidCrystal_I2C.h`).

3. **Upload Code**:
   - Open `sensor_monitoring.ino` in Arduino IDE.
   - Select the correct board and port.
   - Upload the code to the Arduino board.

## Usage

1. **Power On**:
   - Power on the Arduino board.

2. **Monitor Sensor Readings**:
   - The system will start monitoring temperature, humidity, luminosity, and distance.
   - LCD will display the corresponding sensor readings.
   - LEDs will indicate status based on predefined thresholds.

3. **Interact with the System**:
   - Observe changes in LED states and LCD display as sensor readings vary.
   - Proximity LED and buzzer will activate based on defined conditions.

## Contributing

Contributions to improve the codebase or add features are welcome. Please fork the repository, make your changes, and submit a pull request.

