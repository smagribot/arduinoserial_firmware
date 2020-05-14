# Smagribot 🌱

Smagribot 🌱 is an open source indoor iot plant sensor for hydroponics.

It takes control of the environmental circumstances for growing vegetables and herbs in your own home. It monitors, among other things, the temperature, humidity and water temperature. The monitored data allows the system to control a grow light and a water pump to optimize the health and growth of the plants from sowing to harvest.

# Arduino Serial Firmware
Firmware for controlling and reading different sensors with an Arduino via usb serial.

## Hardware/Sensors
- Arduino Uno Rev3 
    - Microcontroller, controls sensors and gets commands via usb serial
- Arduino Samgribot Shield 
    - Easy connection to senors
- USB A/B Cable 
    - Connect Arduino to device, to receive commands
- 12V 1A power supply
- 12V Fan with PWM and RPM signal 
    - For example: Noctua NF-A14 iPPC-3000
- DHT22 or DHT11 
    - Sensor for reading temperature and humdity
- waterproof DS18B20 
    - Sensor for reading water temperature
- Float switch/sensor
    - To check if water must be refilled

## Connection
Firmware uses usb serial of the Arduino.
- Baudrate: 9600
- Delimiter: CR & LF `\r\n` 

## Supported commands
- `setfan value` sets fan speed to value. Value must be between 0-80, non-inclusive. Returns `OK` or `ERROR`.
    - example: `setfan 79` for maximum fan speed
- `getfan` returns fan rpm. Returns value as `int`.
- `getfill` returns 1 or 0 for fill sensor.
- `getdht` returns `humidity temperature` as floats. Humidity in % and Temperature in °C. Returns `ERROR` when Sensor value is invalid.
- `getwatertmp` returns `temperature` as float. Temperature in °C.
- `setrelay relayNo status` sets relay on position `relayNo` to on/off. Returns `OK`.
    - example: `setrelay 0 1` sets relay on position 0 to on.
- `led status` sets built-in led to on/off. Returns `OK`. For testing/sanity checking.
    - example: `led 1` turns led on.
- `getfw` returns current firmware version. In format: Major.Minor.Update, ex.: `0.0.1`.

## Dependencies
- OneWire 2.3.5
- DallasTemperature 3.8.0
- DHT sensor library 1.3.8
- CmdParser 1.6.0

## Tested devices
- Arduino Uno Rev3

## CI/CD
Pipeline for building firmware with Azure DevOps is shown in [arduino_firmware_master_on_azure-pipelines.yml](arduino_firmware_master_on_azure-pipelines.yml).
Pipline needs [arduino-cli extension](https://marketplace.visualstudio.com/items?itemName=PhilippManstein.arduino-cli) installed.