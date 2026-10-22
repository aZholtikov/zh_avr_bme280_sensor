# AVR firmware for BME280 wireless sensor (nRF24L01+)

## Features

1. Wireless sensor firmware for ATMega168/328 microcontrollers with BME280 temperature, pressure, and humidity sensor.
2. Power consumption in sleep mode approximately 5 uA using Power-down mode.
3. Transmits device identifier, battery voltage, and all three BME280 sensor readings on each measurement cycle.
4. Battery measurement through internal ADC with 1.1 V bandgap reference.
5. Wake-up from Power-down mode by watchdog timer interrupt with configurable interval.
6. Supports ATMega168, ATMega168P, ATMega168PB, ATMega328, ATMega328P, and ATMega328PB.
7. Hardware TWI (I2C) for BME280 communication on PORTC pins PC4 (SDA) and PC5 (SCL).

## Attention

1. A compatible RF Gateway is required to receive and process transmitted packets.
2. Changing gateway address or channel requires reflashing the microcontroller.
3. The watchdog timer interval is fixed at 8 seconds; the sensor transmits every 38 ticks (~5 minutes).

## Using

In an existing PlatformIO project, run the following command to install the firmware:

```bash
cd your_project
git clone https://github.com/aZholtikov/zh_avr_bme280_sensor
```
