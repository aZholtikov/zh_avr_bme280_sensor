# AVR firmware for BME280 wireless sensor (nRF24L01+)

## Features

1. Wireless sensor firmware for ATMega168/328 microcontrollers with BME280 temperature, pressure, and humidity sensor.
2. Power consumption in sleep mode approximately 5 uA using Power-down mode.
3. Automatic over-the-air pairing: the sensor requests a device id from the gateway and stores it in EEPROM.
4. Transmits device identifier, battery voltage, and all three BME280 sensor readings (×10 scaling) on each measurement cycle.
5. Battery measurement through internal ADC with 1.1 V bandgap reference.
6. Wake-up from Power-down mode by watchdog timer interrupt with configurable interval.
7. Supports ATMega168, ATMega168P, ATMega168PB, ATMega328, ATMega328P, and ATMega328PB.
8. Hardware TWI (I2C) for BME280 communication on PORTC pins PC4 (SDA) and PC5 (SCL).

## Attention

1. A compatible RF Gateway is required to receive packets and assign a device id during pairing.
2. The watchdog timer interval is fixed at 8 seconds; the sensor transmits every 38 ticks (~5 minutes).
3. Changing gateway address or channel requires reflashing the microcontroller.
4. If the gateway does not respond during pairing, the device enters permanent sleep until reset.

## Using

In an existing PlatformIO project, run the following command to install the firmware:

```bash
cd your_project
git clone https://github.com/aZholtikov/zh_avr_bme280_sensor
```
