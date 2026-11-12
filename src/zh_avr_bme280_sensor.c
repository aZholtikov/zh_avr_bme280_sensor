#include "avr/io.h"
#include "avr/interrupt.h"
#include "avr/sleep.h"
#include "avr/wdt.h"
#include "util/delay.h"
#include "stdbool.h"
#include "math.h"
#include "zh_rf_config.h"

#if defined(__AVR_ATmega328PB__)
#ifndef SPCR
#define SPCR SPCR0
#endif
#ifndef SPDR
#define SPDR SPDR0
#endif
#ifndef SPSR
#define SPSR SPSR0
#endif
#ifndef SPE
#define SPE SPE0
#endif
#ifndef MSTR
#define MSTR MSTR0
#endif
#ifndef SPR0
#define SPR0 SPR00
#endif
#ifndef SPIF
#define SPIF SPIF0
#endif
#ifndef TWBR
#define TWBR TWBR0
#endif
#ifndef TWCR
#define TWCR TWCR0
#endif
#ifndef TWSR
#define TWSR TWSR0
#endif
#ifndef TWDR
#define TWDR TWDR0
#endif
#endif

#define BATTERY_VOLTAGE_CALIBRATION 1.0
#define TEMPERATURE_CALIBRATION 0.0
#define PRESSURE_CALIBRATION 0.0
#define HUMIDITY_CALIBRATION 0.0

#define GATEWAY_CHANNEL 120
#define DEVICE_ID 5

#define WDT_WAKE_COUNT 38
#define ZH_TWI_TWBR 32

#define NRF_CMD_W_REGISTER 0x20
#define NRF_CMD_W_TX_PAYLOAD 0xA0
#define NRF_CMD_FLUSH_TX 0xE1
#define NRF_CMD_NOP 0xFF

#define NRF_REG_CONFIG 0x00
#define NRF_REG_EN_AA 0x01
#define NRF_REG_EN_RXADDR 0x02
#define NRF_REG_SETUP_AW 0x03
#define NRF_REG_SETUP_RETR 0x04
#define NRF_REG_RF_CH 0x05
#define NRF_REG_RF_SETUP 0x06
#define NRF_REG_STATUS 0x07
#define NRF_REG_RX_ADDR_P0 0x0A
#define NRF_REG_TX_ADDR 0x10

#define NRF_CONFIG_MASK_RX_DR 0x40
#define NRF_CONFIG_EN_CRC 0x08
#define NRF_CONFIG_PWR_UP 0x02

#define NRF_STATUS_RX_DR 0x40
#define NRF_STATUS_TX_DS 0x20
#define NRF_STATUS_MAX_RT 0x10

#define BME280_ADDR 0x76
#define BME280_REG_ID 0xD0
#define BME280_REG_RESET 0xE0
#define BME280_REG_CTRL_HUM 0xF2
#define BME280_REG_CTRL_MEAS 0xF4
#define BME280_REG_CONFIG 0xF5
#define BME280_REG_DATA 0xF7
#define BME280_REG_CALIB00 0x88
#define BME280_REG_CALIB26 0xE1

/**
 * @brief Initialize the watchdog timer for periodic wake-up.
 *
 * Configures the watchdog timer with a ~8 second interval using prescaler
 * values WDP3 and WDP0, enables the watchdog interrupt, and clears the
 * MCU reset flag. The watchdog interrupt service routine increments
 * wdt_wake_count on each trigger.
 *
 * Note: this function intentionally does not enable global interrupts
 * itself; the caller is expected to call sei() once all peripherals
 * have been initialized.
 */
static void zh_wdt_init(void);

/**
 * @brief Initialize the hardware TWI (I2C) peripheral.
 *
 * Configures PC4 (SDA) and PC5 (SCL) as inputs with internal pull-up
 * enabled, sets the TWI bit-rate register (TWBR) and enables the TWI
 * module in master mode. Note: internal pull-ups are typically too weak
 * for a reliable I2C bus; external 4.7-10 kOhm resistors are recommended.
 * The hardware TWI is used for communication with the BME280 sensor.
 */
static void zh_i2c_init(void);

/**
 * @brief Wait for the TWI interrupt flag with a timeout.
 *
 * Polls the TWINT bit in TWCR until it is set or the 16-bit timeout
 * counter expires. Returns true if the transfer completed successfully.
 *
 * @return true if TWINT was set before the timeout expired
 * @return false on timeout
 */
static bool zh_i2c_wait(void);

/**
 * @brief Generate a TWI start (or repeated start) condition.
 *
 * Asserts START and waits for completion, then verifies the status code
 * (0x08 = start transmitted, 0x10 = repeated start transmitted).
 *
 * @return true when the start condition was acknowledged by the controller
 * @return false on timeout or unexpected status
 */
static bool zh_i2c_start(void);

/**
 * @brief Generate a TWI stop condition.
 *
 * Asserts STOP and waits (bounded by a timeout loop) until the TWSTO
 * bit is cleared by hardware.
 */
static void zh_i2c_stop(void);

/**
 * @brief Write one byte on the TWI bus and return the ACK status.
 *
 * Loads TWDR, triggers transmission and checks the TWI status code.
 * Returns true when the slave acknowledges the byte (statuses 0x18,
 * 0x28 for data transmit, 0x40 for SLA+R transmit).
 *
 * @param data Byte to transmit
 *
 * @return true if the slave acknowledged the byte
 * @return false on timeout or NACK
 */
static bool zh_i2c_write(uint8_t data);

/**
 * @brief Read one byte from the TWI bus and send an ACK after reading.
 *
 * Enables the acknowledge bit (TWEA), triggers the read cycle, verifies
 * the status code (0x50 = data received, ACK returned) and stores the
 * received byte in *out.
 *
 * @param out Pointer to store the received byte (must not be NULL)
 *
 * @return true on success
 * @return false on timeout or unexpected status
 */
static bool zh_i2c_read_ack(uint8_t *out);

/**
 * @brief Read one byte from the TWI bus and send a NACK after reading.
 *
 * Disables the acknowledge bit, triggers the read cycle, verifies the
 * status code (0x58 = data received, NACK returned) and stores the
 * received byte in *out.
 *
 * @param out Pointer to store the received byte (must not be NULL)
 *
 * @return true on success
 * @return false on timeout or unexpected status
 */
static bool zh_i2c_read_nack(uint8_t *out);

/**
 * @brief Write a register value to the BME280 sensor over TWI.
 *
 * Initiates a TWI transaction with the BME280 address, writes the
 * register address followed by the value, and stops the bus. A delay
 * of 2 ms is applied after the write for the sensor's internal timing.
 *
 * @param reg Register address to write
 * @param value Value to write to the register
 *
 * @return true on success
 * @return false if the start/stop condition or any byte transfer failed
 */
static bool zh_bme280_write_reg(uint8_t reg, uint8_t value);

/**
 * @brief Read multiple bytes from a BME280 register over TWI.
 *
 * Performs a repeated-start transaction: writes the register address
 * then reads the specified number of bytes, sending ACK for each byte
 * except the last one, where a NACK is sent. Any TWI error (timeout or
 * unexpected status code) aborts the transaction and returns false.
 *
 * @param reg Register address to read from
 * @param buf Pointer to the buffer where the data will be stored (must not be NULL)
 * @param len Number of bytes to read (must be >= 1)
 *
 * @return true on success
 * @return false if any start condition or byte transfer failed
 */
static bool zh_bme280_read_reg(uint8_t reg, uint8_t *buf, uint8_t len);

/**
 * @brief Read a little-endian unsigned 16-bit value from a byte buffer.
 *
 * Combines two consecutive bytes arithmetically instead of reinterpreting
 * the buffer through a wider pointer type, so the code stays compliant
 * with strict-aliasing rules.
 *
 * @param p Pointer to the first byte (low byte) of the value (must not be NULL)
 *
 * @return Assembled 16-bit unsigned value
 */
static uint16_t zh_read_u16(const uint8_t *p);

/**
 * @brief Read a little-endian signed 16-bit value from a byte buffer.
 *
 * @param p Pointer to the first byte (low byte) of the value (must not be NULL)
 *
 * @return Assembled 16-bit signed value
 */
static int16_t zh_read_s16(const uint8_t *p);

/**
 * @brief Initialize the BME280 sensor and read calibration data.
 *
 * Verifies the device ID, performs a soft reset, reads all calibration
 * registers (33 bytes total), and configures humidity and filter settings.
 * A 100 ms delay is applied after power-up before reading the device ID.
 *
 * @return true if the sensor is present and initialized successfully
 * @return false if the device ID does not match or a register operation fails
 */
static bool zh_bme280_init(void);

/**
 * @brief Read raw sensor data from the BME280.
 *
 * Configures the sensor for a measurement cycle, waits for conversion
 * to complete, then reads 8 bytes of raw data and unpacks the pressure,
 * temperature, and humidity ADC values.
 *
 * @param temperature Pointer to store the raw temperature ADC value (must not be NULL)
 * @param pressure Pointer to store the raw pressure ADC value (must not be NULL)
 * @param humidity Pointer to store the raw humidity ADC value (must not be NULL)
 *
 * @return true on success
 * @return false if the register write or read operation fails
 */
static bool zh_bme280_read(int32_t *temperature, int32_t *pressure, int32_t *humidity);

/**
 * @brief Compensate raw temperature using Bosch calibration coefficients.
 *
 * Applies the 32-bit compensation algorithm defined in the BME280 datasheet
 * using calibration data read during initialization. The result is stored
 * in bme280_t_fine for use by the pressure and humidity compensators.
 *
 * @param adc_t Raw temperature ADC value
 *
 * @return Compensated temperature in degrees Celsius
 */
static float zh_bme280_compensate_temperature(int32_t adc_t);

/**
 * @brief Compensate raw pressure using Bosch calibration coefficients.
 *
 * Applies the 32-bit compensation algorithm defined in the BME280 datasheet.
 * The result is in Pascals. Returns zero if the denominator becomes zero.
 *
 * @param adc_p Raw pressure ADC value
 *
 * @return Compensated pressure in Pascals
 */
static float zh_bme280_compensate_pressure(int32_t adc_p);

/**
 * @brief Compensate raw humidity using Bosch calibration coefficients.
 *
 * Applies the 32-bit compensation algorithm defined in the BME280 datasheet.
 * The result is clamped to the range [0, 100] percent.
 *
 * @param adc_h Raw humidity ADC value
 *
 * @return Compensated relative humidity in percent
 */
static float zh_bme280_compensate_humidity(int32_t adc_h);

/**
 * @brief Initialize the SPI peripheral for nRF24L01 communication.
 *
 * Configures PB2 (CSN), PB3 (MOSI) and PB5 (SCK) as outputs, leaves PB2
 * high (CSN idle), and configures the SPI peripheral for master mode with
 * Fck/16 clock divider (SPR0 = 1, SPI2X = 0). CE (PB1) is configured
 * separately in zh_nrf24_init(). The SS pin is not managed by hardware
 * and must be controlled manually via PB2.
 */
static void zh_spi_init(void);

/**
 * @brief Transfer one byte over SPI and return the received byte.
 *
 * Writes the data to SPDR, waits for the transfer complete flag, then
 * returns the contents of SPDR.
 *
 * @param data Byte to transmit
 *
 * @return Received byte
 */
static uint8_t zh_spi_transfer(uint8_t data);

/**
 * @brief Initialize the nRF24L01 radio module.
 *
 * Configures the CE and IRQ pins, sets up TX and RX addresses, configures
 * the channel, enables automatic retransmission, and enables CRC. The
 * module is left in standby mode with RX and TX interrupts masked.
 */
static void zh_nrf24_init(void);

/**
 * @brief Transmit a packet via the nRF24L01 module.
 *
 * Powers up the radio, clears stale STATUS flags, flushes the TX FIFO,
 * writes the payload, asserts CE high to trigger transmission, waits for
 * the IRQ line, then deasserts CE. After transmission the STATUS flags
 * for TX_DS/MAX_RT/RX_DR are cleared, and the module is powered down.
 *
 * @param buf Pointer to the data buffer to transmit (read-only, must not be NULL)
 * @param size Number of bytes to transmit (must be <= 32)
 */
static void zh_nrf24_send(const uint8_t *buf, uint8_t size);

/**
 * @brief Write a payload to the nRF24L01 using a write command.
 *
 * Asserts CSN low, transfers the command byte, writes the payload data,
 * then deasserts CSN. Returns the status register value.
 *
 * @param cmd SPI command byte for payload write
 * @param buf Pointer to the data buffer (read-only, must not be NULL)
 * @param count Number of bytes to write
 *
 * @return SPI status register value after the command byte
 */
static uint8_t zh_nrf24_write_buffer(uint8_t cmd, const uint8_t *buf, uint8_t count);

/**
 * @brief Write a single byte to an nRF24L01 register.
 *
 * Asserts CSN low, transfers the register address ORed with the write
 * command mask, writes the value, then deasserts CSN.
 *
 * @param reg Register address
 * @param val Value to write
 *
 * @return SPI status register value after the command byte
 */
static uint8_t zh_nrf24_write_register(uint8_t reg, uint8_t val);

/**
 * @brief Write a buffer of data to an nRF24L01 register block.
 *
 * Asserts CSN low, transfers the register address ORed with the write
 * command mask, writes each byte from the buffer, then deasserts CSN.
 *
 * @param reg Register address for the block
 * @param buf Pointer to the data buffer (read-only, must not be NULL)
 * @param count Number of bytes to write
 *
 * @return SPI status register value after the command byte
 */
static uint8_t zh_nrf24_write_buff_register(uint8_t reg, const uint8_t *buf, uint8_t count);

/**
 * @brief Send a command to the nRF24L01 without a data payload.
 *
 * Asserts CSN low, transfers the command byte, then deasserts CSN.
 * Used for commands such as FLUSH_TX, FLUSH_RX, and NOP.
 *
 * @param cmd Command byte to send
 *
 * @return SPI status register value after the command byte
 */
static uint8_t zh_nrf24_send_command(uint8_t cmd);

/**
 * @brief Measure the battery voltage using the AVR ADC.
 *
 * Configures the ADC with AVcc as the reference and selects the internal
 * 1.1 V bandgap as the measured input. The ADC result then represents
 * the AVcc (battery) voltage according to V_bat = 1024 * V_ref / ADC.
 * Takes two conversions; the first is discarded to let the ADC input
 * settle, the second result is returned. Returns zero if the ADC
 * reading is zero.
 *
 * @return Battery voltage in volts
 */
static float zh_get_battery_voltage(void);

static volatile uint8_t wdt_wake_count = 0;
static uint8_t bme280_calib[33] = {0};
static int32_t bme280_t_fine = 0;
static bool bme280_present = false;

zh_rf_sensor_transmitted_data_t transmitted_data = {DEVICE_ID, ZH_RF_SENSOR_BME280, 0, 0, 0, 0, 0};

int main(void)
{
    cli();
    zh_i2c_init();
    bme280_present = zh_bme280_init();
    zh_spi_init();
    zh_nrf24_init();
    zh_wdt_init();
    set_sleep_mode(SLEEP_MODE_PWR_DOWN);
    sei();
    int32_t adc_t = 0;
    int32_t adc_p = 0;
    int32_t adc_h = 0;
    if (bme280_present == true)
    {
        if (zh_bme280_read(&adc_t, &adc_p, &adc_h) == true)
        {
            transmitted_data.battery_voltage = (int16_t)lroundf(zh_get_battery_voltage() * 100.0);
            transmitted_data.value_2 = (int16_t)lroundf(zh_bme280_compensate_temperature(adc_t) + TEMPERATURE_CALIBRATION);
            transmitted_data.value_3 = (int16_t)lroundf(zh_bme280_compensate_pressure(adc_p) * 0.00750062 + PRESSURE_CALIBRATION);
            transmitted_data.value_1 = (int16_t)lroundf(zh_bme280_compensate_humidity(adc_h) + HUMIDITY_CALIBRATION);
            zh_nrf24_send((const uint8_t *)&transmitted_data, sizeof(transmitted_data));
        }
        else
        {
            bme280_present = false;
        }
    }
    for (;;)
    {
        sleep_enable();
        sleep_cpu();
        sleep_disable();
        if (wdt_wake_count >= WDT_WAKE_COUNT)
        {
            wdt_wake_count = 0;
            if (bme280_present == false)
            {
                bme280_present = zh_bme280_init();
            }
            if (bme280_present == true)
            {
                if (zh_bme280_read(&adc_t, &adc_p, &adc_h) == true)
                {
                    transmitted_data.battery_voltage = (int16_t)lroundf(zh_get_battery_voltage() * 100.0);
                    transmitted_data.value_2 = (int16_t)lroundf(zh_bme280_compensate_temperature(adc_t) + TEMPERATURE_CALIBRATION);
                    transmitted_data.value_3 = (int16_t)lroundf(zh_bme280_compensate_pressure(adc_p) * 0.00750062 + PRESSURE_CALIBRATION);
                    transmitted_data.value_1 = (int16_t)lroundf(zh_bme280_compensate_humidity(adc_h) + HUMIDITY_CALIBRATION);
                    zh_nrf24_send((const uint8_t *)&transmitted_data, sizeof(transmitted_data));
                }
                else
                {
                    bme280_present = false;
                }
            }
        }
    }
    return 0;
}

static void zh_wdt_init(void)
{
    cli();
    wdt_reset();
    MCUSR &= ~(1 << WDRF);
    WDTCSR |= (1 << WDCE) | (1 << WDE);
    WDTCSR = (1 << WDIE) | (1 << WDP3) | (1 << WDP0);
    // sei() намеренно не вызывается — глобальные прерывания включает main()
    // после завершения инициализации всех периферийных модулей.
}

ISR(WDT_vect)
{
    WDTCSR |= (1 << WDIE);
    ++wdt_wake_count;
}

static void zh_i2c_init(void)
{
    DDRC &= ~((1 << PC4) | (1 << PC5));
    PORTC |= (1 << PC4) | (1 << PC5);
    TWSR = 0x00;
    TWBR = ZH_TWI_TWBR;
    TWCR = (1 << TWEN);
}

static bool zh_i2c_wait(void)
{
    uint16_t timeout = 65535;
    while (!(TWCR & (1 << TWINT)))
    {
        if (--timeout == 0)
        {
            return false;
        }
    }
    return true;
}

static bool zh_i2c_start(void)
{
    TWCR = (1 << TWINT) | (1 << TWSTA) | (1 << TWEN);
    if (zh_i2c_wait() == false)
    {
        return false;
    }
    uint8_t status = TWSR & 0xF8;
    return (status == 0x08 || status == 0x10);
}

static void zh_i2c_stop(void)
{
    TWCR = (1 << TWINT) | (1 << TWSTO) | (1 << TWEN);
    uint16_t timeout = 1000;
    while ((TWCR & (1 << TWSTO)) != 0)
    {
        if (--timeout == 0)
        {
            break;
        }
    }
}

static bool zh_i2c_write(uint8_t data)
{
    TWDR = data;
    TWCR = (1 << TWINT) | (1 << TWEN);
    if (zh_i2c_wait() == false)
    {
        return false;
    }
    uint8_t status = TWSR & 0xF8;
    return (status == 0x18 || status == 0x28 || status == 0x40);
}

static bool zh_i2c_read_ack(uint8_t *out)
{
    TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWEA);
    if (zh_i2c_wait() == false)
    {
        return false;
    }
    if ((TWSR & 0xF8) != 0x50)
    {
        return false;
    }
    *out = TWDR;
    return true;
}

static bool zh_i2c_read_nack(uint8_t *out)
{
    TWCR = (1 << TWINT) | (1 << TWEN);
    if (zh_i2c_wait() == false)
    {
        return false;
    }
    if ((TWSR & 0xF8) != 0x58)
    {
        return false;
    }
    *out = TWDR;
    return true;
}

static bool zh_bme280_write_reg(uint8_t reg, uint8_t value)
{
    if (zh_i2c_start() == false)
    {
        zh_i2c_stop();
        return false;
    }
    if (zh_i2c_write(BME280_ADDR << 1) == false)
    {
        zh_i2c_stop();
        return false;
    }
    if (zh_i2c_write(reg) == false)
    {
        zh_i2c_stop();
        return false;
    }
    if (zh_i2c_write(value) == false)
    {
        zh_i2c_stop();
        return false;
    }
    zh_i2c_stop();
    _delay_ms(2);
    return true;
}

static bool zh_bme280_read_reg(uint8_t reg, uint8_t *buf, uint8_t len)
{
    if (buf == 0 || len == 0)
    {
        return false;
    }
    if (zh_i2c_start() == false)
    {
        zh_i2c_stop();
        return false;
    }
    if (zh_i2c_write(BME280_ADDR << 1) == false)
    {
        zh_i2c_stop();
        return false;
    }
    if (zh_i2c_write(reg) == false)
    {
        zh_i2c_stop();
        return false;
    }
    if (zh_i2c_start() == false)
    {
        zh_i2c_stop();
        return false;
    }
    if (zh_i2c_write((BME280_ADDR << 1) | 1) == false)
    {
        zh_i2c_stop();
        return false;
    }
    for (uint8_t i = 0; i < len; ++i)
    {
        bool ok;
        if (i == len - 1)
        {
            ok = zh_i2c_read_nack(&buf[i]);
        }
        else
        {
            ok = zh_i2c_read_ack(&buf[i]);
        }
        if (ok == false)
        {
            zh_i2c_stop();
            return false;
        }
    }
    zh_i2c_stop();
    return true;
}

static uint16_t zh_read_u16(const uint8_t *p)
{
    return (uint16_t)p[0] | ((uint16_t)p[1] << 8);
}

static int16_t zh_read_s16(const uint8_t *p)
{
    return (int16_t)zh_read_u16(p);
}

static bool zh_bme280_init(void)
{
    _delay_ms(100);
    uint8_t id = 0;
    if (zh_bme280_read_reg(BME280_REG_ID, &id, sizeof(id)) == false || id != 0x60)
    {
        return false;
    }
    if (zh_bme280_write_reg(BME280_REG_RESET, 0xB6) == false)
    {
        return false;
    }
    _delay_ms(10);
    if (zh_bme280_read_reg(BME280_REG_CALIB00, bme280_calib, 26) == false)
    {
        return false;
    }
    if (zh_bme280_read_reg(BME280_REG_CALIB26, bme280_calib + 26, 7) == false)
    {
        return false;
    }
    if (zh_bme280_write_reg(BME280_REG_CTRL_HUM, 0x01) == false)
    {
        return false;
    }
    if (zh_bme280_write_reg(BME280_REG_CONFIG, 0x00) == false)
    {
        return false;
    }
    return true;
}

static bool zh_bme280_read(int32_t *temperature, int32_t *pressure, int32_t *humidity)
{
    // 1. ОБЯЗАТЕЛЬНО настраиваем влажность ПЕРЕД запуском измерения температуры/давления
    // 0x01 = Oversampling x1 для влажности
    if (zh_bme280_write_reg(BME280_REG_CTRL_HUM, 0x01) == false)
    {
        return false;
    }

    // 2. Включаем Forced mode (0x25: osrs_t x1, osrs_p x1, forced mode)
    if (zh_bme280_write_reg(BME280_REG_CTRL_MEAS, 0x25) == false)
    {
        return false;
    }

    // 3. Ждем окончания измерения. Для x1/x1/x1 по даташиту типичное время ~9.3 мс, 12 мс — с запасом
    _delay_ms(12);

    uint8_t raw[8] = {0};
    if (zh_bme280_read_reg(BME280_REG_DATA, raw, sizeof(raw)) == false)
    {
        return false;
    }

    // 4. Безопасно собираем данные
    *pressure = (int32_t)(((uint32_t)raw[0] << 12) | ((uint32_t)raw[1] << 4) | (raw[2] >> 4));
    *temperature = (int32_t)(((uint32_t)raw[3] << 12) | ((uint32_t)raw[4] << 4) | (raw[5] >> 4));

    // Сначала собираем в беззнаковое 16-битное число, затем расширяем до int32_t
    uint16_t raw_humidity = ((uint16_t)raw[6] << 8) | raw[7];
    *humidity = (int32_t)raw_humidity;

    return true;
}

static float zh_bme280_compensate_temperature(int32_t adc_t)
{
    // 1. Извлекаем калибровку правильно (сборка байтов без нарушения strict aliasing)
    uint16_t dig_t1 = zh_read_u16(&bme280_calib[0]); // dig_T1 по даташиту UNSIGNED
    int16_t dig_t2 = zh_read_s16(&bme280_calib[2]);  // SIGNED
    int16_t dig_t3 = zh_read_s16(&bme280_calib[4]);  // SIGNED

    // 2. Расчет var1
    int32_t var1 = ((((adc_t >> 3) - ((int32_t)dig_t1 << 1))) * dig_t2) >> 11;

    // 3. Расчет var2 с защитой от переполнения (приведение промежуточного значения к int64_t)
    int32_t intermediate = (adc_t >> 4) - (int32_t)dig_t1;
    int32_t var2 = (int32_t)(((((int64_t)intermediate * intermediate) >> 12) * dig_t3) >> 14);

    // 4. Финальный расчет t_fine (обновляем ГЛОБАЛЬНУЮ переменную — она нужна
    //    компенсаторам давления и влажности; локальная переменная здесь недопустима)
    bme280_t_fine = var1 + var2;

    // 5. Расчет и возврат температуры в градусах Цельсия
    return (float)((bme280_t_fine * 5 + 128) >> 8) / 100.0f;
}

static float zh_bme280_compensate_pressure(int32_t adc_p)
{
    // 1. Безопасное извлечение коэффициентов из массива калибровки
    // dig_P1 по даташиту UNSIGNED, остальные SIGNED
    uint16_t dig_p1 = zh_read_u16(&bme280_calib[6]);
    int16_t dig_p2 = zh_read_s16(&bme280_calib[8]);
    int16_t dig_p3 = zh_read_s16(&bme280_calib[10]);
    int16_t dig_p4 = zh_read_s16(&bme280_calib[12]);
    int16_t dig_p5 = zh_read_s16(&bme280_calib[14]);
    int16_t dig_p6 = zh_read_s16(&bme280_calib[16]);
    int16_t dig_p7 = zh_read_s16(&bme280_calib[18]);
    int16_t dig_p8 = zh_read_s16(&bme280_calib[20]);
    int16_t dig_p9 = zh_read_s16(&bme280_calib[22]);

    // 2. Вычисления var1 и var2 с использованием 64-битной математики (защита от переполнения).
    //    Сдвиг влево отрицательных signed-значений формально является UB — используем
    //    умножение на (1LL << N), что эквивалентно и не имеет UB.
    int64_t var1 = ((int64_t)bme280_t_fine) - 128000;
    int64_t var2 = var1 * var1 * (int64_t)dig_p6;
    var2 = var2 + ((var1 * (int64_t)dig_p5) * (1LL << 17));
    var2 = var2 + (((int64_t)dig_p4) * (1LL << 35));
    var1 = ((var1 * var1 * (int64_t)dig_p3) >> 8) + ((var1 * (int64_t)dig_p2) * (1LL << 12));
    var1 = (((((int64_t)1) << 47) + var1)) * ((int64_t)dig_p1) >> 33;

    if (var1 == 0)
    {
        return 0.0f; // Защита от деления на ноль
    }

    // 3. Вычисление базового давления
    int64_t p = 1048576 - adc_p;
    p = (((p * (1LL << 31)) - var2) * 3125) / var1;

    // 4. Финальная компенсация давления
    var1 = (((int64_t)dig_p9) * (p >> 13) * (p >> 13)) >> 25;
    var2 = (((int64_t)dig_p8) * p) >> 19;
    p = ((p + var1 + var2) >> 8) + (((int64_t)dig_p7) * (1LL << 4));

    // Возвращаем давление в Паскалях (Па)
    return (float)p / 256.0f;
}

static float zh_bme280_compensate_humidity(int32_t adc_h)
{
    // 1. Безопасное извлечение коэффициентов.
    // dig_h1 и dig_h3 по даташиту UNSIGNED (uint8_t), dig_h6 - SIGNED (int8_t).
    // Сборку dig_h4/dig_h5 выполняем через умножение на 16 вместо сдвига влево
    // отрицательных signed-значений (UB).
    uint8_t dig_h1 = bme280_calib[25];
    int16_t dig_h2 = zh_read_s16(&bme280_calib[26]);
    uint8_t dig_h3 = bme280_calib[28];
    int16_t dig_h4 = (int16_t)(((int16_t)(int8_t)bme280_calib[29]) * 16 +
                               (int16_t)(bme280_calib[30] & 0x0F));
    int16_t dig_h5 = (int16_t)(((int16_t)(int8_t)bme280_calib[31]) * 16 +
                               (int16_t)((bme280_calib[30] >> 4) & 0x0F));
    int8_t dig_h6 = (int8_t)bme280_calib[32];

    // 2. Вычисления (внедряем int64_t для защиты от переполнения при умножении).
    //    Левые сдвиги отрицательных signed-значений заменены на умножение на (1LL << N).
    int32_t v_x1_u32r = (bme280_t_fine - ((int32_t)76800));

    int64_t sub_expr1 = ((int64_t)adc_h * (1LL << 14)) -
                        ((int64_t)dig_h4 * (1LL << 20)) -
                        ((int64_t)dig_h5 * v_x1_u32r);
    int64_t sub_expr2 = (((int64_t)v_x1_u32r * dig_h6) >> 10) *
                        ((((int64_t)v_x1_u32r * dig_h3) >> 11) + 32768);
    sub_expr2 = ((sub_expr2 >> 10) + 2097152) * (int64_t)dig_h2 + 8192;

    // Соединяем части формулы
    v_x1_u32r = (int32_t)(((sub_expr1 + 16384) >> 15) * (sub_expr2 >> 14));

    // Вторая часть формулы Bosch
    int64_t sub_expr3 = ((int64_t)v_x1_u32r >> 15) * ((int64_t)v_x1_u32r >> 15);
    v_x1_u32r = v_x1_u32r - (int32_t)(((sub_expr3 >> 7) * dig_h1) >> 4);

    // 3. Ограничение диапазонов (по даташиту)
    if (v_x1_u32r < 0)
    {
        v_x1_u32r = 0;
    }
    if (v_x1_u32r > 419430400)
    {
        v_x1_u32r = 419430400;
    }

    // 4. Возвращаем значение в % влажности
    return (float)(v_x1_u32r >> 12) / 1024.0f;
}

static void zh_spi_init(void)
{
    DDRB |= (1 << PB2) | (1 << PB3) | (1 << PB5);
    PORTB |= (1 << PB2);
    SPCR = (1 << SPE) | (1 << MSTR) | (1 << SPR0);
}

static uint8_t zh_spi_transfer(uint8_t data)
{
    SPDR = data;
    while ((SPSR & (1 << SPIF)) == 0)
    {
    }
    return SPDR;
}

static void zh_nrf24_init(void)
{
    _delay_ms(100);
    DDRB |= (1 << PB1);   // CE as output
    PORTB &= ~(1 << PB1); // CE low (standby)
    DDRD &= ~(1 << PD2);  // IRQ as input
    PORTD |= (1 << PD2);  // pull-up on IRQ (nRF24 IRQ is open-drain active-low)

    /* Адрес шлюза в том порядке, в котором он уходит в эфир. Оригинальный код
     * (uint64_t с little-endian AVR) отправлял байты FF EE DD. Если шлюз
     * ожидает DD EE FF, поменяйте порядок байт здесь. */
    static const uint8_t gateway_addr[3] = {0xFF, 0xEE, 0xDD};

    zh_nrf24_write_register(NRF_REG_CONFIG, 0x00);
    zh_nrf24_write_register(NRF_REG_EN_AA, 0x01);
    zh_nrf24_write_register(NRF_REG_EN_RXADDR, 0x01);
    zh_nrf24_write_register(NRF_REG_SETUP_AW, 0x01);
    zh_nrf24_write_register(NRF_REG_SETUP_RETR, 0xFF);
    zh_nrf24_write_register(NRF_REG_RF_CH, GATEWAY_CHANNEL);
    zh_nrf24_write_register(NRF_REG_RF_SETUP, 0x26);
    zh_nrf24_write_buff_register(NRF_REG_TX_ADDR, gateway_addr, 3);
    zh_nrf24_write_buff_register(NRF_REG_RX_ADDR_P0, gateway_addr, 3);
    zh_nrf24_write_register(NRF_REG_STATUS, NRF_STATUS_RX_DR | NRF_STATUS_TX_DS | NRF_STATUS_MAX_RT);
    zh_nrf24_write_register(NRF_REG_CONFIG, NRF_CONFIG_MASK_RX_DR | NRF_CONFIG_EN_CRC);
}

static void zh_nrf24_send(const uint8_t *buf, uint8_t size)
{
    if (buf == 0 || size == 0 || size > 32)
    {
        return;
    }

    zh_nrf24_write_register(NRF_REG_CONFIG,
                            NRF_CONFIG_MASK_RX_DR | NRF_CONFIG_EN_CRC | NRF_CONFIG_PWR_UP);
    _delay_ms(5);

    // Очищаем старые флаги STATUS, чтобы IRQ не оказалась уже низкой из-за
    // прошлых событий — иначе цикл ожидания ниже выйдет мгновенно.
    zh_nrf24_write_register(NRF_REG_STATUS,
                            NRF_STATUS_RX_DR | NRF_STATUS_TX_DS | NRF_STATUS_MAX_RT);

    zh_nrf24_send_command(NRF_CMD_FLUSH_TX);
    zh_nrf24_write_buffer(NRF_CMD_W_TX_PAYLOAD, buf, size);
    PORTB |= (1 << PB1);
    uint8_t count = 0;
    while ((PIND & (1 << PD2)) != 0)
    {
        if (++count > 100)
        {
            break;
        }
        _delay_ms(1);
    }
    PORTB &= ~(1 << PB1);

    // Читаем STATUS (полезно для отладки: NRF_STATUS_TX_DS / NRF_STATUS_MAX_RT),
    // затем сбрасываем только нужные флаги (write-1-to-clear).
    (void)zh_nrf24_send_command(NRF_CMD_NOP);
    zh_nrf24_write_register(NRF_REG_STATUS,
                            NRF_STATUS_RX_DR | NRF_STATUS_TX_DS | NRF_STATUS_MAX_RT);

    zh_nrf24_send_command(NRF_CMD_FLUSH_TX);
    zh_nrf24_write_register(NRF_REG_CONFIG, NRF_CONFIG_MASK_RX_DR | NRF_CONFIG_EN_CRC);
}

static uint8_t zh_nrf24_write_buffer(uint8_t cmd, const uint8_t *buf, uint8_t count)
{
    PORTB &= ~(1 << PB2);
    uint8_t status = zh_spi_transfer(cmd);
    while (count-- != 0)
    {
        zh_spi_transfer(*(buf++));
    }
    PORTB |= (1 << PB2);
    return status;
}

static uint8_t zh_nrf24_write_register(uint8_t reg, uint8_t val)
{
    PORTB &= ~(1 << PB2);
    uint8_t status = zh_spi_transfer(reg | NRF_CMD_W_REGISTER);
    zh_spi_transfer(val);
    PORTB |= (1 << PB2);
    return status;
}

static uint8_t zh_nrf24_write_buff_register(uint8_t reg, const uint8_t *buf, uint8_t count)
{
    return zh_nrf24_write_buffer((reg | NRF_CMD_W_REGISTER), buf, count);
}

static uint8_t zh_nrf24_send_command(uint8_t cmd)
{
    PORTB &= ~(1 << PB2);
    uint8_t status = zh_spi_transfer(cmd);
    PORTB |= (1 << PB2);
    return status;
}

static float zh_get_battery_voltage(void)
{
    // AVcc как опора + внутренний bandgap 1.1 В как вход.
    // Результат ADC = 1024 * 1.1 / V_avcc, поэтому V_avcc = 1024 * 1.1 / ADC.
    ADMUX = (1 << REFS0) | (1 << MUX3) | (1 << MUX2) | (1 << MUX1);
    ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);
    _delay_ms(10);
    ADCSRA |= (1 << ADSC);
    while (ADCSRA & (1 << ADSC))
    {
    }
    ADCSRA |= (1 << ADSC);
    while (ADCSRA & (1 << ADSC))
    {
    }
    uint16_t adc = ADCL;
    adc |= (uint16_t)ADCH << 8;
    ADCSRA &= ~(1 << ADEN);
    if (adc == 0)
    {
        return 0.0;
    }
    return ((1024.0 * 1.1) / (float)adc) * BATTERY_VOLTAGE_CALIBRATION;
}