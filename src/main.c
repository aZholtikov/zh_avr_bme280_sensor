#include "avr/io.h"
#include "avr/interrupt.h"
#include "avr/sleep.h"
#include "avr/eeprom.h"
#include "avr/wdt.h"
#include "util/delay.h"
#include "stdbool.h"
#include "math.h"
#include "zh_nrf24_config.h"

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
#ifndef PRR
#define PRR PRR0
#endif
#ifndef PRTWI
#define PRTWI PRTWI0
#endif
#ifndef PRSPI
#define PRSPI PRSPI0
#endif
#endif

#define GATEWAY_CHANNEL 120
#define GATEWAY_ADDRESS 0xDDEEFF

#define EEPROM_DEVICE_ID_ADDR ((uint8_t *)0x00)
#define DEVICE_ID_UNASSIGNED 0xFF

#define PAIRING_TIMEOUT_ATTEMPTS 7

#define WDT_WAKE_COUNT 38
#define WDT_T_250MS 0x04
#define WDT_T_4S 0x08
#define WDT_T_8S 0x09

#define TWI_TWBR 32

#define NRF_CMD_W_REGISTER 0x20
#define NRF_CMD_W_TX_PAYLOAD 0xA0
#define NRF_CMD_R_RX_PAYLOAD 0x61
#define NRF_CMD_FLUSH_TX 0xE1
#define NRF_CMD_FLUSH_RX 0xE2
#define NRF_CMD_NOP 0xFF
#define NRF_CMD_ACTIVATE 0x50
#define NRF_CMD_R_RX_PL_WID 0x60

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
#define NRF_REG_RX_PW_P0 0x11
#define NRF_REG_DYNPD 0x1C
#define NRF_REG_FEATURE 0x1D

#define NRF_CONFIG_MASK_RX_DR 0x40
#define NRF_CONFIG_MASK_TX_DS 0x20
#define NRF_CONFIG_MASK_MAX_RT 0x10
#define NRF_CONFIG_EN_CRC 0x08
#define NRF_CONFIG_PWR_UP 0x02
#define NRF_CONFIG_PRIM_RX 0x01

#define NRF_STATUS_RX_DR 0x40
#define NRF_STATUS_TX_DS 0x20
#define NRF_STATUS_MAX_RT 0x10

#define NRF_FEATURE_EN_DPL 0x04
#define NRF_DYNPD_DPL_P0 0x01

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
 * @brief Load the device identifier from EEPROM.
 *
 * @return Stored device id, or DEVICE_ID_UNASSIGNED if none is stored
 */
static uint8_t zh_load_device_id(void);

/**
 * @brief Save the device identifier to EEPROM.
 *
 * @param id Device identifier to store
 */
static void zh_save_device_id(uint8_t id);

/**
 * @brief Initialize the SPI peripheral for nRF24L01 communication.
 *
 * Configures PB2 (CE), PB3 (MISO), and PB5 (SCK) as output, sets PB2
 * high, and configures the SPI peripheral for master mode with Fck/16
 * clock divider. The SS pin is not managed by hardware and must be
 * controlled manually via PB2.
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
 * @brief Initialize the hardware TWI (I2C) peripheral.
 *
 * Configures PC4 (SDA) and PC5 (SCL) as inputs with pull-up enabled,
 * sets the TWI bit-rate register (TWBR) and enables the TWI module
 * in master mode. The hardware TWI is used for communication with
 * the BME280 sensor.
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
 * Enables the acknowledge bit (TWEA), triggers the read cycle and
 * returns the contents of TWDR. Returns 0xFF on timeout.
 *
 * @return Received byte
 */
static uint8_t zh_i2c_read_ack(void);

/**
 * @brief Read one byte from the TWI bus and send a NACK after reading.
 *
 * Disables the acknowledge bit, triggers the read cycle and returns
 * the contents of TWDR. Returns 0xFF on timeout.
 *
 * @return Received byte
 */
static uint8_t zh_i2c_read_nack(void);

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
 * except the last one, where a NACK is sent.
 *
 * @param reg Register address to read from
 * @param buf Pointer to the buffer where the data will be stored (must not be NULL)
 * @param len Number of bytes to read
 *
 * @return true on success
 * @return false if any start condition or byte transfer failed
 */
static bool zh_bme280_read_reg(uint8_t reg, uint8_t *buf, uint8_t len);

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
 * @brief Initialize the nRF24L01 radio module.
 *
 * Configures the CE and IRQ pins, sets up TX and RX addresses, configures
 * the channel, enables automatic retransmission, and enables CRC. The
 * module is left in standby mode with RX and TX interrupts masked.
 */
static void zh_nrf24_init(void);

/**
 * @brief Initialize INT0 external interrupt for the nRF24 IRQ signal.
 *
 * Configures PD2 as a falling-edge triggered external interrupt source and
 * enables the INT0 interrupt mask bit. The ISR sets the nrf_event flag used
 * by the pairing state machine.
 */
static void zh_nrf24_interrupt_init(void);

/**
 * @brief Transmit a packet via the nRF24L01 module.
 *
 * Powers up the radio, flushes the TX FIFO, writes the payload, asserts
 * CE high to trigger transmission, waits for the IRQ line, then deasserts
 * CE. The status register is read, cleared, and the TX FIFO is flushed
 * again after transmission.
 *
 * @param buf Pointer to the data buffer to transmit (must not be NULL)
 * @param size Number of bytes to transmit
 */
static void zh_nrf24_send(uint8_t *buf, uint8_t size);

/**
 * @brief Start a transmission without blocking on completion.
 *
 * Loads the payload, pulses CE for 15 us and returns immediately. The
 * transmission completion is signalled by the nRF24 IRQ pin.
 *
 * @param buf Pointer to the payload data to transmit (must not be NULL)
 * @param size Number of bytes in the payload buffer (must be > 0)
 */
static void zh_nrf24_send_no_wait(uint8_t *buf, uint8_t size);

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
 * @brief Write a payload to the nRF24L01 using a write command.
 *
 * Asserts CSN low, transfers the command byte, writes the payload data,
 * then deasserts CSN. Returns the status register value.
 *
 * @param cmd SPI command byte for payload write
 * @param buf Pointer to the data buffer (must not be NULL)
 * @param count Number of bytes to write
 *
 * @return SPI status register value after the command byte
 */
static uint8_t zh_nrf24_write_buffer(uint8_t cmd, uint8_t *buf, uint8_t count);

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
 * @param buf Pointer to the data buffer (must not be NULL)
 * @param count Number of bytes to write
 *
 * @return SPI status register value after the command byte
 */
static uint8_t zh_nrf24_write_buff_register(uint8_t reg, uint8_t *buf, uint8_t count);

/**
 * @brief Read the received payload from the nRF24L01+ into a buffer.
 *
 * @param buf Pointer to the destination buffer (must not be NULL)
 * @param count Number of bytes to read from the RX FIFO
 */
static void zh_nrf24_read_payload(uint8_t *buf, uint8_t count);

/**
 * @brief Read the width of the payload in the nRF24L01+ RX FIFO.
 *
 * @return Payload width in bytes
 */
static uint8_t zh_nrf24_read_payload_width(void);

/**
 * @brief Switch the nRF24L01+ to RX mode on pipe 0.
 */
static void zh_nrf24_enter_rx(void);

/**
 * @brief Put the nRF24L01+ into power-down mode.
 */
static void zh_nrf24_enter_power_down(void);

/**
 * @brief Start the watchdog timer in interrupt mode with the given timeout.
 *
 * @param timeout_bits Watchdog timeout prescaler bits (WDP3:0)
 */
static void zh_wdt_start(uint8_t timeout_bits);

/**
 * @brief Stop the watchdog timer.
 */
static void zh_wdt_stop(void);

/**
 * @brief Generate a pseudo-random token from MCU state and ADC noise.
 *
 * @return 16-bit random token (never zero)
 */
static uint16_t zh_get_random_token(void);

/**
 * @brief Run the non-blocking pairing procedure to obtain a device id.
 *
 * Sends pairing requests, waits for the gateway acknowledgment in RX mode
 * and retries with watchdog-timed backoff. Sleeps forever if the gateway
 * does not answer within the allowed number of attempts.
 *
 * @return Device identifier assigned by the gateway
 */
static uint8_t zh_do_pairing(void);

/**
 * @brief Enter permanent power-down sleep until the next reset.
 *
 * Disables all interrupts, the watchdog, the radio, the ADC, SPI and all
 * unused peripheral clocks, then loops in power-down sleep mode.
 */
static void zh_enter_final_sleep(void);

/**
 * @brief Measure the battery voltage using the AVR ADC.
 *
 * Configures the ADC with the internal 1.1 V reference and selects the
 * 1.1 V bandgap input. Takes two conversions and returns the average
 * as a voltage value. Returns zero if the ADC reading is zero.
 *
 * @return Battery voltage in volts
 */
static float zh_get_battery_voltage(void);

static volatile uint8_t wdt_wake_count = 0;
static volatile uint8_t nrf_event = false;
static uint8_t bme280_calib[33] = {0};
static int32_t bme280_t_fine = 0;
static bool bme280_present = false;

zh_nrf24_sensor_transmitted_data_t transmitted_data = {0};

int main(void)
{
    transmitted_data.type = ZH_NRF24_SENSOR_BME280;
    MCUSR &= ~(1 << WDRF);
    wdt_disable();
    cli();
    zh_i2c_init();
    bme280_present = zh_bme280_init();
    zh_spi_init();
    zh_nrf24_init();
    set_sleep_mode(SLEEP_MODE_PWR_DOWN);
    sei();
    uint8_t device_id = zh_load_device_id();
    if (device_id == DEVICE_ID_UNASSIGNED)
    {
        device_id = zh_do_pairing();
    }
    transmitted_data.id = (int16_t)device_id;
    {
        uint8_t probe[ZH_NRF24_MAX_PAYLOAD_SIZE];
        if (zh_nrf24_sensor_build_payload(&transmitted_data, probe, sizeof(probe)) == 0)
        {
            zh_enter_final_sleep();
        }
    }
    zh_wdt_start(WDT_T_8S);
    int32_t adc_t = 0;
    int32_t adc_p = 0;
    int32_t adc_h = 0;
    if (bme280_present == true)
    {
        if (zh_bme280_read(&adc_t, &adc_p, &adc_h) == true)
        {
            transmitted_data.battery = (int16_t)lroundf(zh_get_battery_voltage() * 100.0);
            transmitted_data.value_2 = (int16_t)lroundf(zh_bme280_compensate_temperature(adc_t) * 10.0);
            transmitted_data.value_3 = (int16_t)lroundf(zh_bme280_compensate_pressure(adc_p) * 0.00750062 * 10.0);
            transmitted_data.value_1 = (int16_t)lroundf(zh_bme280_compensate_humidity(adc_h) * 10.0);
            uint8_t payload[ZH_NRF24_MAX_PAYLOAD_SIZE];
            uint8_t size = zh_nrf24_sensor_build_payload(&transmitted_data, payload, sizeof(payload));
            if (size > 0)
            {
                zh_nrf24_send(payload, size);
            }
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
                    transmitted_data.battery = (int16_t)lroundf(zh_get_battery_voltage() * 100.0);
                    transmitted_data.value_2 = (int16_t)lroundf(zh_bme280_compensate_temperature(adc_t));
                    transmitted_data.value_3 = (int16_t)lroundf(zh_bme280_compensate_pressure(adc_p) * 0.00750062);
                    transmitted_data.value_1 = (int16_t)lroundf(zh_bme280_compensate_humidity(adc_h));
                    uint8_t payload[ZH_NRF24_MAX_PAYLOAD_SIZE];
                    uint8_t size = zh_nrf24_sensor_build_payload(&transmitted_data, payload, sizeof(payload));
                    if (size > 0)
                    {
                        zh_nrf24_send(payload, size);
                    }
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

ISR(WDT_vect)
{
    WDTCSR |= (1 << WDIE);
    if (wdt_wake_count < WDT_WAKE_COUNT)
    {
        ++wdt_wake_count;
    }
}

ISR(INT0_vect)
{
    nrf_event = true;
}

static uint8_t zh_load_device_id(void)
{
    uint8_t id = eeprom_read_byte(EEPROM_DEVICE_ID_ADDR);
    if (id == 0xFF || id == 0x00)
    {
        return DEVICE_ID_UNASSIGNED;
    }
    return id;
}

static void zh_save_device_id(uint8_t id)
{
    eeprom_update_byte(EEPROM_DEVICE_ID_ADDR, id);
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

static void zh_i2c_init(void)
{
    DDRC &= ~((1 << PC4) | (1 << PC5));
    PORTC |= (1 << PC4) | (1 << PC5);
    TWSR = 0x00;
    TWBR = TWI_TWBR;
    TWCR = (1 << TWEN);
}

static bool zh_i2c_wait(void)
{
    uint16_t timeout = 65535;
    while ((TWCR & (1 << TWINT)) == 0)
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
    while ((TWCR & (1 << TWSTO)) != 0 && --timeout)
    {
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

static uint8_t zh_i2c_read_ack(void)
{
    TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWEA);
    if (zh_i2c_wait() == false)
    {
        return 0xFF;
    }
    return TWDR;
}

static uint8_t zh_i2c_read_nack(void)
{
    TWCR = (1 << TWINT) | (1 << TWEN);
    if (zh_i2c_wait() == false)
    {
        return 0xFF;
    }
    return TWDR;
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
        buf[i] = (i == len - 1) ? zh_i2c_read_nack() : zh_i2c_read_ack();
    }
    zh_i2c_stop();
    return true;
}

static bool zh_bme280_read(int32_t *temperature, int32_t *pressure, int32_t *humidity)
{
    if (zh_bme280_write_reg(BME280_REG_CTRL_MEAS, 0x25) == false)
    {
        return false;
    }
    _delay_ms(10);
    uint8_t raw[8] = {0};
    if (zh_bme280_read_reg(BME280_REG_DATA, raw, sizeof(raw)) == false)
    {
        return false;
    }
    *pressure = (int32_t)(((uint32_t)raw[0] << 12) | ((uint32_t)raw[1] << 4) | (raw[2] >> 4));
    *temperature = (int32_t)(((uint32_t)raw[3] << 12) | ((uint32_t)raw[4] << 4) | (raw[5] >> 4));
    *humidity = (int32_t)(((uint32_t)raw[6] << 8) | (uint32_t)raw[7]);
    return true;
}

static float zh_bme280_compensate_temperature(int32_t adc_t)
{
    int32_t dig_t1 = (int32_t)(uint16_t)(bme280_calib[1] << 8 | bme280_calib[0]);
    int32_t dig_t2 = (int16_t)(bme280_calib[3] << 8 | bme280_calib[2]);
    int32_t dig_t3 = (int16_t)(bme280_calib[5] << 8 | bme280_calib[4]);
    int32_t var1 = ((((adc_t >> 3) - (dig_t1 << 1))) * dig_t2) >> 11;
    int32_t var2 = (((((adc_t >> 4) - dig_t1) * ((adc_t >> 4) - dig_t1)) >> 12) * dig_t3) >> 14;
    bme280_t_fine = var1 + var2;
    return (float)((bme280_t_fine * 5 + 128) >> 8) / 100.0;
}

static float zh_bme280_compensate_pressure(int32_t adc_p)
{
    int32_t dig_p1 = (int32_t)(uint16_t)(bme280_calib[7] << 8 | bme280_calib[6]);
    int32_t dig_p2 = (int16_t)(bme280_calib[9] << 8 | bme280_calib[8]);
    int32_t dig_p3 = (int16_t)(bme280_calib[11] << 8 | bme280_calib[10]);
    int32_t dig_p4 = (int16_t)(bme280_calib[13] << 8 | bme280_calib[12]);
    int32_t dig_p5 = (int16_t)(bme280_calib[15] << 8 | bme280_calib[14]);
    int32_t dig_p6 = (int16_t)(bme280_calib[17] << 8 | bme280_calib[16]);
    int32_t dig_p7 = (int16_t)(bme280_calib[19] << 8 | bme280_calib[18]);
    int32_t dig_p8 = (int16_t)(bme280_calib[21] << 8 | bme280_calib[20]);
    int32_t dig_p9 = (int16_t)(bme280_calib[23] << 8 | bme280_calib[22]);
    int32_t var1 = (((int32_t)bme280_t_fine) >> 1) - 64000;
    int32_t var2 = (((var1 >> 2) * (var1 >> 2)) >> 11) * dig_p6;
    var2 = var2 + ((var1 * dig_p5) << 1);
    var2 = (var2 >> 2) + (((int32_t)dig_p4) << 16);
    var1 = (((dig_p3 * (((var1 >> 2) * (var1 >> 2)) >> 13)) >> 3) + (((int32_t)dig_p2 * var1) >> 1)) >> 18;
    var1 = ((((32768 + var1)) * dig_p1) >> 15);
    if (var1 == 0)
    {
        return 0.0;
    }
    uint32_t p = (((uint32_t)(((int32_t)1048576) - adc_p) - (var2 >> 12))) * 3125;
    if (p < 0x80000000)
    {
        p = (p << 1) / ((uint32_t)var1);
    }
    else
    {
        p = (p / (uint32_t)var1) * 2;
    }
    var1 = (((int32_t)dig_p9) * ((int32_t)(((p >> 3) * (p >> 3)) >> 13))) >> 12;
    var2 = (((int32_t)(p >> 2)) * ((int32_t)dig_p8)) >> 13;
    return (float)((int32_t)p + ((var1 + var2 + dig_p7) >> 4));
}

static float zh_bme280_compensate_humidity(int32_t adc_h)
{
    int32_t dig_h1 = bme280_calib[25];
    int32_t dig_h2 = (int16_t)(bme280_calib[27] << 8 | bme280_calib[26]);
    int32_t dig_h3 = bme280_calib[28];
    int32_t dig_h4 = (int16_t)((bme280_calib[29] << 4) | (bme280_calib[30] & 0x0F));
    int32_t dig_h5 = (int16_t)((bme280_calib[31] << 4) | ((bme280_calib[30] >> 4) & 0x0F));
    int8_t dig_h6 = (int8_t)bme280_calib[32];
    int32_t v_x1_u32r = (bme280_t_fine - ((int32_t)76800));
    v_x1_u32r = (((((adc_h << 14) - (dig_h4 << 20) - (dig_h5 * v_x1_u32r)) + 16384) >> 15) * (((((((v_x1_u32r * dig_h6) >> 10) * (((v_x1_u32r * dig_h3) >> 11) + 32768)) >> 10) + 2097152) * dig_h2 + 8192) >> 14));
    v_x1_u32r = (v_x1_u32r - (((((v_x1_u32r >> 15) * (v_x1_u32r >> 15)) >> 7) * dig_h1) >> 4));
    if (v_x1_u32r < 0)
    {
        v_x1_u32r = 0;
    }
    if (v_x1_u32r > 419430400)
    {
        v_x1_u32r = 419430400;
    }
    return (float)(v_x1_u32r >> 12) / 1024.0;
}

static void zh_nrf24_init(void)
{
    _delay_ms(100);
    DDRB |= (1 << PB1);
    PORTB &= ~(1 << PB1);
    DDRD &= ~(1 << PD2);
    PORTD |= (1 << PD2);
    uint64_t gateway_address = GATEWAY_ADDRESS;
    uint8_t activation_key = 0x73;
    zh_nrf24_write_register(NRF_REG_CONFIG, 0x00);
    zh_nrf24_write_register(NRF_REG_EN_AA, 0x01);
    zh_nrf24_write_register(NRF_REG_EN_RXADDR, 0x01);
    zh_nrf24_write_register(NRF_REG_SETUP_AW, 0x01);
    zh_nrf24_write_register(NRF_REG_SETUP_RETR, 0xFF);
    zh_nrf24_write_register(NRF_REG_RF_CH, GATEWAY_CHANNEL);
    zh_nrf24_write_register(NRF_REG_RF_SETUP, 0x26);
    zh_nrf24_write_buff_register(NRF_REG_TX_ADDR, (uint8_t *)&gateway_address, 3);
    zh_nrf24_write_buff_register(NRF_REG_RX_ADDR_P0, (uint8_t *)&gateway_address, 3);
    zh_nrf24_write_register(NRF_REG_STATUS, NRF_STATUS_RX_DR | NRF_STATUS_TX_DS | NRF_STATUS_MAX_RT);
    zh_nrf24_write_buffer(NRF_CMD_ACTIVATE, &activation_key, sizeof(activation_key));
    zh_nrf24_write_register(NRF_REG_FEATURE, NRF_FEATURE_EN_DPL);
    zh_nrf24_write_register(NRF_REG_DYNPD, NRF_DYNPD_DPL_P0);
    zh_nrf24_write_register(NRF_REG_CONFIG, NRF_CONFIG_MASK_RX_DR | NRF_CONFIG_EN_CRC);
}

static void zh_nrf24_interrupt_init(void)
{
    DDRD &= ~(1 << PD2);
    PORTD |= (1 << PD2);
    EICRA &= ~((1 << ISC01) | (1 << ISC00));
    EICRA |= (1 << ISC01);
    EIFR |= (1 << INTF0);
    EIMSK |= (1 << INT0);
}

static void zh_nrf24_send(uint8_t *buf, uint8_t size)
{
    zh_nrf24_write_register(NRF_REG_CONFIG, NRF_CONFIG_MASK_RX_DR | NRF_CONFIG_EN_CRC | NRF_CONFIG_PWR_UP);
    _delay_ms(5);
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
    uint8_t status = zh_nrf24_send_command(NRF_CMD_NOP);
    zh_nrf24_write_register(NRF_REG_STATUS, status);
    zh_nrf24_send_command(NRF_CMD_FLUSH_TX);
    zh_nrf24_write_register(NRF_REG_CONFIG, NRF_CONFIG_MASK_RX_DR | NRF_CONFIG_EN_CRC);
}

static void zh_nrf24_send_no_wait(uint8_t *buf, uint8_t size)
{
    PORTB &= ~(1 << PB1);
    zh_nrf24_write_register(NRF_REG_CONFIG, NRF_CONFIG_MASK_RX_DR | NRF_CONFIG_EN_CRC | NRF_CONFIG_PWR_UP);
    _delay_ms(2);
    zh_nrf24_send_command(NRF_CMD_FLUSH_TX);
    zh_nrf24_write_register(NRF_REG_STATUS, NRF_STATUS_TX_DS | NRF_STATUS_MAX_RT | NRF_STATUS_RX_DR);
    zh_nrf24_write_buffer(NRF_CMD_W_TX_PAYLOAD, buf, size);
    PORTB |= (1 << PB1);
    _delay_us(15);
    PORTB &= ~(1 << PB1);
}

static uint8_t zh_nrf24_send_command(uint8_t cmd)
{
    PORTB &= ~(1 << PB2);
    uint8_t status = zh_spi_transfer(cmd);
    PORTB |= (1 << PB2);
    return status;
}

static uint8_t zh_nrf24_write_buffer(uint8_t cmd, uint8_t *buf, uint8_t count)
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

static uint8_t zh_nrf24_write_buff_register(uint8_t reg, uint8_t *buf, uint8_t count)
{
    return zh_nrf24_write_buffer((reg | NRF_CMD_W_REGISTER), buf, count);
}

static void zh_nrf24_read_payload(uint8_t *buf, uint8_t count)
{
    PORTB &= ~(1 << PB2);
    zh_spi_transfer(NRF_CMD_R_RX_PAYLOAD);
    while (count-- != 0)
    {
        *buf++ = zh_spi_transfer(0xFF);
    }
    PORTB |= (1 << PB2);
}

static uint8_t zh_nrf24_read_payload_width(void)
{
    PORTB &= ~(1 << PB2);
    zh_spi_transfer(NRF_CMD_R_RX_PL_WID);
    uint8_t width = zh_spi_transfer(0xFF);
    PORTB |= (1 << PB2);
    return width;
}

static void zh_nrf24_enter_rx(void)
{
    PORTB &= ~(1 << PB1);
    zh_nrf24_write_register(NRF_REG_STATUS, NRF_STATUS_TX_DS | NRF_STATUS_MAX_RT | NRF_STATUS_RX_DR);
    zh_nrf24_send_command(NRF_CMD_FLUSH_RX);
    zh_nrf24_write_register(NRF_REG_CONFIG, NRF_CONFIG_EN_CRC | NRF_CONFIG_PWR_UP | NRF_CONFIG_PRIM_RX);
    _delay_ms(2);
    PORTB |= (1 << PB1);
    _delay_us(130);
}

static void zh_nrf24_enter_power_down(void)
{
    PORTB &= ~(1 << PB1);
    zh_nrf24_write_register(NRF_REG_CONFIG, NRF_CONFIG_EN_CRC);
}

static void zh_wdt_start(uint8_t timeout_bits)
{
    uint8_t wdp = (timeout_bits & 0x07) | ((timeout_bits & 0x08) ? (1 << WDP3) : 0);
    cli();
    wdt_reset();
    WDTCSR = (1 << WDCE) | (1 << WDE);
    WDTCSR = (1 << WDIE) | wdp;
    sei();
}

static void zh_wdt_stop(void)
{
    cli();
    wdt_reset();
    WDTCSR = (1 << WDCE) | (1 << WDE);
    WDTCSR = 0;
    sei();
}

static uint16_t zh_get_random_token(void)
{
    uint16_t token = MCUSR;
    MCUSR = 0;
    ADMUX = (1 << REFS0);
    ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);
    _delay_ms(5);
    for (uint8_t i = 0; i < 16; ++i)
    {
        ADCSRA |= (1 << ADSC);
        while ((ADCSRA & (1 << ADSC)) != 0)
        {
        }
        token = (token << 1) ^ (token >> 15) ^ ADC;
    }
    ADCSRA &= ~(1 << ADEN);
    if (token == 0)
    {
        token = 0xACE1;
    }
    return token;
}

static uint8_t zh_do_pairing(void)
{
    uint16_t token = zh_get_random_token();
    zh_nrf24_sensor_pairing_data_t pairing_request;
    pairing_request.type = (uint8_t)transmitted_data.type;
    pairing_request.magic = ZH_NRF24_PAIRING_REQ_MAGIC;
    pairing_request.token_hi = (uint8_t)(token >> 8);
    pairing_request.token_lo = (uint8_t)(token & 0xFF);
    pairing_request.id = 0;
    zh_nrf24_interrupt_init();
    zh_nrf24_sensor_pairing_state_t state = ZH_NRF24_PAIR_SEND;
    uint8_t attempts = 0;
    for (;;)
    {
        switch (state)
        {
        case ZH_NRF24_PAIR_SEND:
            nrf_event = false;
            zh_nrf24_send_no_wait((uint8_t *)&pairing_request, ZH_NRF24_PAIRING_PACKET_SIZE);
            state = ZH_NRF24_PAIR_TX_WAIT;
            break;
        case ZH_NRF24_PAIR_TX_WAIT:
            if ((PIND & (1 << PD2)) == 0)
            {
                nrf_event = true;
            }
            else
            {
                nrf_event = false;
                zh_wdt_start(WDT_T_250MS);
                sleep_enable();
                sei();
                sleep_cpu();
                sleep_disable();
                zh_wdt_stop();
                sei();
            }
            if (nrf_event == true)
            {
                nrf_event = false;
                uint8_t status = zh_nrf24_send_command(NRF_CMD_NOP);
                zh_nrf24_write_register(NRF_REG_STATUS, NRF_STATUS_TX_DS | NRF_STATUS_MAX_RT | NRF_STATUS_RX_DR);
                if ((status & NRF_STATUS_TX_DS) != 0)
                {
                    zh_nrf24_enter_rx();
                    state = ZH_NRF24_PAIR_RX_WAIT;
                }
                else
                {
                    state = ZH_NRF24_PAIR_BACKOFF;
                }
            }
            else
            {
                state = ZH_NRF24_PAIR_BACKOFF;
            }
            break;
        case ZH_NRF24_PAIR_RX_WAIT:
            if ((PIND & (1 << PD2)) == 0)
            {
                nrf_event = true;
            }
            else
            {
                nrf_event = false;
                zh_wdt_start(WDT_T_250MS);
                sleep_enable();
                sei();
                sleep_cpu();
                sleep_disable();
                zh_wdt_stop();
                sei();
            }
            if (nrf_event == true)
            {
                nrf_event = false;
                uint8_t status = zh_nrf24_send_command(NRF_CMD_NOP);
                if ((status & NRF_STATUS_RX_DR) != 0)
                {
                    uint8_t width = zh_nrf24_read_payload_width();
                    if (width > ZH_NRF24_MAX_PAYLOAD_SIZE)
                    {
                        zh_nrf24_send_command(NRF_CMD_FLUSH_RX);
                        zh_nrf24_write_register(NRF_REG_STATUS, NRF_STATUS_RX_DR);
                        state = ZH_NRF24_PAIR_BACKOFF;
                        break;
                    }
                    if (width != ZH_NRF24_PAIRING_PACKET_SIZE)
                    {
                        uint8_t dummy[ZH_NRF24_MAX_PAYLOAD_SIZE];
                        zh_nrf24_read_payload(dummy, width);
                        zh_nrf24_write_register(NRF_REG_STATUS, NRF_STATUS_RX_DR);
                        zh_nrf24_send_command(NRF_CMD_FLUSH_RX);
                        state = ZH_NRF24_PAIR_BACKOFF;
                        break;
                    }
                    zh_nrf24_sensor_pairing_data_t pairing_response;
                    zh_nrf24_read_payload((uint8_t *)&pairing_response, width);
                    zh_nrf24_write_register(NRF_REG_STATUS, NRF_STATUS_RX_DR | NRF_STATUS_TX_DS | NRF_STATUS_MAX_RT);
                    zh_nrf24_send_command(NRF_CMD_FLUSH_RX);
                    if (pairing_response.magic == ZH_NRF24_PAIRING_ACK_MAGIC && pairing_response.type == pairing_request.type &&
                        pairing_response.token_hi == pairing_request.token_hi && pairing_response.token_lo == pairing_request.token_lo &&
                        pairing_response.id != 0 && pairing_response.id != DEVICE_ID_UNASSIGNED)
                    {
                        zh_nrf24_enter_power_down();
                        EIMSK &= ~(1 << INT0);
                        zh_save_device_id(pairing_response.id);
                        return pairing_response.id;
                    }
                }
                state = ZH_NRF24_PAIR_BACKOFF;
            }
            else
            {
                state = ZH_NRF24_PAIR_BACKOFF;
            }
            break;
        case ZH_NRF24_PAIR_BACKOFF:
            zh_nrf24_enter_power_down();
            nrf_event = false;
            zh_wdt_start(WDT_T_4S);
            sleep_enable();
            sei();
            sleep_cpu();
            sleep_disable();
            zh_wdt_stop();
            sei();
            if (++attempts >= PAIRING_TIMEOUT_ATTEMPTS)
            {
                zh_enter_final_sleep();
            }
            state = ZH_NRF24_PAIR_SEND;
            break;
        }
    }
}

static void zh_enter_final_sleep(void)
{
    cli();
    EIMSK = 0;
    EIFR = 0xFF;
    zh_wdt_stop();
    PORTB &= ~(1 << PB1);
    zh_nrf24_write_register(NRF_REG_CONFIG, 0x00);
    ADCSRA = 0;
    TWCR = 0;
    SPCR = 0;
    PRR = (1 << PRTWI) | (1 << PRTIM2) | (1 << PRTIM1) | (1 << PRTIM0) | (1 << PRSPI) | (1 << PRUSART0);
    set_sleep_mode(SLEEP_MODE_PWR_DOWN);
    sleep_enable();
    for (;;)
    {
        sleep_cpu();
    }
}

static float zh_get_battery_voltage(void)
{
    ADMUX = (1 << REFS0) | (1 << MUX3) | (1 << MUX2) | (1 << MUX1);
    ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);
    _delay_ms(10);
    ADCSRA |= (1 << ADSC);
    while ((ADCSRA & (1 << ADSC)) != 0)
    {
    }
    ADCSRA |= (1 << ADSC);
    while ((ADCSRA & (1 << ADSC)) != 0)
    {
    }
    uint16_t adc = ADCL;
    adc |= (uint16_t)ADCH << 8;
    ADCSRA &= ~(1 << ADEN);
    if (adc == 0)
    {
        return 0.0;
    }
    return (1024.0 * 1.1) / (float)adc;
}
