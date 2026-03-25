#pragma once
#include <QuickI2C.h>
#include <driver/i2c.h>
#include <math.h>

/**
 * Usage example:
 *
 *     GZP6895<> sensor(I2C_NUM_0);
 *     sensor.Initialize();
 *     auto pressure = sensor.readPressure();
 *     if (pressure) {
 *         printf("Pressure: %ld Pa\n", *pressure);
 *     }
 *
 *     auto temperature = sensor.readTemperature();
 *     if (temperature) {
 *         printf("Temperature register: %u\n", *temperature);
 *     }
 */

template<
    // Read filters
    Postprocessor8Bit postprocessRead8=noop,
    Postprocessor16Bit postprocessRead16=invertByteorder16,
    Postprocessor24Bit postprocessRead24=invertByteorder24,
    Postprocessor32Bit postprocessRead32=invertByteorder32,
    // Write filters
    Postprocessor8Bit postprocessWrite8=noop,
    Postprocessor16Bit postprocessWrite16=invertByteorder16,
    Postprocessor24Bit postprocessWrite24=invertByteorder24,
    Postprocessor32Bit postprocessWrite32=invertByteorder32
>
class GZP6895 : public QuickI2CDevice {
public:
    inline GZP6895(i2c_port_t port): QuickI2CDevice(0x6D, /* 0x91 for write */ port, 100000) {}

    QUICKI2C_DEFINE_REGISTER8_RO(PressureMSB, 0x06) {};
    QUICKI2C_DEFINE_REGISTER8_RO(PressureCSB, 0x07) {};
    QUICKI2C_DEFINE_REGISTER8_RO(PressureLSB, 0x08) {};

    QUICKI2C_DEFINE_REGISTER8_RO(TemperatureMSB, 0x09) {};
    QUICKI2C_DEFINE_REGISTER8_RO(TemperatureLSB, 0x0A) {};

    QUICKI2C_DEFINE_REGISTER8_RW(Command, 0x30, 0x30) {
        MeasureTemperatureOnly = 0b000 << 0,
        MeasurePressureOnly = 0b001 << 0,
        MeasureTemperatureThenPressure = 0b010 << 0,
        AutoMeasure = 0b011 << 0, // Measure temperature and pressure continuously

        // "sco" in the datasheet
        StartDataAcquisition = 0b1 << 3,
        StopDataAcquisition = 0b0 << 3,

        SleepTime62p5ms = 0b0001 << 4,
        SleepTime125ms = 0b0010 << 4,
        SleepTime187p5ms = 0b0011 << 4,
        SleepTime250ms = 0b0100 << 4,
        SleepTime312p5ms = 0b0101 << 4,
        SleepTime375ms = 0b0110 << 4,
        SleepTime437p5ms = 0b0111 << 4,
        SleepTime500ms = 0b1000 << 4,
        SleepTime562p5ms = 0b1001 << 4,
        SleepTime625ms = 0b1010 << 4,
        SleepTime687p5ms = 0b1011 << 4,
        SleepTime750ms = 0b1100 << 4,
        SleepTime812p5ms = 0b1101 << 4,
        SleepTime875ms = 0b1110 << 4,
        SleepTime937p5ms = 0b1111 << 4,
    };
    

    // NOTE: Defined as RO because they are OTP only
    QUICKI2C_DEFINE_REGISTER8_RO(SysConfig, 0xA5) {};
    QUICKI2C_DEFINE_REGISTER8_RO(PConfig, 0xA5) {};

    /**
     * @brief Initialize sensor in auto-measure mode.
     *
     * Sets auto measurement, starts data acquisition, and configures sleep interval.
     * @return QuickI2CStatus
     */
    QuickI2CStatus Initialize() {
        return writeAndVerifyCommand(
            GZP6895<>::Command::AutoMeasure |
            GZP6895<>::Command::StartDataAcquisition |
            GZP6895<>::Command::SleepTime62p5ms
        );
    }

    /**
     * @brief Read 24-bit pressure value and return signed 32-bit result.
     *
     * Combines MSB/CSB/LSB registers and sign-extends 24-bit range.
     */
    tl::expected<int32_t, QuickI2CStatus> readPressure() {
        auto msb = readPressureMSB();
        if(!msb) {
            return tl::make_unexpected(msb.error());
        }
        auto csb = readPressureCSB();
        if(!csb) {
            return tl::make_unexpected(csb.error());
        }
        auto lsb = readPressureLSB();
        if(!lsb) {
            return tl::make_unexpected(lsb.error());
        }

        int32_t _unsigned = ((*msb << 16) | (*csb << 8) | *lsb);
        // convert 24 bit value to signed
        return (_unsigned & 0x800000) ? (_unsigned | 0xFF000000) : _unsigned;
    }

    /**
     * @brief Read 16-bit temperature register.
     *
     * Returns combined MSB/LSB temperature reading as raw register value.
     */
    tl::expected<uint16_t, QuickI2CStatus> readTemperature() {
        auto msb = readTemperatureMSB();
        if(!msb) {
            return tl::make_unexpected(msb.error());
        }
        auto lsb = readTemperatureLSB();
        if(!lsb) {
            return tl::make_unexpected(lsb.error());
        }

        return (*msb << 8) | *lsb;
    }
};
