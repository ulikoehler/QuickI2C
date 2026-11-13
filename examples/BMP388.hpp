#pragma once
#include <QuickI2C.h>
#include <esp_task_wdt.h>
#include <esp_log.h>
#include <esp_rom_sys.h>
#include <math.h>

/**
 * Usage example:
 *
 *     BMP388<> baro(QUICKI2C_DEFAULT_PORT);
 *     if (baro.Initialize() != QuickI2CStatus::OK) {
 *         printf("BMP388 init failed\n");
 *     }
 *
 *     auto data = baro.readPressureAndTemperature();
 *     if (data) {
 *         printf("P: %.2f Pa, T: %.2f C\n", data->pressure, data->temperature);
 *     } else {
 *         printf("BMP388 read error %d\n", (int)data.error());
 *     }
 */

template<
    // Read filters
    Postprocessor8Bit postprocessRead8=noop,
    Postprocessor16Bit postprocessRead16=noop,
    Postprocessor24Bit postprocessRead24=noop,
    Postprocessor32Bit postprocessRead32=noop,
    // Write filters
    Postprocessor8Bit postprocessWrite8=noop,
    Postprocessor16Bit postprocessWrite16=noop,
    Postprocessor24Bit postprocessWrite24=noop,
    Postprocessor32Bit postprocessWrite32=noop
>
class BMP388 : public QuickI2CDevice {
public:
    // BMP388 I2C address is 0x76 (SDO low) or 0x77 (SDO high)
    inline BMP388(QuickI2CPort port = QUICKI2C_DEFAULT_PORT, uint8_t address = 0x76)
        : QuickI2CDevice(address, port, 400000) {}

    struct PressureAndTemperature {
        float pressure; // Pressure in Pa
        float temperature; // Temperature in Celsius
    };

    // Chip identification
    QUICKI2C_DEFINE_REGISTER8_RO(ChipId, 0x00) {}; // 0x50 = BMP388, 0x60 = BMP390

    QUICKI2C_DEFINE_REGISTER8_RO(ErrReg, 0x02) {
        FATAL_ERR = 0b1 << 0,
        CMD_ERR = 0b1 << 1,
        CONF_ERR = 0b1 << 2,
    };

    QUICKI2C_DEFINE_REGISTER8_RO(Status, 0x03) {
        CMD_RDY = 0b1 << 4,
        DRDY_PRESS = 0b1 << 5,
        DRDY_TEMP = 0b1 << 6,
    };

    QUICKI2C_DEFINE_REGISTER8_RW(PwrCtrl, 0x1B, 0x1B) {
        PRESS_EN = 0b1 << 0,
        TEMP_EN = 0b1 << 1,
        OP_MODE_MASK = 0b11 << 4,
        OP_MODE_SLEEP = 0b00 << 4,
        OP_MODE_FORCED = 0b01 << 4,
        OP_MODE_NORMAL = 0b11 << 4,
    };

    QUICKI2C_DEFINE_REGISTER8_RW(Osr, 0x1C, 0x1C) {
        PRESS_OS_1X = 0b000,
        PRESS_OS_2X = 0b001,
        PRESS_OS_4X = 0b010,
        PRESS_OS_8X = 0b011,
        PRESS_OS_16X = 0b100,
        PRESS_OS_32X = 0b101,
        TEMP_OS_1X = 0b000 << 3,
        TEMP_OS_2X = 0b001 << 3,
        TEMP_OS_4X = 0b010 << 3,
        TEMP_OS_8X = 0b011 << 3,
        TEMP_OS_16X = 0b100 << 3,
        TEMP_OS_32X = 0b101 << 3,
    };

    QUICKI2C_DEFINE_REGISTER8_RW(Odr, 0x1D, 0x1D) {
        ODR_200HZ = 0x00,
        ODR_100HZ = 0x01,
        ODR_50HZ = 0x02,
        ODR_25HZ = 0x03,
        ODR_12_5HZ = 0x04,
        ODR_6_25HZ = 0x05,
    };

    QUICKI2C_DEFINE_REGISTER8_RW(Config, 0x1F, 0x1F) {
        IIR_DISABLE = 0b000 << 1,
        IIR_COEFF_1 = 0b001 << 1,
        IIR_COEFF_3 = 0b010 << 1,
        IIR_COEFF_7 = 0b011 << 1,
        IIR_COEFF_15 = 0b100 << 1,
        IIR_COEFF_31 = 0b101 << 1,
        IIR_COEFF_63 = 0b110 << 1,
        IIR_COEFF_127 = 0b111 << 1,
    };

    QUICKI2C_DEFINE_REGISTER8_RW(Cmd, 0x7E, 0x7E) {
        FIFO_FLUSH = 0xB0,
        SOFT_RESET = 0xB6,
    };

    // Sensor data: 6 bytes at 0x04, pressure LE24 then temperature LE24
    static constexpr uint8_t DataAddress = 0x04;
    // Factory calibration data: 21 bytes at 0x31
    static constexpr uint8_t CalibDataAddress = 0x31;
    static constexpr size_t CalibDataLength = 21;

    struct CalibrationData {
        uint16_t t1 = 0, t2 = 0;
        int8_t t3 = 0;
        int16_t p1 = 0, p2 = 0;
        int8_t p3 = 0, p4 = 0;
        uint16_t p5 = 0, p6 = 0;
        int8_t p7 = 0, p8 = 0;
        int16_t p9 = 0;
        int8_t p10 = 0, p11 = 0;
    };

    /**
     * @brief Initialize BMP388/BMP390 and configure continuous 25Hz mode.
     *
     * - soft reset
     * - checks ChipId (0x50 BMP388 / 0x60 BMP390)
     * - reads factory calibration from NVM-mapped registers
     * - sets 25Hz ODR, 2x oversampling, mild IIR
     * - enables pressure + temperature in normal mode
     *
     * @return QuickI2CStatus
     */
    QuickI2CStatus Initialize() {
        // Soft reset the sensor
        auto rc = writeCmd(Cmd::SOFT_RESET);
        if (rc != QuickI2CStatus::OK) {
            ESP_LOGE("BMP388", "Initialize(): failed to write soft reset command (%s)",
                     QuickI2CStatusToString(rc));
            return rc;
        }
        esp_rom_delay_us(2000);

        auto chip = readChipId();
        if (!chip) {
            ESP_LOGE("BMP388", "Initialize(): failed to read ChipId (%s)",
                     QuickI2CStatusToString(chip.error()));
            return chip.error();
        }
        if (*chip == 0x50) {
            ESP_LOGI("BMP388", "Initialize(): ChipId OK (0x50, BMP388)");
        } else if (*chip == 0x60) {
            ESP_LOGI("BMP388", "Initialize(): ChipId OK (0x60, BMP390)");
        } else {
            ESP_LOGE("BMP388", "Initialize(): unexpected ChipId 0x%02X (expected 0x50)", *chip);
            return QuickI2CStatus::UnknownError;
        }

        uint8_t calibRaw[CalibDataLength] = {0};
        rc = readData(CalibDataAddress, calibRaw, CalibDataLength);
        if (rc != QuickI2CStatus::OK) {
            ESP_LOGE("BMP388", "Initialize(): failed to read calibration data (%s)",
                     QuickI2CStatusToString(rc));
            return rc;
        }
        parseCalibrationData(calibRaw);

        // Configure 2x oversampling for pressure and temperature
        rc = writeOsr(Osr::PRESS_OS_2X | Osr::TEMP_OS_2X);
        if (rc != QuickI2CStatus::OK) {
            ESP_LOGE("BMP388", "Initialize(): failed to write OSR (%s)", QuickI2CStatusToString(rc));
            return rc;
        }
        // 25Hz ODR - same rate the LPS22HB was configured with
        rc = writeOdr(Odr::ODR_25HZ);
        if (rc != QuickI2CStatus::OK) {
            ESP_LOGE("BMP388", "Initialize(): failed to write ODR (%s)", QuickI2CStatusToString(rc));
            return rc;
        }
        // Mild IIR low-pass, roughly equivalent to LPS22HB EN_LPFP
        rc = writeConfig(Config::IIR_COEFF_1);
        if (rc != QuickI2CStatus::OK) {
            ESP_LOGE("BMP388", "Initialize(): failed to write CONFIG (%s)", QuickI2CStatusToString(rc));
            return rc;
        }
        // Enable pressure + temperature, normal (continuous) mode
        rc = writePwrCtrl(PwrCtrl::PRESS_EN | PwrCtrl::TEMP_EN | PwrCtrl::OP_MODE_NORMAL);
        if (rc != QuickI2CStatus::OK) {
            ESP_LOGE("BMP388", "Initialize(): failed to write PWR_CTRL (%s)", QuickI2CStatusToString(rc));
            return rc;
        }

        auto err = readErrReg();
        if (!err) {
            ESP_LOGW("BMP388", "Initialize(): failed to read ERR_REG (%s)",
                     QuickI2CStatusToString(err.error()));
        } else if (*err & ErrReg::CONF_ERR) {
            ESP_LOGE("BMP388", "Initialize(): OSR/ODR configuration rejected by sensor (ERR_REG=0x%02X)", *err);
            return QuickI2CStatus::UnknownError;
        }
        return QuickI2CStatus::OK;
    }

    /**
     * @brief Read pressure (Pa) and temperature (°C) from sensor.
     *
     * Waits up to 200 ms for pressure data ready, then reads all 6 data
     * registers in one burst and applies Bosch fixed-point compensation.
     * @return tl::expected<PressureAndTemperature, QuickI2CStatus>
     */
    tl::expected<PressureAndTemperature, QuickI2CStatus> readPressureAndTemperature() {
        const TickType_t startTicks = xTaskGetTickCount();
        while (true) {
            auto status = readStatus();
            if (!status) {
                return tl::make_unexpected(status.error());
            }
            if (*status & Status::DRDY_PRESS) {
                break;
            }
            (void)esp_task_wdt_reset();
            if ((xTaskGetTickCount() - startTicks) > pdMS_TO_TICKS(200)) {
                return tl::make_unexpected(QuickI2CStatus::DataTimeout);
            }
            vTaskDelay(pdMS_TO_TICKS(1));
        }

        uint8_t raw[6] = {0};
        auto rc = readData(DataAddress, raw, sizeof(raw));
        if (rc != QuickI2CStatus::OK) {
            return tl::make_unexpected(rc);
        }

        uint32_t rawPressure = static_cast<uint32_t>(raw[0]) |
                               (static_cast<uint32_t>(raw[1]) << 8) |
                               (static_cast<uint32_t>(raw[2]) << 16);
        uint32_t rawTemp = static_cast<uint32_t>(raw[3]) |
                           (static_cast<uint32_t>(raw[4]) << 8) |
                           (static_cast<uint32_t>(raw[5]) << 16);

        int64_t tLin = compensateTemperature(rawTemp);
        // Compensated temperature in centi-°C, clamped to sensor range (-40..85 °C)
        int64_t compTemp = (tLin * 25) / 16384;
        compTemp = compTemp < -4000 ? -4000 : (compTemp > 8500 ? 8500 : compTemp);
        float temperature = static_cast<float>(compTemp) / 100.0f;
        float pressure = compensatePressure(rawPressure, tLin);
        return PressureAndTemperature{pressure, temperature};
    }

private:
    CalibrationData calib_;

    static uint16_t readLe16(const uint8_t* p) {
        return static_cast<uint16_t>(p[0]) | (static_cast<uint16_t>(p[1]) << 8);
    }
    static int16_t readLe16s(const uint8_t* p) {
        return static_cast<int16_t>(readLe16(p));
    }

    void parseCalibrationData(const uint8_t* raw) {
        calib_.t1 = readLe16(&raw[0]);
        calib_.t2 = readLe16(&raw[2]);
        calib_.t3 = static_cast<int8_t>(raw[4]);
        calib_.p1 = readLe16s(&raw[5]);
        calib_.p2 = readLe16s(&raw[7]);
        calib_.p3 = static_cast<int8_t>(raw[9]);
        calib_.p4 = static_cast<int8_t>(raw[10]);
        calib_.p5 = readLe16(&raw[11]);
        calib_.p6 = readLe16(&raw[13]);
        calib_.p7 = static_cast<int8_t>(raw[15]);
        calib_.p8 = static_cast<int8_t>(raw[16]);
        calib_.p9 = readLe16s(&raw[17]);
        calib_.p10 = static_cast<int8_t>(raw[19]);
        calib_.p11 = static_cast<int8_t>(raw[20]);
    }

    // Bosch BMP3 fixed-point compensation (bst-bmp3 ds000, bmp3.c)
    int64_t compensateTemperature(uint32_t rawTemp) {
        int64_t partial1 = static_cast<int64_t>(rawTemp) - (256LL * calib_.t1);
        int64_t partial2 = static_cast<int64_t>(calib_.t2) * partial1;
        int64_t partial3 = partial1 * partial1;
        int64_t partial4 = partial3 * calib_.t3;
        int64_t partial5 = (partial2 * 262144LL) + partial4;
        return partial5 / 4294967296LL; // t_lin
    }

    float compensatePressure(uint32_t rawPressure, int64_t tLin) {
        int64_t partial1 = tLin * tLin;
        int64_t partial2 = partial1 / 64;
        int64_t partial3 = (partial2 * tLin) / 256;
        int64_t partial4 = (calib_.p8 * partial3) / 32;
        int64_t partial5 = (calib_.p7 * partial1) * 16;
        int64_t partial6 = (calib_.p6 * tLin) * 4194304LL;
        int64_t offset = (calib_.p5 * 140737488355328LL) + partial4 + partial5 + partial6;
        partial2 = (calib_.p4 * partial3) / 32;
        partial4 = (calib_.p3 * partial1) * 4;
        partial5 = (static_cast<int64_t>(calib_.p2) - 16384) * tLin * 2097152LL;
        int64_t sensitivity = ((static_cast<int64_t>(calib_.p1) - 16384) * 70368744177664LL) +
                              partial2 + partial4 + partial5;
        partial1 = (sensitivity / 16777216LL) * static_cast<int64_t>(rawPressure);
        partial2 = static_cast<int64_t>(calib_.p10) * tLin;
        partial3 = partial2 + (65536LL * calib_.p9);
        partial4 = (partial3 * static_cast<int64_t>(rawPressure)) / 8192;
        // Divide by 10 then multiply by 10 to avoid overflow of rawPressure * partial4
        partial5 = (static_cast<int64_t>(rawPressure) * (partial4 / 10)) / 512;
        partial5 *= 10;
        partial6 = static_cast<int64_t>(rawPressure) * rawPressure;
        partial2 = (static_cast<int64_t>(calib_.p11) * partial6) / 65536;
        partial3 = (partial2 * static_cast<int64_t>(rawPressure)) / 128;
        partial4 = (offset / 4) + partial1 + partial5 + partial3;
        uint64_t compPress = (static_cast<uint64_t>(partial4) * 25) / 1099511627776ULL;
        // Clamp to sensor range (30000..125000 Pa), like the Bosch reference driver
        compPress = compPress < 3000000ULL ? 3000000ULL : (compPress > 12500000ULL ? 12500000ULL : compPress);
        return static_cast<float>(compPress) / 100.0f; // hundredths of Pa -> Pa
    }
};
