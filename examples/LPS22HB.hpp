#pragma once
#include <QuickI2C.h>
#include <driver/i2c.h>
#include <esp_task_wdt.h>
#include <esp_log.h>
#include <math.h>

/**
 * Usage example:
 *
 *     LPS22HB<> baro(I2C_NUM_0);
 *     if (baro.Initialize() != QuickI2CStatus::OK) {
 *         printf("LPS22HB init failed\n");
 *     }
 *
 *     auto data = baro.readPressureAndTemperature();
 *     if (data) {
 *         printf("P: %.2f Pa, T: %.2f C\n", data->pressure, data->temperature);
 *     } else {
 *         printf("LPS22HB read error %d\n", (int)data.error());
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
class LPS22HB : public QuickI2CDevice {
public:
    inline LPS22HB(i2c_port_t port): QuickI2CDevice(0x5C, port, 400000) {} // LPS22HB I2C address

    struct PressureAndTemperature {
        float pressure; // Pressure in Pa
        float temperature; // Temperature in Celsius
    };

    // Status and identification registers
    QUICKI2C_DEFINE_REGISTER8_RO(InterruptCfg, 0x0B) {
        AUTORIFP = 0b1 << 7,    // Auto-increment register address for pressure high/low limits
        RESET_ARP = 0b1 << 6,   // Reset AutoRifP bit
        AUTOZERO = 0b1 << 5,    // Autozero function
        RESET_AZ = 0b1 << 4,    // Reset Autozero function
        DIFF_EN = 0b1 << 3,     // Enable interrupt generation on differential pressure
        LIR = 0b1 << 2,         // Latch interrupt request
        PL_E = 0b1 << 1,        // Enable interrupt generation on differential pressure low event
        PH_E = 0b1 << 0,        // Enable interrupt generation on differential pressure high event
    };

    QUICKI2C_DEFINE_REGISTER8_RO(ThrPressL, 0x0C) {}; // Pressure threshold value low part
    QUICKI2C_DEFINE_REGISTER8_RO(ThrPressH, 0x0D) {}; // Pressure threshold value high part

    QUICKI2C_DEFINE_REGISTER8_RO(WhoAmI, 0x0F) {}; // Who am I register (should return 0xB1)

    QUICKI2C_DEFINE_REGISTER8_RW(CtrlReg1, 0x10, 0x10) {
        ODR_MASK = 0b111 << 4,      // Output data rate mask
        ODR_ONE_SHOT = 0b000 << 4,  // One shot mode
        ODR_1HZ = 0b001 << 4,       // 1 Hz
        ODR_10HZ = 0b010 << 4,      // 10 Hz
        ODR_25HZ = 0b011 << 4,      // 25 Hz
        ODR_50HZ = 0b100 << 4,      // 50 Hz
        ODR_75HZ = 0b101 << 4,      // 75 Hz
        
        EN_LPFP = 0b1 << 3,         // Enable low-pass filter on pressure data
        LPFP_CFG = 0b1 << 2,        // Low-pass filter configuration
        BDU = 0b1 << 1,             // Block data update
        SIM = 0b1 << 0,             // SPI interface mode selection
    };

    QUICKI2C_DEFINE_REGISTER8_RW(CtrlReg2, 0x11, 0x11) {
        BOOT = 0b1 << 7,            // Reboot memory content
        FIFO_EN = 0b1 << 6,         // FIFO enable
        STOP_ON_FTH = 0b1 << 5,     // Stop on FIFO watermark
        IF_ADD_INC = 0b1 << 4,      // Register address automatically incremented during multiple byte access
        I2C_DIS = 0b1 << 3,         // Disable I2C interface
        SWRESET = 0b1 << 2,         // Software reset
        ONE_SHOT = 0b1 << 0,        // One shot enable
    };

    QUICKI2C_DEFINE_REGISTER8_RW(CtrlReg3, 0x12, 0x12) {
        INT_H_L = 0b1 << 7,         // Interrupt active high/low
        PP_OD = 0b1 << 6,           // Push-pull/open drain selection on interrupt pads
        F_FSS5 = 0b1 << 5,          // FIFO full flag on INT_DRDY pin
        F_FTH = 0b1 << 4,           // FIFO watermark status on INT_DRDY pin
        F_OVR = 0b1 << 3,           // FIFO overrun interrupt on INT_DRDY pin
        DRDY = 0b1 << 2,            // Data ready signal on INT_DRDY pin
        INT_S_MASK = 0b11 << 0,     // Data signal on INT_DRDY pin control bits
        INT_S_DATA_READY = 0b00 << 0,
        INT_S_PRESSURE_HIGH = 0b01 << 0,
        INT_S_PRESSURE_LOW = 0b10 << 0,
        INT_S_PRESSURE_LOW_OR_HIGH = 0b11 << 0,
    };

    QUICKI2C_DEFINE_REGISTER8_RO(FifoCtrl, 0x14) {
        F_MODE_MASK = 0b111 << 5,       // FIFO mode selection
        F_MODE_BYPASS = 0b000 << 5,     // Bypass mode
        F_MODE_FIFO = 0b001 << 5,       // FIFO mode
        F_MODE_STREAM = 0b010 << 5,     // Stream mode
        F_MODE_STREAM_TO_FIFO = 0b011 << 5, // Stream-to-FIFO mode
        F_MODE_BYPASS_TO_STREAM = 0b100 << 5, // Bypass-to-Stream mode
        F_MODE_DYNAMIC_STREAM = 0b110 << 5,   // Dynamic-Stream mode
        F_MODE_BYPASS_TO_FIFO = 0b111 << 5,   // Bypass-to-FIFO mode
        
        WTM_MASK = 0b11111 << 0,        // FIFO watermark level
    };

    QUICKI2C_DEFINE_REGISTER8_RO(RefPressXL, 0x15) {}; // Reference pressure low part
    QUICKI2C_DEFINE_REGISTER8_RO(RefPressL, 0x16) {};  // Reference pressure middle part
    QUICKI2C_DEFINE_REGISTER8_RO(RefPressH, 0x17) {};  // Reference pressure high part

    QUICKI2C_DEFINE_REGISTER8_RO(RpdsL, 0x18) {};      // Pressure offset low part
    QUICKI2C_DEFINE_REGISTER8_RO(RpdsH, 0x19) {};      // Pressure offset high part

    QUICKI2C_DEFINE_REGISTER8_RO(ResConf, 0x1A) {
        LC_EN = 0b1 << 0,           // Low current mode enable
    };

    QUICKI2C_DEFINE_REGISTER8_RO(IntSource, 0x25) {
        BOOT_STATUS = 0b1 << 7,     // Boot status
        IA = 0b1 << 2,              // Interrupt active
        PL = 0b1 << 1,              // Differential pressure low
        PH = 0b1 << 0,              // Differential pressure high
    };

    QUICKI2C_DEFINE_REGISTER8_RO(FifoStatus, 0x26) {
        FTH_FIFO = 0b1 << 7,        // FIFO watermark status
        OVR = 0b1 << 6,             // FIFO overrun status
        FSS_MASK = 0b11111 << 0,    // FIFO stored data level
    };

    QUICKI2C_DEFINE_REGISTER8_RO(Status, 0x27) {
        T_OR = 0b1 << 5,            // Temperature data overrun
        P_OR = 0b1 << 4,            // Pressure data overrun
        Temperature_DataAvailable = 0b1 << 1,            // Temperature data available
        Pressure_DataAvailable = 0b1 << 0,            // Pressure data available
    };

    // Pressure data registers
    QUICKI2C_DEFINE_REGISTER8_RO(PressOutXL, 0x28) {};  // Pressure output value low part
    QUICKI2C_DEFINE_REGISTER8_RO(PressOutL, 0x29) {};   // Pressure output value middle part
    QUICKI2C_DEFINE_REGISTER8_RO(PressOutH, 0x2A) {};   // Pressure output value high part

    // Temperature data registers
    QUICKI2C_DEFINE_REGISTER8_RO(TempOutL, 0x2B) {};    // Temperature output value low part
    QUICKI2C_DEFINE_REGISTER8_RO(TempOutH, 0x2C) {};    // Temperature output value high part

    QUICKI2C_DEFINE_REGISTER8_RO(LpfpRes, 0x33) {};     // Low-pass filter reset register

    /**
     * @brief Initialize LPS22HB and configure continuous pressure mode.
     *
     * - checks WhoAmI identity
     * - sets 25Hz ODR
     * - enables low-pass filter
     *
     * @return QuickI2CStatus
     */
    QuickI2CStatus Initialize() {
        // first, check device identity by reading WhoAmI register
        auto who = readWhoAmI();
        if (!who) {
            ESP_LOGE("LPS22HB", "Initialize(): failed to read WhoAmI (%s)", QuickI2CStatusToString(who.error()));
            return who.error();
        }
        if (*who != 0xB1) {
            ESP_LOGW("LPS22HB", "Initialize(): unexpected WhoAmI value 0x%02X (expected 0xB1)", *who);
        } else {
            ESP_LOGI("LPS22HB", "Initialize(): WhoAmI OK (0xB1)");
        }

        // Configure for continuous mode at 25Hz with LP filter enabled
        uint8_t cfg_val = (uint8_t)LPS22HB<>::CtrlReg1::ODR_25HZ |
                          (uint8_t)LPS22HB<>::CtrlReg1::EN_LPFP; // Bandwidth: ODR/9

        QuickI2CStatus rc = writeAndVerifyCtrlReg1(cfg_val);
        if (rc != QuickI2CStatus::OK) {
            // Log extra diagnostics when the verify step fails or other error occurs
            // read current register value for debugging purposes
            auto readBack = readCtrlReg1();
            if (readBack) {
                ESP_LOGE("LPS22HB", "Initialize(): wrote 0x%02X but read back 0x%02X (%s)",
                         cfg_val, static_cast<uint8_t>(*readBack), QuickI2CStatusToString(rc));
            } else {
                ESP_LOGE("LPS22HB", "Initialize(): wrote 0x%02X but failed to read back CtrlReg1 (%s); init error %s",
                         cfg_val, QuickI2CStatusToString(readBack.error()), QuickI2CStatusToString(rc));
            }
        }
        return rc;
    }

    /**
     * @brief Read pressure (Pa) and temperature (°C) from sensor.
     *
     * Waits up to 200 ms for data ready, then reads all required registers.
     * @return tl::expected<PressureAndTemperature, QuickI2CStatus>
     */
    tl::expected<PressureAndTemperature, QuickI2CStatus> readPressureAndTemperature() {
        // Wait until pressure data is available.
        // Important: never abort from inside a sensor driver.
        // Also apply a timeout so callers don't block indefinitely.
        const TickType_t startTicks = xTaskGetTickCount();
        while (true) {
            auto status = readStatus();
            if (!status) {
                return tl::make_unexpected(status.error());
            }
            if (*status & LPS22HB<>::Status::Pressure_DataAvailable) {
                break; // Data is available
            }

            // Reset WDT if this task is subscribed; ignore ESP_ERR_INVALID_STATE.
            (void)esp_task_wdt_reset();

            if ((xTaskGetTickCount() - startTicks) > pdMS_TO_TICKS(200)) {
                return tl::make_unexpected(QuickI2CStatus::DataTimeout);
            }

            // Small delay to prevent busy waiting
            vTaskDelay(pdMS_TO_TICKS(1));
        }

        auto xl = readPressOutXL();
        if(!xl) {
            return tl::make_unexpected(xl.error());
        }
        auto l = readPressOutL();
        if(!l) {
            return tl::make_unexpected(l.error());
        }
        auto h = readPressOutH();
        if(!h) {
            return tl::make_unexpected(h.error());
        }

        auto tempL = readTempOutL();
        if(!tempL) {
            return tl::make_unexpected(tempL.error());
        }
        auto tempH = readTempOutH();
        if(!tempH) {
            return tl::make_unexpected(tempH.error());
        }

        int32_t rawPressure = ((*h << 16) | (*l << 8) | *xl);
        // Convert 24-bit two's complement to 32-bit signed
        if (rawPressure & 0x800000) {
            rawPressure |= 0xFF000000;
        }
        
        float pressure = rawPressure / 4096.0f; // LSB = 1/4096 Pa
        int16_t rawTemp = static_cast<int16_t>((*tempH << 8) | *tempL);
        float temperature = rawTemp / 100.0f; // LSB = 0.01°C
        return PressureAndTemperature{pressure, temperature};
    }

    tl::expected<int16_t, QuickI2CStatus> readTemperature() {
        auto l = readTempOutL();
        if(!l) {
            return tl::make_unexpected(l.error());
        }
        auto h = readTempOutH();
        if(!h) {
            return tl::make_unexpected(h.error());
        }

        int16_t rawTemp = ((*h << 8) | *l);
        return rawTemp; // Returns temperature in °C * 100 (LSB = 0.01°C)
    }

};
