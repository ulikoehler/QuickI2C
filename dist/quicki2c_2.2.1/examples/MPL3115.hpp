#pragma once
#include <QuickI2C.h>
#include <math.h>
#include <cstdio>

/**
 * Usage example:
 *
 *     MPL3115 baro(QUICKI2C_DEFAULT_PORT);
 *     if (baro.Initialize() != QuickI2CStatus::OK) {
 *         printf("MPL3115 init failed\n");
 *     }
 *
 *     auto pres = baro.MeasurePressure(MPL3115::Ctrl1::OSR32);
 *     if (pres.has_value()) {
 *         printf("Pressure raw value: %u\n", pres.value());
 *     } else {
 *         printf("pressure read error: %d\n", (int)pres.error());
 *     }
 */

class MPL3115 : public QuickI2CDevice {
public:
    /**
     * @brief Construct a new MPL3115 object
     *
    * @param port QuickI2C bus port.
     */
    inline MPL3115(QuickI2CPort port = QUICKI2C_DEFAULT_PORT): QuickI2CDevice(0x60 /* Address */, port, 400000) {}
    
    QUICKI2C_DEFINE_REGISTER8_RO(Status, 0x06);
    QUICKI2C_DEFINE_REGISTER8_RW(SysMode, 0x11, 0x11);
    QUICKI2C_DEFINE_REGISTER8_RW(CtrlReg1, 0x26, 0x26);
    QUICKI2C_DEFINE_REGISTER8_RW(PtDataCfg, 0x13, 0x13);
    QUICKI2C_DEFINE_REGISTER24_RO(OutPressure, 0x01); // LSB, CSB, MSB

    /**
     * @brief Convert 24-bit big-endian pressure read into native format.
     *
     * This method is called by the base QuickI2CDevice read path after fetching
     * 24-bit sensor data. MPL3115 returns pressure as [MSB, CSB, LSB], with
     * an extra unused byte in the raw 32-bit value. We need to swap bytes and
     * discard the least significant 8 bits.
     *
     * @param address Register address that was read.
     * @param rawValue Raw value from I2C read operation.
     * @return tl::expected<uint32_t, QuickI2CStatus> Processed 24-bit pressure value or I2C status error.
     */
    virtual tl::expected<uint32_t, QuickI2CStatus> postprocessRead24(uint8_t address, tl::expected<uint32_t, QuickI2CStatus> rawValue) override {
        if(rawValue.has_value()) {
            return __builtin_bswap32(rawValue.value()) >> 8; // Shift out the unused LSB
        } else {
            return tl::unexpected{rawValue.error()};
        }
    }

    enum SystemMode {
        SystemStandby = 0b0,
        SystemActive = 0b1,
    };

    enum PtData : uint8_t {
        DataEventEnableOnNewTemperatureData = 0b1 << 0,
        DataEventEnableOnNewPressureData = 0b1 << 1,
        DataReadyEventMode = 0b0 << 2
    };

    enum Ctrl1 : uint8_t {
        Barometer = 0b0 << 7,
        Altimeter = 0b1 << 7,
        OSR1 = 0b000 << 3,
        OSR2 = 0b001 << 3,
        OSR4 = 0b010 << 3,
        OSR8 = 0b011 << 3,
        OSR16 = 0b100 << 3,
        OSR32 = 0b101 << 3,
        OSR64 = 0b110 << 3,
        OSR128 = 0b111 << 3,
        SoftwareReset = 0b1 << 2,
        StartMeasurement = 0b1 << 1, // OST, Initiate measurement immediately
        Standby = 0b0 << 1, // No periodic measurements
        Active = 0b1 << 0, // Periodic measurements
    };

    enum Status1 : uint8_t {
        PressureDataRead = 0b1 << 2,
        TemperatureDataReady = 0b1 << 1,
        // Others not listed here.
    };

    /**
     * @brief Initialize with sane default parameters:
     * - Active mode
     * - Data event flags enabled for new temperature and pressure data
     * - Data ready event mode (not latched)
     * Note: The sensor starts taking measurements immediately in active mode
     * 
     * @return QuickI2CStatus 
     */
    /**
     * @brief Initialize MPL3115 power and event flags.
     *
     * - Sets System Mode to Active
     * - Enables new temperature and pressure data event flags
     * - Uses Data Ready Event Mode
     *
     * @return QuickI2CStatus OK on success or hardware status code on failure
     */
    inline QuickI2CStatus Initialize() {
        auto result = writeSysMode(SystemMode::SystemActive);
        if(result != QuickI2CStatus::OK) {
            return result;
        }
        return writePtDataCfg(
            PtData::DataEventEnableOnNewTemperatureData |
            PtData::DataEventEnableOnNewPressureData |
            PtData::DataReadyEventMode
        );
    }

    /**
     * @brief Perform one pressure measurement and return raw pressure count.
     *
     * This method triggers a one-shot pressure measurement, waits until the
     * PressureDataRead status bit is set, then reads the 24-bit pressure value.
     *
     * @param osr Oversampling rate (one of Ctrl1::OSR1 .. Ctrl1::OSR128)
     * @return tl::expected<uint32_t, QuickI2CStatus> Pressure (raw 20-bit sample in 24-bit container) or error.
     */
    inline tl::expected<uint32_t, QuickI2CStatus> MeasurePressure(Ctrl1 osr) {
        // Start measurement
        auto result = writeCtrlReg1(Ctrl1::Barometer | osr | Ctrl1::Standby | Ctrl1::StartMeasurement);
        if(result != QuickI2CStatus::OK) {
            return tl::unexpected{result};
        }
        // Wait for measurement to complete
        while(true) {
            auto status = readStatus();
            if(status.has_value()) {
                if(status.value() & Status1::PressureDataRead) {
                    break;
                } else {
                    // You can uncomment this for debugging
                    // printf("Waiting for measurement to complete... %d\n", (int)status.value());
                }
            } else {
                // Handle error
                return tl::unexpected{status.error()};
            }
            // Wait until new data might be available
            vTaskDelay(5 / portTICK_PERIOD_MS);
        }
        // Read pressure data
        return readOutPressure();
    }
};