#pragma once

#include <QuickI2C.h>

#include <math.h>

class TMP10x : public QuickI2CDevice {
public:
    enum class Address : uint8_t {
        GND_GND = 0x48,
        GND_FLOAT = 0x49,
        GND_VCC = 0x4A,
        FLOAT_GND = 0x4B,
        VCC_GND = 0x4C,
        VCC_FLOAT = 0x4D,
        VCC_VCC = 0x4E,
        FLOAT_VCC = 0x4F,
    };

    enum class Register : uint8_t {
        Temperature = 0x00,
        Configuration = 0x01,
        TLow = 0x02,
        THigh = 0x03,
    };

    enum class ThermostatMode : uint8_t {
        Comparator = 0x00,
        Interrupt = 0x02,
    };

    enum class AlertPolarity : uint8_t {
        ActiveLow = 0x00,
        ActiveHigh = 0x04,
    };

    enum class FaultQueue : uint8_t {
        Consecutive1 = 0x00,
        Consecutive2 = 0x08,
        Consecutive4 = 0x10,
        Consecutive6 = 0x18,
    };

    enum class Resolution : uint8_t {
        Bits9 = 0x00,
        Bits10 = 0x20,
        Bits11 = 0x40,
        Bits12 = 0x60,
    };

    static constexpr uint8_t ConfigurationShutdownMask = 0x01;
    static constexpr uint8_t ConfigurationThermostatModeMask = 0x02;
    static constexpr uint8_t ConfigurationPolarityMask = 0x04;
    static constexpr uint8_t ConfigurationFaultQueueMask = 0x18;
    static constexpr uint8_t ConfigurationResolutionMask = 0x60;
    static constexpr uint8_t ConfigurationOneShotMask = 0x80;

    static constexpr uint8_t GeneralCallAddress = 0x00;
    static constexpr uint8_t GeneralCallAddressLatchCommand = 0x04;
    static constexpr uint8_t GeneralCallResetCommand = 0x06;
    static constexpr uint8_t SmbusAlertResponseAddress = 0x0C;

    #ifdef QUICKI2C_DRIVER_ARDUINO
    inline TMP10x(
        Address address = Address::GND_GND,
        TwoWire& wire = Wire,
        uint32_t i2cClockSpeed = 400000,
        uint32_t timeout = 100
    ) : QuickI2CDevice(static_cast<uint8_t>(address), wire, i2cClockSpeed, timeout) {}
    #elif defined(QUICKI2C_DRIVER_ESPIDF)
    inline TMP10x(
        Address address = Address::GND_GND,
        i2c_port_t port = I2C_NUM_0,
        uint32_t i2cClockSpeed = 400000,
        uint32_t timeout = 100
    ) : QuickI2CDevice(static_cast<uint8_t>(address), port, i2cClockSpeed, timeout) {}
    #endif

    inline tl::expected<uint8_t, QuickI2CStatus> readConfiguration() {
        return read8BitRegister(static_cast<uint8_t>(Register::Configuration));
    }

    inline QuickI2CStatus writeConfiguration(uint8_t configuration) {
        return write8BitRegister(static_cast<uint8_t>(Register::Configuration), configuration);
    }

    inline QuickI2CStatus writeAndVerifyConfiguration(uint8_t configuration) {
        QuickI2CStatus status = writeConfiguration(configuration);
        if(status != QuickI2CStatus::OK) {
            return status;
        }

        auto readback = readConfiguration();
        if(!readback) {
            return readback.error();
        }

        if(((*readback) & static_cast<uint8_t>(~ConfigurationOneShotMask)) !=
           (configuration & static_cast<uint8_t>(~ConfigurationOneShotMask))) {
            return QuickI2CStatus::VerifyMismatch;
        }

        return QuickI2CStatus::OK;
    }

    inline QuickI2CStatus setShutdownEnabled(bool enabled) {
        return updateConfiguration(ConfigurationShutdownMask, enabled ? ConfigurationShutdownMask : 0x00);
    }

    inline tl::expected<bool, QuickI2CStatus> isShutdownEnabled() {
        return readConfigurationBit(ConfigurationShutdownMask);
    }

    inline QuickI2CStatus setThermostatMode(ThermostatMode mode) {
        return updateConfiguration(ConfigurationThermostatModeMask, static_cast<uint8_t>(mode));
    }

    inline tl::expected<ThermostatMode, QuickI2CStatus> getThermostatMode() {
        auto configuration = readConfiguration();
        if(!configuration) {
            return tl::unexpected<QuickI2CStatus>(configuration.error());
        }
        return static_cast<ThermostatMode>(*configuration & ConfigurationThermostatModeMask);
    }

    inline QuickI2CStatus setAlertPolarity(AlertPolarity polarity) {
        return updateConfiguration(ConfigurationPolarityMask, static_cast<uint8_t>(polarity));
    }

    inline tl::expected<AlertPolarity, QuickI2CStatus> getAlertPolarity() {
        auto configuration = readConfiguration();
        if(!configuration) {
            return tl::unexpected<QuickI2CStatus>(configuration.error());
        }
        return static_cast<AlertPolarity>(*configuration & ConfigurationPolarityMask);
    }

    inline QuickI2CStatus setFaultQueue(FaultQueue queue) {
        return updateConfiguration(ConfigurationFaultQueueMask, static_cast<uint8_t>(queue));
    }

    inline tl::expected<FaultQueue, QuickI2CStatus> getFaultQueue() {
        auto configuration = readConfiguration();
        if(!configuration) {
            return tl::unexpected<QuickI2CStatus>(configuration.error());
        }
        return static_cast<FaultQueue>(*configuration & ConfigurationFaultQueueMask);
    }

    inline QuickI2CStatus setResolution(Resolution resolution) {
        return updateConfiguration(ConfigurationResolutionMask, static_cast<uint8_t>(resolution));
    }

    inline tl::expected<Resolution, QuickI2CStatus> getResolution() {
        auto configuration = readConfiguration();
        if(!configuration) {
            return tl::unexpected<QuickI2CStatus>(configuration.error());
        }
        return static_cast<Resolution>(*configuration & ConfigurationResolutionMask);
    }

    inline static uint16_t getTypicalConversionTimeMilliseconds(Resolution resolution) {
        switch(resolution) {
            case Resolution::Bits9:
                return 40;
            case Resolution::Bits10:
                return 80;
            case Resolution::Bits11:
                return 160;
            case Resolution::Bits12:
            default:
                return 320;
        }
    }

    inline QuickI2CStatus triggerOneShot() {
        auto configuration = readConfiguration();
        if(!configuration) {
            return configuration.error();
        }
        return writeConfiguration(
            ((*configuration) & static_cast<uint8_t>(~ConfigurationOneShotMask)) |
            ConfigurationOneShotMask
        );
    }

    inline tl::expected<bool, QuickI2CStatus> readAlertStatus() {
        auto configuration = readConfiguration();
        if(!configuration) {
            return tl::unexpected<QuickI2CStatus>(configuration.error());
        }

        bool osAlertBit = ((*configuration) & ConfigurationOneShotMask) != 0;
        bool activeHigh = ((*configuration) & ConfigurationPolarityMask) != 0;
        return activeHigh ? osAlertBit : !osAlertBit;
    }

    inline QuickI2CStatus clearAlert() {
        auto configuration = readConfiguration();
        if(!configuration) {
            return configuration.error();
        }
        return QuickI2CStatus::OK;
    }

    inline tl::expected<uint16_t, QuickI2CStatus> readTemperatureRegisterRaw() {
        return readRegister16(static_cast<uint8_t>(Register::Temperature));
    }

    inline tl::expected<int16_t, QuickI2CStatus> readTemperatureRaw() {
        auto raw = readTemperatureRegisterRaw();
        if(!raw) {
            return tl::unexpected<QuickI2CStatus>(raw.error());
        }
        return decodeTemperatureRegister(*raw);
    }

    inline tl::expected<float, QuickI2CStatus> readTemperatureCelsius() {
        auto raw = readTemperatureRaw();
        if(!raw) {
            return tl::unexpected<QuickI2CStatus>(raw.error());
        }
        return rawTemperatureToCelsius(*raw);
    }

    inline tl::expected<uint16_t, QuickI2CStatus> readTLowRegisterRaw() {
        return readRegister16(static_cast<uint8_t>(Register::TLow));
    }

    inline tl::expected<int16_t, QuickI2CStatus> readTLowRaw() {
        auto raw = readTLowRegisterRaw();
        if(!raw) {
            return tl::unexpected<QuickI2CStatus>(raw.error());
        }
        return decodeTemperatureRegister(*raw);
    }

    inline tl::expected<float, QuickI2CStatus> readTLowCelsius() {
        auto raw = readTLowRaw();
        if(!raw) {
            return tl::unexpected<QuickI2CStatus>(raw.error());
        }
        return rawTemperatureToCelsius(*raw);
    }

    inline QuickI2CStatus writeTLowRegisterRaw(uint16_t value) {
        return writeRegister16(static_cast<uint8_t>(Register::TLow), value);
    }

    inline QuickI2CStatus writeAndVerifyTLowRegisterRaw(uint16_t value) {
        return writeAndVerifyRegister16(static_cast<uint8_t>(Register::TLow), value);
    }

    inline QuickI2CStatus writeTLowRaw(int16_t value) {
        return writeTLowRegisterRaw(encodeTemperatureRegister(value));
    }

    inline QuickI2CStatus writeAndVerifyTLowRaw(int16_t value) {
        return writeAndVerifyTLowRegisterRaw(encodeTemperatureRegister(value));
    }

    inline QuickI2CStatus writeTLowCelsius(float value) {
        return writeTLowRaw(celsiusToRawTemperature(value));
    }

    inline QuickI2CStatus writeAndVerifyTLowCelsius(float value) {
        return writeAndVerifyTLowRaw(celsiusToRawTemperature(value));
    }

    inline tl::expected<uint16_t, QuickI2CStatus> readTHighRegisterRaw() {
        return readRegister16(static_cast<uint8_t>(Register::THigh));
    }

    inline tl::expected<int16_t, QuickI2CStatus> readTHighRaw() {
        auto raw = readTHighRegisterRaw();
        if(!raw) {
            return tl::unexpected<QuickI2CStatus>(raw.error());
        }
        return decodeTemperatureRegister(*raw);
    }

    inline tl::expected<float, QuickI2CStatus> readTHighCelsius() {
        auto raw = readTHighRaw();
        if(!raw) {
            return tl::unexpected<QuickI2CStatus>(raw.error());
        }
        return rawTemperatureToCelsius(*raw);
    }

    inline QuickI2CStatus writeTHighRegisterRaw(uint16_t value) {
        return writeRegister16(static_cast<uint8_t>(Register::THigh), value);
    }

    inline QuickI2CStatus writeAndVerifyTHighRegisterRaw(uint16_t value) {
        return writeAndVerifyRegister16(static_cast<uint8_t>(Register::THigh), value);
    }

    inline QuickI2CStatus writeTHighRaw(int16_t value) {
        return writeTHighRegisterRaw(encodeTemperatureRegister(value));
    }

    inline QuickI2CStatus writeAndVerifyTHighRaw(int16_t value) {
        return writeAndVerifyTHighRegisterRaw(encodeTemperatureRegister(value));
    }

    inline QuickI2CStatus writeTHighCelsius(float value) {
        return writeTHighRaw(celsiusToRawTemperature(value));
    }

    inline QuickI2CStatus writeAndVerifyTHighCelsius(float value) {
        return writeAndVerifyTHighRaw(celsiusToRawTemperature(value));
    }

    inline static float rawTemperatureToCelsius(int16_t value) {
        return static_cast<float>(value) / 16.0f;
    }

    inline static int16_t celsiusToRawTemperature(float value) {
        return static_cast<int16_t>(roundf(value * 16.0f));
    }

    inline static int16_t decodeTemperatureRegister(uint16_t value) {
        int16_t raw = static_cast<int16_t>(value >> 4);
        if((raw & 0x0800) != 0) {
            raw |= static_cast<int16_t>(0xF000);
        }
        return raw;
    }

    inline static uint16_t encodeTemperatureRegister(int16_t value) {
        return static_cast<uint16_t>(value) << 4;
    }

    #ifdef QUICKI2C_DRIVER_ARDUINO
    inline static QuickI2CStatus latchAddressPins(
        TwoWire& wire = Wire,
        uint32_t i2cClockSpeed = 400000,
        uint32_t timeout = 100
    ) {
        QuickI2CDevice generalCall(GeneralCallAddress, wire, i2cClockSpeed, timeout);
        return generalCall.writeData(GeneralCallAddressLatchCommand, nullptr, 0);
    }

    inline static QuickI2CStatus generalCallReset(
        TwoWire& wire = Wire,
        uint32_t i2cClockSpeed = 400000,
        uint32_t timeout = 100
    ) {
        QuickI2CDevice generalCall(GeneralCallAddress, wire, i2cClockSpeed, timeout);
        return generalCall.writeData(GeneralCallResetCommand, nullptr, 0);
    }

    inline static tl::expected<uint8_t, QuickI2CStatus> readSmbusAlertResponse(
        TwoWire& wire = Wire,
        uint32_t i2cClockSpeed = 400000,
        uint32_t timeout = 100
    ) {
        wire.setClock(i2cClockSpeed);
        wire.setTimeout(timeout);
        size_t bytesRead = wire.requestFrom(static_cast<uint8_t>(SmbusAlertResponseAddress), static_cast<uint8_t>(1));
        if(bytesRead < 1) {
            return tl::unexpected<QuickI2CStatus>(QuickI2CStatus::SlaveNotResponding);
        }
        return static_cast<uint8_t>(wire.read());
    }
    #elif defined(QUICKI2C_DRIVER_ESPIDF)
    inline static QuickI2CStatus latchAddressPins(
        i2c_port_t port = I2C_NUM_0,
        uint32_t i2cClockSpeed = 400000,
        uint32_t timeout = 100
    ) {
        QuickI2CDevice generalCall(GeneralCallAddress, port, i2cClockSpeed, timeout);
        return generalCall.writeData(GeneralCallAddressLatchCommand, nullptr, 0);
    }

    inline static QuickI2CStatus generalCallReset(
        i2c_port_t port = I2C_NUM_0,
        uint32_t i2cClockSpeed = 400000,
        uint32_t timeout = 100
    ) {
        QuickI2CDevice generalCall(GeneralCallAddress, port, i2cClockSpeed, timeout);
        return generalCall.writeData(GeneralCallResetCommand, nullptr, 0);
    }

    inline static tl::expected<uint8_t, QuickI2CStatus> readSmbusAlertResponse(
        i2c_port_t port = I2C_NUM_0,
        uint32_t timeout = 100
    ) {
        uint8_t response = 0;
        esp_err_t err = i2c_master_read_from_device(
            port,
            SmbusAlertResponseAddress,
            &response,
            1,
            timeout / portTICK_PERIOD_MS
        );
        switch(err) {
            case ESP_OK:
                return response;
            case ESP_ERR_TIMEOUT:
                return tl::unexpected<QuickI2CStatus>(QuickI2CStatus::BusBusy);
            case ESP_FAIL:
                return tl::unexpected<QuickI2CStatus>(QuickI2CStatus::SlaveNotResponding);
            case ESP_ERR_INVALID_STATE:
                return tl::unexpected<QuickI2CStatus>(QuickI2CStatus::DriverNotInitialized);
            default:
                return tl::unexpected<QuickI2CStatus>(QuickI2CStatus::UnknownError);
        }
    }
    #endif

private:
    inline QuickI2CStatus updateConfiguration(uint8_t mask, uint8_t value) {
        auto configuration = readConfiguration();
        if(!configuration) {
            return configuration.error();
        }
        uint8_t updated =
            (((*configuration) & static_cast<uint8_t>(~ConfigurationOneShotMask)) & static_cast<uint8_t>(~mask)) |
            (value & mask);
        return writeConfiguration(updated);
    }

    inline tl::expected<bool, QuickI2CStatus> readConfigurationBit(uint8_t mask) {
        auto configuration = readConfiguration();
        if(!configuration) {
            return tl::unexpected<QuickI2CStatus>(configuration.error());
        }
        return (*configuration & mask) != 0;
    }

    inline tl::expected<uint16_t, QuickI2CStatus> readRegister16(uint8_t registerAddress) {
        uint8_t buffer[2] = {0, 0};
        QuickI2CStatus status = readData(registerAddress, buffer, sizeof(buffer));
        if(status != QuickI2CStatus::OK) {
            return tl::unexpected<QuickI2CStatus>(status);
        }
        return (static_cast<uint16_t>(buffer[0]) << 8) | static_cast<uint16_t>(buffer[1]);
    }

    inline QuickI2CStatus writeRegister16(uint8_t registerAddress, uint16_t value) {
        uint8_t buffer[2] = {
            static_cast<uint8_t>(value >> 8),
            static_cast<uint8_t>(value & 0xFF),
        };
        return writeData(registerAddress, buffer, sizeof(buffer));
    }

    inline QuickI2CStatus writeAndVerifyRegister16(uint8_t registerAddress, uint16_t value) {
        uint8_t buffer[2] = {
            static_cast<uint8_t>(value >> 8),
            static_cast<uint8_t>(value & 0xFF),
        };
        return writeAndVerifyData(registerAddress, registerAddress, buffer, sizeof(buffer));
    }
};