#pragma once

#include <QuickI2C.h>

#include <math.h>

/**
 * Usage example:
 *
 *     TMP10x sensor(TMP10x::Address::GND_GND, I2C_NUM_0);
 *     auto temperature = sensor.readTemperatureCelsius();
 *     if (temperature) {
 *         printf("Temperature: %.2f C\n", *temperature);
 *     } else {
 *         printf("Error reading temperature: %d\n", (int)temperature.error());
 *     }
 */
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
    /**
     * @brief Construct a TMP10x object for Arduino Wire driver.
     *
     * @param address TMP10x I2C address configuration.
     * @param wire TwoWire instance.
     * @param i2cClockSpeed I2C bus speed.
     * @param timeout I2C transaction timeout in ms.
     */
    inline TMP10x(
        Address address = Address::GND_GND,
        TwoWire& wire = Wire,
        uint32_t i2cClockSpeed = 400000,
        uint32_t timeout = 100
    ) : QuickI2CDevice(static_cast<uint8_t>(address), wire, i2cClockSpeed, timeout) {}
    #elif defined(QUICKI2C_DRIVER_ESPIDF)
    /**
     * @brief Construct a TMP10x object for ESP-IDF I2C driver.
     *
     * @param address TMP10x I2C address configuration.
     * @param port i2c_port_t (I2C_NUM_0 or I2C_NUM_1).
     * @param i2cClockSpeed I2C bus speed.
     * @param timeout I2C transaction timeout in ms.
     */
    inline TMP10x(
        Address address = Address::GND_GND,
        i2c_port_t port = I2C_NUM_0,
        uint32_t i2cClockSpeed = 400000,
        uint32_t timeout = 100
    ) : QuickI2CDevice(static_cast<uint8_t>(address), port, i2cClockSpeed, timeout) {}
    #endif

    // Register-level accessors provided by QuickI2C macros
    QUICKI2C_DEFINE_REGISTER16_RO(TemperatureRegister, 0x00);
    QUICKI2C_DEFINE_REGISTER8_RW(Configuration, 0x01, 0x01);
    QUICKI2C_DEFINE_REGISTER16_RW(TLowRegister, 0x02, 0x02);
    QUICKI2C_DEFINE_REGISTER16_RW(THighRegister, 0x03, 0x03);

    // Configuration register direct access is provided via QUICKI2C_DEFINE_REGISTER8_RW(Configuration, 0x01, 0x01)

    /**
     * @brief Enable or disable shutdown mode.
     * @param enabled true to shutdown, false to wake.
     * @return QuickI2CStatus
     */
    inline QuickI2CStatus setShutdownEnabled(bool enabled) {
        return updateConfiguration(ConfigurationShutdownMask, enabled ? ConfigurationShutdownMask : 0x00);
    }

    /**
     * @brief Query shutdown bit state.
     * @return tl::expected<bool, QuickI2CStatus>
     */
    inline tl::expected<bool, QuickI2CStatus> isShutdownEnabled() {
        return readConfigurationBit(ConfigurationShutdownMask);
    }

    /**
     * @brief Set thermostat comparator interrupt mode.
     * @param mode Comparator or Interrupt mode.
     * @return QuickI2CStatus
     */
    inline QuickI2CStatus setThermostatMode(ThermostatMode mode) {
        return updateConfiguration(ConfigurationThermostatModeMask, static_cast<uint8_t>(mode));
    }

    /**
     * @brief Get thermostat comparator interrupt mode.
     * @return tl::expected<ThermostatMode, QuickI2CStatus>
     */
    inline tl::expected<ThermostatMode, QuickI2CStatus> getThermostatMode() {
        auto configuration = readConfiguration();
        if(!configuration) {
            return tl::unexpected<QuickI2CStatus>(configuration.error());
        }
        return static_cast<ThermostatMode>(*configuration & ConfigurationThermostatModeMask);
    }

    /**
     * @brief Set alert pin polarity.
     * @param polarity ActiveLow or ActiveHigh.
     * @return QuickI2CStatus
     */
    inline QuickI2CStatus setAlertPolarity(AlertPolarity polarity) {
        return updateConfiguration(ConfigurationPolarityMask, static_cast<uint8_t>(polarity));
    }

    /**
     * @brief Get alert pin polarity.
     * @return tl::expected<AlertPolarity, QuickI2CStatus>
     */
    inline tl::expected<AlertPolarity, QuickI2CStatus> getAlertPolarity() {
        auto configuration = readConfiguration();
        if(!configuration) {
            return tl::unexpected<QuickI2CStatus>(configuration.error());
        }
        return static_cast<AlertPolarity>(*configuration & ConfigurationPolarityMask);
    }

    /**
     * @brief Set fault queue depth for thermostat events.
     * @param queue Number of consecutive faults required to assert alert.
     * @return QuickI2CStatus
     */
    inline QuickI2CStatus setFaultQueue(FaultQueue queue) {
        return updateConfiguration(ConfigurationFaultQueueMask, static_cast<uint8_t>(queue));
    }

    /**
     * @brief Get current fault queue setting.
     * @return tl::expected<FaultQueue, QuickI2CStatus>
     */
    inline tl::expected<FaultQueue, QuickI2CStatus> getFaultQueue() {
        auto configuration = readConfiguration();
        if(!configuration) {
            return tl::unexpected<QuickI2CStatus>(configuration.error());
        }
        return static_cast<FaultQueue>(*configuration & ConfigurationFaultQueueMask);
    }

    /**
     * @brief Set ADC conversion resolution.
     * @param resolution One of 9..12-bit modes.
     * @return QuickI2CStatus
     */
    inline QuickI2CStatus setResolution(Resolution resolution) {
        return updateConfiguration(ConfigurationResolutionMask, static_cast<uint8_t>(resolution));
    }

    /**
     * @brief Get current ADC conversion resolution.
     * @return tl::expected<Resolution, QuickI2CStatus>
     */
    inline tl::expected<Resolution, QuickI2CStatus> getResolution() {
        auto configuration = readConfiguration();
        if(!configuration) {
            return tl::unexpected<QuickI2CStatus>(configuration.error());
        }
        return static_cast<Resolution>(*configuration & ConfigurationResolutionMask);
    }

    /**
     * @brief Get typical conversion delay by resolution in ms.
     * @param resolution Conversion resolution.
     * @return uint16_t Delay in milliseconds.
     */
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

    /**
     * @brief Trigger a single temperature conversion (one-shot mode).
     * @return QuickI2CStatus
     */
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

    /**
     * @brief Read alert status (thermostat output) based on one-shot flag and polarity.
     * @return tl::expected<bool, QuickI2CStatus>
     */
    inline tl::expected<bool, QuickI2CStatus> readAlertStatus() {
        auto configuration = readConfiguration();
        if(!configuration) {
            return tl::unexpected<QuickI2CStatus>(configuration.error());
        }

        bool osAlertBit = ((*configuration) & ConfigurationOneShotMask) != 0;
        bool activeHigh = ((*configuration) & ConfigurationPolarityMask) != 0;
        return activeHigh ? osAlertBit : !osAlertBit;
    }

    /**
     * @brief Clear alert status on the TMP10x (no-op for devices that don't support state clear).
     * @return QuickI2CStatus
     */
    inline QuickI2CStatus clearAlert() {
        auto configuration = readConfiguration();
        if(!configuration) {
            return configuration.error();
        }
        return QuickI2CStatus::OK;
    }

    /**
     * @brief Read raw 16-bit temperature register value.
     * @return tl::expected<uint16_t, QuickI2CStatus>
     */
    inline tl::expected<uint16_t, QuickI2CStatus> readTemperatureRegisterRaw() {
        return readRegister16(static_cast<uint8_t>(Register::Temperature));
    }

    /**
     * @brief Read and decode raw temperature value from register.
     * @return tl::expected<int16_t, QuickI2CStatus>
     */
    inline tl::expected<int16_t, QuickI2CStatus> readTemperatureRaw() {
        auto raw = readTemperatureRegisterRaw();
        if(!raw) {
            return tl::unexpected<QuickI2CStatus>(raw.error());
        }
        return decodeTemperatureRegister(*raw);
    }

    /**
     * @brief Read the temperature in degrees Celsius.
     * @return tl::expected<float, QuickI2CStatus>
     */
    inline tl::expected<float, QuickI2CStatus> readTemperatureCelsius() {
        auto raw = readTemperatureRaw();
        if(!raw) {
            return tl::unexpected<QuickI2CStatus>(raw.error());
        }
        return rawTemperatureToCelsius(*raw);
    }

    /**
     * @brief Read raw TLow register value.
     * @return tl::expected<uint16_t, QuickI2CStatus>
     */
    inline tl::expected<uint16_t, QuickI2CStatus> readTLowRegisterRaw() {
        return readRegister16(static_cast<uint8_t>(Register::TLow));
    }

    /**
     * @brief Read decoded TLow value as signed int16.
     * @return tl::expected<int16_t, QuickI2CStatus>
     */
    inline tl::expected<int16_t, QuickI2CStatus> readTLowRaw() {
        auto raw = readTLowRegisterRaw();
        if(!raw) {
            return tl::unexpected<QuickI2CStatus>(raw.error());
        }
        return decodeTemperatureRegister(*raw);
    }

    /**
     * @brief Read TLow threshold in degrees Celsius.
     * @return tl::expected<float, QuickI2CStatus>
     */
    inline tl::expected<float, QuickI2CStatus> readTLowCelsius() {
        auto raw = readTLowRaw();
        if(!raw) {
            return tl::unexpected<QuickI2CStatus>(raw.error());
        }
        return rawTemperatureToCelsius(*raw);
    }

    /**
     * @brief Write TLow threshold value (raw 16-bit register quantity).
     * @param value Raw register value.
     * @return QuickI2CStatus
     */
    inline QuickI2CStatus writeTLowRegisterRaw(uint16_t value) {
        return writeRegister16(static_cast<uint8_t>(Register::TLow), value);
    }

    /**
     * @brief Write and verify TLow raw register.
     * @param value Raw register value.
     * @return QuickI2CStatus
     */
    inline QuickI2CStatus writeAndVerifyTLowRegisterRaw(uint16_t value) {
        return writeAndVerifyRegister16(static_cast<uint8_t>(Register::TLow), value);
    }

    /**
     * @brief Write TLow threshold in raw scaled int16 temperature units.
     * @param value Signed temperature value from decode steps (format 1/16 °C).
     * @return QuickI2CStatus
     */
    inline QuickI2CStatus writeTLowRaw(int16_t value) {
        return writeTLowRegisterRaw(encodeTemperatureRegister(value));
    }

    /**
     * @brief Write and verify TLow threshold in raw scaled format.
     * @param value Signed temperature value from decode steps (format 1/16 °C).
     * @return QuickI2CStatus
     */
    inline QuickI2CStatus writeAndVerifyTLowRaw(int16_t value) {
        return writeAndVerifyTLowRegisterRaw(encodeTemperatureRegister(value));
    }

    /**
     * @brief Write TLow threshold in degrees Celsius.
     * @param value Temperature in °C.
     * @return QuickI2CStatus
     */
    inline QuickI2CStatus writeTLowCelsius(float value) {
        return writeTLowRaw(celsiusToRawTemperature(value));
    }

    /**
     * @brief Write and verify TLow threshold in degrees Celsius.
     * @param value Temperature in °C.
     * @return QuickI2CStatus
     */
    inline QuickI2CStatus writeAndVerifyTLowCelsius(float value) {
        return writeAndVerifyTLowRaw(celsiusToRawTemperature(value));
    }

    /**
     * @brief Read raw THigh register value.
     * @return tl::expected<uint16_t, QuickI2CStatus>
     */
    inline tl::expected<uint16_t, QuickI2CStatus> readTHighRegisterRaw() {
        return readRegister16(static_cast<uint8_t>(Register::THigh));
    }

    /**
     * @brief Read decoded THigh value as signed temperature raw register.
     * @return tl::expected<int16_t, QuickI2CStatus>
     */
    inline tl::expected<int16_t, QuickI2CStatus> readTHighRaw() {
        auto raw = readTHighRegisterRaw();
        if(!raw) {
            return tl::unexpected<QuickI2CStatus>(raw.error());
        }
        return decodeTemperatureRegister(*raw);
    }

    /**
     * @brief Read THigh threshold in degrees Celsius.
     * @return tl::expected<float, QuickI2CStatus>
     */
    inline tl::expected<float, QuickI2CStatus> readTHighCelsius() {
        auto raw = readTHighRaw();
        if(!raw) {
            return tl::unexpected<QuickI2CStatus>(raw.error());
        }
        return rawTemperatureToCelsius(*raw);
    }

    /**
     * @brief Write THigh threshold raw register value.
     * @param value Raw register value.
     * @return QuickI2CStatus
     */
    inline QuickI2CStatus writeTHighRegisterRaw(uint16_t value) {
        return writeRegister16(static_cast<uint8_t>(Register::THigh), value);
    }

    /**
     * @brief Write and verify THigh threshold raw register value.
     * @param value Raw register value.
     * @return QuickI2CStatus
     */
    inline QuickI2CStatus writeAndVerifyTHighRegisterRaw(uint16_t value) {
        return writeAndVerifyRegister16(static_cast<uint8_t>(Register::THigh), value);
    }

    /**
     * @brief Write THigh threshold in raw scaled temperature units.
     * @param value Signed temperature value (1/16 °C units).
     * @return QuickI2CStatus
     */
    inline QuickI2CStatus writeTHighRaw(int16_t value) {
        return writeTHighRegisterRaw(encodeTemperatureRegister(value));
    }

    /**
     * @brief Write and verify THigh threshold in raw scaled temperature units.
     * @param value Signed temperature value (1/16 °C units).
     * @return QuickI2CStatus
     */
    inline QuickI2CStatus writeAndVerifyTHighRaw(int16_t value) {
        return writeAndVerifyTHighRegisterRaw(encodeTemperatureRegister(value));
    }

    /**
     * @brief Write THigh threshold in degrees Celsius.
     * @param value Temperature in °C.
     * @return QuickI2CStatus
     */
    inline QuickI2CStatus writeTHighCelsius(float value) {
        return writeTHighRaw(celsiusToRawTemperature(value));
    }

    /**
     * @brief Write and verify THigh threshold in degrees Celsius.
     * @param value Temperature in °C.
     * @return QuickI2CStatus
     */
    inline QuickI2CStatus writeAndVerifyTHighCelsius(float value) {
        return writeAndVerifyTHighRaw(celsiusToRawTemperature(value));
    }

    /**
     * @brief Convert raw temperature value (1/16 °C fixed-point) to Celsius.
     * @param value Decoded raw temperature.
     * @return float Celsius.
     */
    inline static float rawTemperatureToCelsius(int16_t value) {
        return static_cast<float>(value) / 16.0f;
    }

    /**
     * @brief Convert Celsius to raw 1/16°C fixed point register value.
     * @param value Celsius temperature.
     * @return int16_t Raw encoded temperature.
     */
    inline static int16_t celsiusToRawTemperature(float value) {
        return static_cast<int16_t>(roundf(value * 16.0f));
    }

    /**
     * @brief Decode raw 16-bit temperature register value to signed 12-bit value.
     *
     * TMP10x stores temperature in upper 12 bits of the 16-bit register, with
     * sign extension.
     *
     * @param value Raw register value.
     * @return int16_t Decoded signed temperature value.
     */
    inline static int16_t decodeTemperatureRegister(uint16_t value) {
        int16_t raw = static_cast<int16_t>(value >> 4);
        if((raw & 0x0800) != 0) {
            raw |= static_cast<int16_t>(0xF000);
        }
        return raw;
    }

    /**
     * @brief Encode signed temperature value to raw register format.
     * @param value Decoded signed temperature value (1/16°C units).
     * @return uint16_t Raw register value.
     */
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