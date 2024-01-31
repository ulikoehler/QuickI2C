#include "QuickI2C.h"

const char* QuickI2CStatusToString(QuickI2CStatus status) {
    if(status == QuickI2CStatus::OK) {
        return "OK";
    } else if(status == QuickI2CStatus::SlaveNotResponding) {
        return "Slave not Responding";
    } else if(status == QuickI2CStatus::DataTimeout) {
        return "Data timeout";
    } else if(status == QuickI2CStatus::VerifyMismatch) {
        return "Verify mismatch";
    } else if(status == QuickI2CStatus::BusBusy) {
        return "Bus busy";
    } else if(status == QuickI2CStatus::DriverNotInitialized) {
        return "Driver not initialized";
    }  else if(status == QuickI2CStatus::UnknownError) {
        return "Unknown error";
    } else {
        return "Unknown";
    }
}

QuickI2CDevice::QuickI2CDevice(
    uint8_t address,
    #ifdef QUICKI2C_DRIVER_ARDUINO
        TwoWire& wire,
    #elif defined(QUICKI2C_DRIVER_ESPIDF)
        i2c_port_t port,
    #endif
    uint32_t i2cClockSpeed,
    uint32_t timeout
    ) :
    #ifdef QUICKI2C_DRIVER_ARDUINO
    wire(wire),
    #elif defined(QUICKI2C_DRIVER_ESPIDF),
    port(port),
    #endif
    deviceAddress(address), i2cClockSpeed(i2cClockSpeed), timeout(timeout) {
}

QuickI2CStatus QuickI2CDevice::writeData(uint8_t registerAddress, const uint8_t* buf, size_t len) {
    #ifdef QUICKI2C_DRIVER_ARDUINO
        // Configure clock speed & timeout
        wire.setClock(this->i2cClockSpeed);
        // Don't need timeout for write since we will never read
        // Transmit address
        wire.beginTransmission(this->deviceAddress);
        wire.write(registerAddress);
        wire.write(buf, len);
        #if ESP_ARDUINO_VERSION_MAJOR == 1
            wire.endTransmission();
            // Check for missing
            if(wire.lastError() == I2C_ERROR_ACK) {
                return QuickI2CStatus::SlaveNotResponding;
            }
            return QuickI2CStatus::OK;
        #else // ESP_ARDUINO_VERSION_MAJOR == 2
            uint8_t lastError =  wire.endTransmission();
            // "Magic" values: See https://github.com/espressif/arduino-esp32/blob/7bb30b3cf82426a6b6947b9f69dffa3b8f95dbc1/libraries/Wire/src/Wire.cpp#L348
            // TODO recheck
            if(lastError == 5) {
                return QuickI2CStatus::DataTimeout;
            } else {
                return QuickI2CStatus::OK;
            }
        #endif
    #elif defined(QUICKI2C_DRIVER_ESPIDF)
        txbuf[0] = registerAddress;
        memcpy(txbuf + 1, buf, len);
        esp_err_t err = i2c_master_write(port, deviceAddress, &registerAddress, 1, txbuf, len + 1 /* reg addr + data*/, timeout / portTICK_PERIOD_MS);
        switch(err) {
            case ESP_OK:
                return QuickI2CStatus::OK;
            case ESP_ERR_TIMEOUT:
                return QuickI2CStatus::BusBusy;
            case ESP_FAIL:
                return QuickI2CStatus::SlaveNotResponding;
            case ESP_ERR_INVALID_STATE:
                return QuickI2CStatus::DriverNotInitialized;
            default:
                return QuickI2CStatus::UnknownError;
        }
    #endif
}

QuickI2CStatus QuickI2CDevice::readData(uint8_t registerAddress, uint8_t* buf, size_t len) {
    #ifdef QUICKI2C_DRIVER_ARDUINO
        // Configure clock speed & timeout
        wire.setClock(this->i2cClockSpeed);
        wire.setTimeout(this->computeTimeout(len));
        // Transmit address
        wire.beginTransmission(this->deviceAddress);
        wire.write(registerAddress);
        wire.endTransmission();
        // Check for missing
        #if ESP_ARDUINO_VERSION_MAJOR == 1
            wire.endTransmission();
            // Check for missing
            if(wire.lastError() == I2C_ERROR_ACK) {
                return QuickI2CStatus::SlaveNotResponding;
            }
            return QuickI2CStatus::OK;
        #else // ESP_ARDUINO_VERSION_MAJOR == 2
            uint8_t lastError =  wire.endTransmission();
            // "Magic" values: See https://github.com/espressif/arduino-esp32/blob/7bb30b3cf82426a6b6947b9f69dffa3b8f95dbc1/libraries/Wire/src/Wire.cpp#L348
            // TODO recheck
            if(lastError == 5) {
                return QuickI2CStatus::DataTimeout;
            }
        #endif
        // Receive data
        wire.requestFrom(this->address, len);
        size_t actuallyRead = wire.readBytes(buf, len);
        if(actuallyRead < len) {
            // Did not read enough bytes
            this->dataBytesReadUntilTimeout = actuallyRead;
            return QuickI2CStatus::DataTimeout;
        }
        return QuickI2CStatus::OK;
    #elif defined(QUICKI2C_DRIVER_ESPIDF)
        esp_err_t err = i2c_master_write_read_device(port, deviceAddress, &registerAddress, 1, buf, len, timeout / portTICK_PERIOD_MS);
        switch(err) {
            case ESP_OK:
                return QuickI2CStatus::OK;
            case ESP_ERR_TIMEOUT:
                return QuickI2CStatus::BusBusy;
            case ESP_FAIL:
                return QuickI2CStatus::SlaveNotResponding;
            case ESP_ERR_INVALID_STATE:
                return QuickI2CStatus::DriverNotInitialized;
            default:
                return QuickI2CStatus::UnknownError;
        }
    #endif
}

QuickI2CStatus QuickI2CDevice::writeAndVerifyData(uint8_t readAddress, uint8_t writeAddress, const uint8_t* buf, size_t len) {
    // Try to write - if it fails, do not try to verify
    QuickI2CStatus rc = writeData(writeAddress, buf, len);
    if(rc != QuickI2CStatus::OK) {
        return rc;
    }
    // Insert grace time between write and read
    delay(delayBetweenWriteAndRead);
    // Read back data for verify
    rc = readData(readAddress, rxbuf, len);
    if(rc != QuickI2CStatus::OK) {
        return rc;
    }
    // Compare data
    if(memcmp(rxbuf, buf, len) != 0) {
        // rxbuf is not the same as buf
        return QuickI2CStatus::VerifyMismatch;
    }
    return QuickI2CStatus::OK;
}

tl::expected<uint8_t, QuickI2CStatus> QuickI2CDevice::read8BitRegister(uint8_t registerAddress) {
    uint8_t ret = 0;
    QuickI2CStatus status = readData(registerAddress, (uint8_t*)&ret, 1);
    if(status != QuickI2CStatus::OK) {
        return tl::unexpected<QuickI2CStatus>(status);
    }
    return ret;
}

tl::expected<uint16_t, QuickI2CStatus> QuickI2CDevice::read16BitRegister(uint8_t registerAddress) {
    uint16_t ret = 0;
    QuickI2CStatus status = readData(registerAddress, (uint8_t*)&ret, 2);
    if(status != QuickI2CStatus::OK) {
        return tl::unexpected<QuickI2CStatus>(status);
    }
    return ret;
}

tl::expected<uint32_t, QuickI2CStatus> QuickI2CDevice::read24BitRegister(uint8_t registerAddress) {
    uint32_t ret = 0;
    QuickI2CStatus status = readData(registerAddress, (uint8_t*)&ret, 3);
    if(status != QuickI2CStatus::OK) {
        return tl::unexpected<QuickI2CStatus>(status);
    }
    return ret;
}

tl::expected<uint32_t, QuickI2CStatus> QuickI2CDevice::read32BitRegister(uint8_t registerAddress) {
    uint32_t ret = 0;
    QuickI2CStatus status = readData(registerAddress, (uint8_t*)&ret, 4);
    if(status != QuickI2CStatus::OK) {
        return tl::unexpected<QuickI2CStatus>(status);
    }
    return ret;
}

QuickI2CStatus QuickI2CDevice::write8BitRegister(uint8_t registerAddress, uint8_t value) {
    return writeData(registerAddress, (uint8_t*)&value, 1);
}

QuickI2CStatus QuickI2CDevice::write16BitRegister(uint8_t registerAddress, uint16_t value) {
    return writeData(registerAddress, (uint8_t*)&value, 2);
}

QuickI2CStatus QuickI2CDevice::write24BitRegister(uint8_t registerAddress, uint32_t value) {
    return writeData(registerAddress, (uint8_t*)&value, 3);
}

QuickI2CStatus QuickI2CDevice::write32BitRegister(uint8_t registerAddress, uint32_t value) {
    return writeData(registerAddress, (uint8_t*)&value, 4);
}

QuickI2CStatus QuickI2CDevice::writeAndVerify8BitRegister(uint8_t readAddress, uint8_t writeAddress, uint8_t value) {
    return writeAndVerifyData(readAddress, writeAddress, (uint8_t*)&value, 1);
}

QuickI2CStatus QuickI2CDevice::writeAndVerify16BitRegister(uint8_t readAddress, uint8_t writeAddress, uint16_t value) {
    return writeAndVerifyData(readAddress, writeAddress, (uint8_t*)&value, 2);
}

QuickI2CStatus QuickI2CDevice::writeAndVerify24BitRegister(uint8_t readAddress, uint8_t writeAddress, uint32_t value) {
    return writeAndVerifyData(readAddress, writeAddress, (uint8_t*)&value, 3);
}

QuickI2CStatus QuickI2CDevice::writeAndVerify32BitRegister(uint8_t readAddress, uint8_t writeAddress, uint32_t value) {
    return writeAndVerifyData(readAddress, writeAddress, (uint8_t*)&value, 4);
}

uint32_t QuickI2CDevice::computeTimeout(uint32_t bytesToTransfer) {
    uint32_t numBits = bytesToTransfer * 9; // 8 data bits + 1 ACK/NACK bit per byte
    // NOTE: We avoid to use floating point arithmetic here.
    uint32_t durationMilliseconds = numBits * 1000 / this->i2cClockSpeed; // This will ALWAYS be rounded down!
    // Give an extra millisecond so we have ensured to always round up
    return durationMilliseconds + 2;
}