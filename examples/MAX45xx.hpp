#pragma once
#include <QuickI2C.h>

/**
 * MCP45xx volatile DigiPot
 */
class MCP45xx : public QuickI2CDevice {
public:
    inline MCP45xx(): QuickI2CDevice(0b0101110, Wire, 25000) {}

    static constexpr uint8_t CMDWriteData = 0b00 << 2;
    static constexpr uint8_t CMDReadData = 0b11 << 2;
    static constexpr uint8_t VolatileWiper0Address = 0x00;
    static constexpr uint8_t VolatileWiper1Address = 0x01;
    static constexpr uint8_t TCONAddress = 0x04;

    inline QuickI2CStatus readVolatileWiper0(uint16_t* buf) {
        return read16BitRegister((VolatileWiper0Address << 4) | CMDReadData, buf);
    }

    inline QuickI2CStatus writeVolatileWiper0(uint16_t value) {
        // The command byte contains the two MSBs of the value
        uint8_t twoMSBs = (value & 0b1100000000) >> 8;
        return write8BitRegister((VolatileWiper0Address << 4) | CMDWriteData | twoMSBs, value & 0xFF);
    }

    inline QuickI2CStatus readTCON(uint16_t* buf) {
        return read16BitRegister((TCONAddress << 4) | CMDReadData, buf);
    }

    inline QuickI2CStatus writeTCON(uint16_t value) {
        // The command byte contains the two MSBs of the value
        uint8_t twoMSBs = (value & 0b1100000000) >> 8;
        return write16BitRegister((TCONAddress << 4) | CMDWriteData | twoMSBs, value & 0xFF);
    }

};
