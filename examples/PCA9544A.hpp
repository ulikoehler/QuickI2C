#pragma once
#include <QuickI2C.h>
#include <driver/i2c.h>
#include <math.h>

class PCA9544A : public QuickI2CDevice {
public:
    inline PCA9544A(i2c_port_t port): QuickI2CDevice(0x70 /* Address */, port, 400000) {}

    inline void SetSwitch(uint8_t position, bool enable=true) {
        // This component does not use the usual address/data scheme,
        // but the address itself is the data address to enable
        writeData(position | ((enable ? 1 : 0) << 2), nullptr, 0);
    }
};