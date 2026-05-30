#include <Wire.h>
#include <QuickI2C.h>

class SmokeDevice : public QuickI2CDevice {
public:
    SmokeDevice(TwoWire& wire, uint8_t addr)
        : QuickI2CDevice(wire, addr) {}
};

SmokeDevice smokeDevice(Wire, 0x42);

void setup() {
    (void)smokeDevice;
}

void loop() {
}
