#pragma once
#include <QuickI2C.h>
#include <math.h>

/**
 * Usage example:
 *
 *     PCA9544A mux(QUICKI2C_DEFAULT_PORT);
 *     if (mux.SetSwitch(2, true) != QuickI2CStatus::OK) {
 *         printf("PCA9544A switch set failed\n");
 *     }
 */
class PCA9544A : public QuickI2CDevice {
public:
    /**
     * @brief Construct a PCA9544A I2C channel multiplexer.
    * @param port QuickI2C bus port.
     */
    inline PCA9544A(QuickI2CPort port = QUICKI2C_DEFAULT_PORT): QuickI2CDevice(0x70 /* Address */, port, 400000) {}

    /**
     * @brief Enable or disable a channel on the multiplexer.
     *
     * PCA9544A addresses channel select via the device address bits, so this
     * operation writes the channel index and enable bit as data encoded in
     * the 8-bit command.
     *
     * @param position Channel number (0..3)
     * @param enable true to enable the channel, false to disable.
     * @return QuickI2CStatus
     */
    inline QuickI2CStatus SetSwitch(uint8_t position, bool enable=true) {
        // This component does not use the usual address/data scheme,
        // but the address itself is the data address to enable
        return writeData(position | ((enable ? 1 : 0) << 2), nullptr, /*len = */0);
    }
};