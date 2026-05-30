#include <QuickI2C.h>

namespace {

class SmokeDevice : public QuickI2CDevice {
public:
    using QuickI2CDevice::QuickI2CDevice;
};

static_assert(std::is_base_of<QuickI2CDevice, SmokeDevice>::value, "SmokeDevice must derive from QuickI2CDevice");

}

extern "C" void app_main(void) {
    i2c_master_bus_config_t bus_config = {};
    bus_config.i2c_port = I2C_NUM_0;
    bus_config.sda_io_num = GPIO_NUM_21;
    bus_config.scl_io_num = GPIO_NUM_22;
    bus_config.clk_source = I2C_CLK_SRC_DEFAULT;

    (void)bus_config;
}
