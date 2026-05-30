# QuickI2C

Practical Arduino Wire I2C wrapper, allowing direct register access with proper error handling.

This is intended to be similar to [AdaFruit BusIO](https://github.com/adafruit/Adafruit_BusIO) but provide a more convenient API and less memory footprint. For example, every [Adafruit_BusIO_Register](https://github.com/adafruit/Adafruit_BusIO/blob/master/Adafruit_BusIO_Register.h) is an actual object occupying a bunch of bytes of RAM and does not provide verified read functionality.

## ESP-IDF Compatibility

QuickI2C supports both ESP-IDF 5 and ESP-IDF 6 automatically.

- On ESP-IDF 5, the component links against the classic `driver` component.
- On ESP-IDF 6, the component auto-detects the split driver layout and adds `esp_driver_i2c` as a required dependency.

No user-side CMake changes are required for this version split.

## How?

QuickI2C provides classes macro tricks to automatically define appropriate functions for any register, with zero memory footprint for additional registers.

## Installation

In PlatformIO, add the library by adding the following to `platformio.ini`:

```ini
lib_deps =
    ulikoehler/QuickI2C@^2.2
```

## ESP-IDF driver selection

When building with ESP-IDF, QuickI2C now supports three selection modes via `menuconfig`:

- `Automatic`: use the legacy `driver/i2c.h` backend before ESP-IDF 6.0 and the new `driver/i2c_master.h` backend on ESP-IDF 6.0 and later.
- `Legacy driver/i2c.h`: always use the deprecated legacy backend.
- `New driver/i2c_master.h`: always use the new master-bus backend.

## Portable port type

Examples use `QuickI2CPort` and `QUICKI2C_DEFAULT_PORT` instead of platform-specific port types. On Arduino this maps to `Wire`, and on ESP-IDF it maps to `I2C_NUM_0` by default.
