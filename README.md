# QuickI2C

Practical Arduino Wire I2C wrapper, allowing direct register access with proper error handling.

This is intended to be similar to [AdaFruit BusIO](https://github.com/adafruit/Adafruit_BusIO) but provide a more convenient API and less memory footprint. For example, every [Adafruit_BusIO_Register](https://github.com/adafruit/Adafruit_BusIO/blob/master/Adafruit_BusIO_Register.h) is an actual object occupying a bunch of bytes of RAM and does not provide verified read functionality.

## How?

QuickI2C provides classes macro tricks to automatically define appropriate functions for any register, with zero memory footprint for additional registers.

## Installation

In PlatformIO, add the library by adding the following to `platformio.ini`:

```ini
lib_deps =
    ulikoehler/QuickI2C@^1.1
```

## ESP-IDF driver selection

When building with ESP-IDF, QuickI2C now supports three selection modes via `menuconfig`:

- `Automatic`: use the legacy `driver/i2c.h` backend before ESP-IDF 6.0 and the new `driver/i2c_master.h` backend on ESP-IDF 6.0 and later.
- `Legacy driver/i2c.h`: always use the deprecated legacy backend.
- `New driver/i2c_master.h`: always use the new master-bus backend.

## Portable port type

Examples use `QuickI2CPort` and `QUICKI2C_DEFAULT_PORT` instead of platform-specific port types. On Arduino this maps to `Wire`, and on ESP-IDF it maps to `I2C_NUM_0` by default.
