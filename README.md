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
