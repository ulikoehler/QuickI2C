#pragma once

#include <stdint.h>
#include <stddef.h>

// Find the correct driver
#if defined(ARDUINO) && !defined(QUICKI2C_DISABLE_ARDUINO)
    #define QUICKI2C_DRIVER_ARDUINO
#elif defined(IDF_VER) && !defined(QUICKI2C_DISABLE_ESP_IDF)
    #define QUICKI2C_DRIVER_ESPIDF
#else
    #error "Could not determine I2C driver (Arduino or ESP-IDF)"
#endif


// Include the correct driver
#ifdef QUICKI2C_DRIVER_ARDUINO
#include <Arduino.h>
#include <Wire.h>
#elif defined(QUICKI2C_DRIVER_ESPIDF)
#include <driver/i2c.h>
#endif

#include "expected.hpp"

#include <type_traits>

#if !defined(QUICKI2C_NO_ENUM_OPERATIONS) && !defined(QUICKSPI_ENUM_OPERATIONS)
#define QUICKI2C_ENUM_OPERATIONS

/**
 * @brief Define ORing of two enum classes (register definitions)
 */
template<class T>
constexpr typename std::enable_if<std::is_enum<T>::value, T>::type operator|(T lhs, T rhs) 
{
    return static_cast<T>(
        static_cast<typename std::underlying_type<T>::type>(lhs) | 
        static_cast<typename std::underlying_type<T>::type>(rhs));
}

/**
 * @brief Define ORing of an enum class and an integer
 */
template<class T, class I>
constexpr typename std::enable_if<std::is_enum<T>::value && std::is_integral<I>::value, I>::type operator|(T lhs, I rhs) 
{
    return static_cast<typename std::underlying_type<T>::type>(lhs) | rhs;
}
// Reversed argument order
template<class T, class I>
constexpr typename std::enable_if<std::is_enum<T>::value && std::is_integral<I>::value, I>::type operator|(I lhs, T rhs)
{
    return lhs | static_cast<typename std::underlying_type<T>::type>(rhs);
}

/**
 * @brief Define ORing of an enum class and an integer
 */
template<class T, class I>
constexpr typename std::enable_if<std::is_enum<T>::value && std::is_integral<I>::value, I>::type operator&(T lhs, I rhs) 
{
    return static_cast<typename std::underlying_type<T>::type>(lhs) & rhs;
}
// Reversed argument order
template<class T, class I>
constexpr typename std::enable_if<std::is_enum<T>::value && std::is_integral<I>::value, I>::type operator&(I lhs, T rhs)
{
    return lhs & static_cast<typename std::underlying_type<T>::type>(rhs);
}

/**
 * @brief Define adding of enum classes (register definitions) and numbers
 */
template<class T, class I>
constexpr typename std::enable_if<std::is_enum<T>::value && std::is_integral<I>::value, I>::type operator+(T lhs, I rhs) 
{
    return static_cast<typename std::underlying_type<T>::type>(lhs) + rhs;
}
// Reversed argument order
template<class T, class I>
constexpr typename std::enable_if<std::is_enum<T>::value && std::is_integral<I>::value, I>::type operator+(I lhs, T rhs) 
{
    return lhs + static_cast<typename std::underlying_type<T>::type>(rhs);
}

/**
 * @brief Define subtraction of enum classes (register definitions) and numbers
 */
template<class T, class I>
constexpr typename std::enable_if<std::is_enum<T>::value && std::is_integral<I>::value, I>::type operator-(T lhs, I rhs) 
{
    return static_cast<typename std::underlying_type<T>::type>(lhs) - rhs;
}
// Reversed argument order
template<class T, class I>
constexpr typename std::enable_if<std::is_enum<T>::value && std::is_integral<I>::value, I>::type operator-(I lhs, T rhs) 
{
    return lhs - static_cast<typename std::underlying_type<T>::type>(rhs);

}

/**
 * @brief Define multiplication of enum classes (register definitions) and numbers
 */
template<class T, class I>
constexpr typename std::enable_if<std::is_enum<T>::value && std::is_integral<I>::value, I>::type operator*(T lhs, I rhs) 
{
    return static_cast<typename std::underlying_type<T>::type>(lhs) * rhs;
}
// Reversed argument order
template<class T, class I>
constexpr typename std::enable_if<std::is_enum<T>::value && std::is_integral<I>::value, I>::type operator*(I lhs, T rhs) 
{
    return lhs * static_cast<typename std::underlying_type<T>::type>(rhs);

}

#endif // QUICKI2C_NO_ENUM_OPERATIONS

enum class QuickI2CStatus : int8_t {
    OK = 0,
    /**
     * If you see this error code, check the device address - reason:
     * Missing ACK on the bus, possibly caused by write size mismatch
     */
    SlaveNotResponding = 1,
    /**
     * Device responded to address write but did not send enough/any data within timeout.
     * Check dataBytesReadUntilTimeout!
     */
    DataTimeout = 3,
    /**
     * Write succeeded but reading the register did not result in the correct value.
     * Check the rxbuf to see what the chip returned.
     */
    VerifyMismatch = 4, // Write succeeded, but verify failed
    /**
     * The IO operation did not succeed because the bus was busy
     * and not unlocked within the timeout
     */
    BusBusy = 5,
    /**
     * The IO operation did not succeed because the I2C device
     * has not been initialized properly
     */
    DriverNotInitialized = 6,
    UnknownError = 7,
};

const char* QuickI2CStatusToString(QuickI2CStatus status);

// Pre/postprocess functions that do nothing
inline uint8_t noop(uint8_t address, uint8_t rawValue) {return rawValue;}
inline uint16_t noop(uint8_t address, uint16_t rawValue) {return rawValue;}
inline uint32_t noop(uint8_t address, uint32_t rawValue) {return rawValue;}

// Pre/postprocess functions that swap the byte order (but do not change the bit order)
inline uint16_t invertByteorder16(uint8_t address, uint16_t rawValue) {return __builtin_bswap16(rawValue);}
inline uint32_t invertByteorder24(uint8_t address, uint32_t rawValue) {return __builtin_bswap32(rawValue) >> 8;}
inline uint32_t invertByteorder32(uint8_t address, uint32_t rawValue) {return __builtin_bswap32(rawValue);}

typedef uint8_t (*Postprocessor8Bit)(uint8_t, uint8_t);
typedef uint16_t (*Postprocessor16Bit)(uint8_t, uint16_t);
typedef uint32_t (*Postprocessor24Bit)(uint8_t, uint32_t);
typedef uint32_t (*Postprocessor32Bit)(uint8_t, uint32_t);

/**
 * Define a Read-Write register and its associated functions on
 * 
 * DO NOT terminate these macro calls with a semicolon!
 */
#define _DEFINE_READ_WRITE_ADDRESS_MEMBERS(name, raddr, waddr)\
static constexpr uint8_t name##ReadAddress = (raddr);\
static constexpr uint8_t name##WriteAddress = (waddr);\

#define _EXPECTED_TRANSFORM8(raddr, postproc, read) read8BitRegister((raddr)).transform([](uint8_t val) {return (postproc)((raddr), val);})
#define _EXPECTED_TRANSFORM16(raddr, postproc, read) read16BitRegister((raddr)).transform([](uint16_t val) {return (postproc)((raddr), val);})
#define _EXPECTED_TRANSFORM24(raddr, postproc, read) read24BitRegister((raddr)).transform([](uint32_t val) {return (postproc)((raddr), val);})
#define _EXPECTED_TRANSFORM32(raddr, postproc, read) read32BitRegister((raddr)).transform([](uint32_t val) {return (postproc)((raddr), val);})

/**
 * Define a Read-Write register and its associated functions on
 * 
 * DO NOT terminate these macro calls with a semicolon!
 */
#define QUICKI2C_DEFINE_REGISTER8_RW(name, raddr, waddr)\
enum class name : uint8_t;\
_DEFINE_READ_WRITE_ADDRESS_MEMBERS(name, raddr, waddr)\
inline tl::expected<uint8_t, QuickI2CStatus> read##name() {return _EXPECTED_TRANSFORM8(raddr, postprocessRead8, read8BitRegister);}\
inline void write##name(uint8_t val) {write8BitRegister((waddr), postprocessWrite8((waddr), val));}\
inline void write##name(name val) {write8BitRegister((waddr), postprocessWrite8((waddr), static_cast<uint8_t>(val)));}\
inline QuickI2CStatus writeAndVerify##name(uint8_t val) {return writeAndVerify8BitRegister((raddr), (waddr), postprocessWrite8((waddr), val));}\
enum class name : uint8_t

#define QUICKI2C_DEFINE_REGISTER16_RW(name, raddr, waddr)\
enum class name : uint16_t;\
_DEFINE_READ_WRITE_ADDRESS_MEMBERS(name, raddr, waddr)\
inline tl::expected<uint16_t, QuickI2CStatus> read##name() {return postprocessRead16((raddr), read16BitRegister((raddr)));}\
inline void write##name(uint16_t val) {write16BitRegister((waddr), postprocessWrite16((waddr), val));}\
inline void write##name(name val) {write16BitRegister((waddr), postprocessWrite16((waddr), static_cast<uint16_t>(val)));}\
inline QuickI2CStatus writeAndVerify##name(uint16_t val) {return writeAndVerify16BitRegister((raddr), (waddr), postprocessWrite16((waddr), val));}\
enum class name : uint16_t

#define QUICKI2C_DEFINE_REGISTER24_RW(name, raddr, waddr)\
enum class name : uint32_t;\
_DEFINE_READ_WRITE_ADDRESS_MEMBERS(name, raddr, waddr)\
inline tl::expected<uint32_t, QuickI2CStatus> read##name() {return postprocessRead24((raddr), read24BitRegister((raddr)));}\
inline void write##name(uint32_t val) {return write24BitRegister((waddr), postprocessWrite24((waddr), val));}\
inline void write##name(name val) {return write24BitRegister((waddr), postprocessWrite24((waddr), static_cast<uint32_t>(val)));}\
inline QuickI2CStatus writeAndVerify##name(uint32_t val) {return writeAndVerify24BitRegister((raddr), (waddr), postprocessWrite24((waddr), val));}\
enum class name : uint32_t

#define QUICKI2C_DEFINE_REGISTER32_RW(name, raddr, waddr)\
enum class name : uint32_t;\
_DEFINE_READ_WRITE_ADDRESS_MEMBERS(name, raddr, waddr)\
inline tl::expected<uint32_t, QuickI2CStatus> read##name() {return postprocessRead32((raddr), read32BitRegister((raddr)));}\
inline void write##name(uint32_t val) {return write32BitRegister((waddr), postprocessWrite32((waddr), val));}\
inline void write##name(name val) {return write16BitRegister((waddr), postprocessWrite32((waddr), static_cast<uint32_t>(val)));}\
inline QuickI2CStatus writeAndVerify##name(uint32_t val) {return writeAndVerify32BitRegister((raddr), (waddr), postprocessWrite32((waddr), val));}\
enum class name : uint32_t

// Read-only register definitions
#define QUICKI2C_DEFINE_REGISTER8_RO(name, addr)\
static constexpr uint8_t name##Address = addr;\
inline tl::expected<uint8_t, QuickI2CStatus> read##name() {return _EXPECTED_TRANSFORM8(addr, postprocessRead8, read8BitRegister);}\
enum class name : uint8_t

#define QUICKI2C_DEFINE_REGISTER16_RO(name, addr)\
static constexpr uint8_t name##Address = addr;\
inline tl::expected<uint16_t, QuickI2CStatus> read##name() {return _EXPECTED_TRANSFORM16(addr, postprocessRead16, read16BitRegister);}\
enum class name : uint16_t

#define QUICKI2C_DEFINE_REGISTER24_RO(name, addr)\
static constexpr uint8_t name##Address = addr;\
inline tl::expected<uint32_t, QuickI2CStatus> read##name() {return _EXPECTED_TRANSFORM32(addr, postprocessRead24, read24BitRegister);}\
enum class name : uint32_t

#define QUICKI2C_DEFINE_REGISTER32_RO(name, addr)\
static constexpr uint8_t name##Address = addr;\
inline tl::expected<uint32_t, QuickI2CStatus> read##name() {return _EXPECTED_TRANSFORM32(addr, postprocessRead32, read32BitRegister);}\
enum class name : uint32_t

/**
 * Represents a single device on the I2C bus.
 * 
 * This will automatically manage the timeout and clock speed of the I2C interface,
 * automatically computing a sensible timeout value to use depending on the clock speed:
 * It will not wait the default of one second, but use an adaptive timeout depending on the
 * number of bytes to be transferred
 * 
 * You can have multiple QuickI2CDevice instances, each with a different clock speed.
 * QuickI2CDevice will managed the clock speed, but it will not automagically manage the signal integrity!
 * You need to ensure if using a higher speed like fast mode plus that the slower devices will never
 * think they are being selected. The best way to absolutely ensure proper function is to just always
 * use speeds all devices support, or to hard-disconnect devices not supporting the highest speed
 * using an I2C multiplexer.
 * 
 * Note: This class is not inherently thread-safe and does not perform locking by itself.
 * Either ensure that not concurrent accesses are possible or perform locking.
 */
class QuickI2CDevice {
public:
    QuickI2CDevice(
        uint8_t address,
        #ifdef QUICKI2C_DRIVER_ARDUINO
        TwoWire& wire = Wire,
        #elif defined(QUICKI2C_DRIVER_ESPIDF)
        i2c_port_t port = I2C_NUM_0,
        #endif
        uint32_t i2cClockSpeed = 400000, /* Hz */
        uint32_t timeout = 100 /* ms */
    );

    tl::expected<uint8_t, QuickI2CStatus>  read8BitRegister(uint8_t registerAddress);
    tl::expected<uint16_t, QuickI2CStatus>  read16BitRegister(uint8_t registerAddress);
    tl::expected<uint32_t, QuickI2CStatus>  read24BitRegister(uint8_t registerAddress);
    tl::expected<uint32_t, QuickI2CStatus>  read32BitRegister(uint8_t registerAddress);
    
    QuickI2CStatus readData(uint8_t registerAddress, uint8_t* buf, size_t len);

    QuickI2CStatus write8BitRegister(uint8_t registerAddress, uint8_t value);
    QuickI2CStatus write16BitRegister(uint8_t registerAddress, uint16_t value);
    QuickI2CStatus write24BitRegister(uint8_t registerAddress, uint32_t value);
    QuickI2CStatus write32BitRegister(uint8_t registerAddress, uint32_t value);

    /**
     * Write data, storing the received data in the given buffer.
     * NOTE: This will discard data received while transmitting the address byte,
     * storing overall [len] bytes in buf
     */
    QuickI2CStatus writeAndReceiveData(uint8_t registerAddress, uint8_t* buf, size_t len);
    /**
     * Write data, discarding the received data
     */
    QuickI2CStatus writeData(uint8_t registerAddress, const uint8_t* buf, size_t len);

    /**
     * @brief Writes data to a register and verifies if the data has been written correctly by reading back the register
     * and comparing with the original value.
     * 
     */
    QuickI2CStatus writeAndVerify8BitRegister(uint8_t readAddress, uint8_t writeAddress, uint8_t value);
    QuickI2CStatus writeAndVerify16BitRegister(uint8_t readAddress, uint8_t writeAddress, uint16_t value);
    QuickI2CStatus writeAndVerify24BitRegister(uint8_t readAddress, uint8_t writeAddress, uint32_t value);
    QuickI2CStatus writeAndVerify32BitRegister(uint8_t readAddress, uint8_t writeAddress, uint32_t value);
    QuickI2CStatus writeAndVerifyData(uint8_t readAddress, uint8_t writeAddress, const uint8_t* buf, size_t len);
    /**
     * This is for debugging purposes.
     * For the previous read, it stores how many data bytes were validly read from the device
     * before the error condition occured.
     * In case the previous operation does not return DeviceNACKDuringData or DataTimeout
     */
    uint32_t dataBytesReadUntilTimeout;

    // Delay in milliseconds between write and read during writeAndVerify.
    uint32_t delayBetweenWriteAndRead = 1;
protected:
    /**
     * Compute the maximum timeout in milliseconds
     * that transferring [bytesToTransfer] could take
     * (plus some extra for safety)
     */
    uint32_t computeTimeout(size_t bytesToTransfer);
    #ifdef QUICKI2C_DRIVER_ESPIDF
    uint8_t txbuf[32];
    #endif
    uint8_t rxbuf[32]; // For write & verify

    #ifdef QUICKI2C_DRIVER_ARDUINO
    TwoWire& wire;
    #elif defined(QUICKI2C_DRIVER_ESPIDF)
    i2c_port_t port;
    #endif
    /**
     * Device address on the I2C bus.
     */
    uint8_t deviceAddress;
    /**
     * timeout in milliseconds
     */
    uint32_t timeout;

    uint32_t i2cClockSpeed;
};