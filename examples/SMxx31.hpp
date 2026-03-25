#pragma once
#include <QuickI2C.h>
#include <driver/i2c.h>
#include <math.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <esp_log.h>

/**
 * NOTE: The scaling & offset constants for SMxx31 sensors
 * are part-number-specific and are, for this example,
 * verified on the SM4331-BCE-S-300-000.
 * If you have another MPN, you need to change the constants, check the datasheet!
 *
 * Usage example:
 *
 *     SMxx31<> sensor(I2C_NUM_0);
 *     auto pressure = sensor.readPressure();
 *     if (pressure) {
 *         printf("Pressure: %.2f Pa\n", *pressure);
 *     } else {
 *         printf("Pressure read failed: %d\n", (int)pressure.error());
 *     }
 */
template<
    // Read filters
    Postprocessor8Bit postprocessRead8=noop,
    Postprocessor16Bit postprocessRead16=noop,
    Postprocessor24Bit postprocessRead24=noop,
    Postprocessor32Bit postprocessRead32=noop,
    // Write filters
    Postprocessor8Bit postprocessWrite8=noop,
    Postprocessor16Bit postprocessWrite16=noop,
    Postprocessor24Bit postprocessWrite24=noop,
    Postprocessor32Bit postprocessWrite32=noop
>
class SMxx31 : public QuickI2CDevice {
private:
    SemaphoreHandle_t pressure_mutex;

public:
    /**
     * @brief Construct a new SMxx31 object.
     *
     * @param port I2C port (I2C_NUM_0 or I2C_NUM_1)
     */
    inline SMxx31(i2c_port_t port): QuickI2CDevice(0x6C, port, 400000) {
        // Create mutex for thread-safe pressure readings
        pressure_mutex = xSemaphoreCreateMutex();
        if (pressure_mutex == NULL) {
            ESP_LOGE("SMxx31", "Failed to create pressure reading mutex! Thread safety not guaranteed.");
        }
    }
    
    /**
     * @brief Destroy the SMxx31 object and release mutex.
     */
    inline ~SMxx31() {
        if (pressure_mutex != NULL) {
            vSemaphoreDelete(pressure_mutex);
        }
    }

    QUICKI2C_DEFINE_REGISTER16_RO(RawTemperature, 0x2E) {};
    QUICKI2C_DEFINE_REGISTER16_RO(RawPressure, 0x30) {};
    QUICKI2C_DEFINE_REGISTER16_RO(Status, 0x32) {};

    /**
     * @brief Read pressure in Pascal (thread-safe with mutex).
     *
     * Returns tl::unexpected on mutex timeout or I2C failure.
     *
     * @return tl::expected<float, QuickI2CStatus>
     */
    tl::expected<float, QuickI2CStatus> readPressure() {
        // Take mutex with timeout to prevent indefinite blocking
        if (pressure_mutex == NULL || xSemaphoreTake(pressure_mutex, pdMS_TO_TICKS(1000)) != pdTRUE) {
            // If mutex is not available or taking it fails, return error
            return tl::make_unexpected(QuickI2CStatus::UnknownError);
        }
        
        // Mutex guard - automatically releases mutex when going out of scope
        struct MutexGuard {
            SemaphoreHandle_t& mutex;
            MutexGuard(SemaphoreHandle_t& m) : mutex(m) {}
            ~MutexGuard() { 
                if (mutex != NULL) {
                    xSemaphoreGive(mutex); 
                }
            }
        } guard(pressure_mutex);
        
        auto rawPressureUnsigned = readRawPressure();
        if (!rawPressureUnsigned) {
            return tl::make_unexpected(rawPressureUnsigned.error());
        }
        int16_t rawPressure = static_cast<int16_t>(*rawPressureUnsigned);

        // Print read buffer values, aligned to the same terminal column
        constexpr int16_t P_DOUTMIN = -26215; // From datasheet, 6. Operating Characteristics Table
        constexpr int16_t P_DOUTMAX = 26214; // From datasheet, 6. Operating Characteristics Table
        // From part numbering key, for SM4331-BCE-S-300-000
        constexpr float Pmin =  -300.0; // mBar
        constexpr float Pmax = +300.0; // mBar

        // Calculate pressure in Pa
        float pressure_mBar = (static_cast<float>(rawPressure) - P_DOUTMIN) / (P_DOUTMAX - P_DOUTMIN) * (Pmax - Pmin) + Pmin;

        // Convert mBar to Pa
        float pressure_Pa = pressure_mBar * 100.0f; // 1 mBar = 100 Pa

        return static_cast<float>(pressure_Pa);
    }

};
