# Include from git submodule
set(quicki2c_requires driver)

if(DEFINED ENV{IDF_PATH} AND EXISTS "$ENV{IDF_PATH}/components/esp_driver_i2c/CMakeLists.txt")
    list(APPEND quicki2c_requires esp_driver_i2c)
endif()

idf_component_register(SRCS "src/QuickI2C.cpp"
                    INCLUDE_DIRS "include"
                    REQUIRES ${quicki2c_requires})
