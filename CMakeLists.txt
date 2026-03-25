# Include from git submodule
idf_component_register(SRCS "src/QuickI2C.cpp"
                    INCLUDE_DIRS "include" "examples"
                    REQUIRES driver esp_driver_i2c)
