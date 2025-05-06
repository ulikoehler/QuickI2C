# Include from git submodule
idf_component_register(SRCS "QuickI2C/src/QuickI2C.cpp"
                    INCLUDE_DIRS "QuickI2C/include"
                    REQUIRES driver)
