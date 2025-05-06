# Include from git submodule
idf_component_register(SRCS "src/QuickI2C.cpp"
                    INCLUDE_DIRS "include"
                    REQUIRES driver)
