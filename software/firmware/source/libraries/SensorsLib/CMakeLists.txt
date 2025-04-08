set(src_dirs ./src
             ./src/touch
             ./src/platform
             ./src/bosch
             ./src/bosch/BMM150
             ./src/bosch/common
             )
set(include_dirs ./src
                ./src/REG
                ./src/touch
                ./src/platform
                ./src/bosch
                ./src/bosch/firmware
                ./src/bosch/BMM150
                ./src/bosch/common)

idf_component_register(SRC_DIRS ${src_dirs} 
                        INCLUDE_DIRS ${include_dirs}
                        REQUIRES esp_timer esp_driver_gpio driver)
