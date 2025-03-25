idf_component_register(SRCS "kode_bq25896.c"
                    INCLUDE_DIRS "include"
                    REQUIRES driver esp_driver_i2c freertos esp_common)
