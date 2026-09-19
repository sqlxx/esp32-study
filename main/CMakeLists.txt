idf_component_register(SRCS "main.c" "gatt_svr.c"
                       PRIV_REQUIRES bt nvs_flash esp_hw_support esp_timer freertos servo bmi088 bldc as5600
                       INCLUDE_DIRS ".")
