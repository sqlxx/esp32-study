idf_component_register(SRCS "hello_world_main.c" "gatt_svr.c"
                       PRIV_REQUIRES bt nvs_flash esp_hw_support esp_timer freertos servo bmi088
                       INCLUDE_DIRS ".")
