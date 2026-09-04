idf_component_register(SRCS "hello_world_main.c" "gatt_svr.c" "servo.c"
                       PRIV_REQUIRES bt nvs_flash esp_driver_mcpwm esp_hw_support esp_timer
                       INCLUDE_DIRS ".")
