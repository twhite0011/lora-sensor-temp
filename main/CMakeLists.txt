idf_component_register(
    SRCS "app_main.c" "shtc3.c"
    INCLUDE_DIRS "."
    REQUIRES driver esp_timer esp_adc lmic
)
