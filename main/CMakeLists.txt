idf_component_register(
    SRCS
        "src/app_main.c"
        "src/wifi_config.c"
        "src/nvs_store.c"
        "src/log_service.c"
        "src/config_store.c"
        "src/buzzer.c"
        "src/crsf_parser.c"
        "src/espnow_rx.c"
        "src/gps_tracker.c"
        "src/pointing_calc.c"
        "src/servo_driver.c"
        "src/antenna_control.c"
        "src/web_calib.c"
    INCLUDE_DIRS
        "include"
    REQUIRES
        esp_driver_gpio
        esp_driver_ledc
        esp_event
        esp_netif
        esp_wifi
        esp_http_server
        esp_timer
        nvs_flash
        json
    EMBED_FILES
        "web/calib.html"
)

# Auto-generate version header before every build
add_custom_target(version_bump ALL
    COMMAND ${PYTHON} ${CMAKE_SOURCE_DIR}/version_bump.py
    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
    COMMENT "Bumping firmware version"
)
add_dependencies(${COMPONENT_LIB} version_bump)
