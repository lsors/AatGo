idf_component_register(
    SRCS
        "src/app_main.c"
        "src/wifi_config.c"
        "src/nvs_store.c"
        "src/log_service.c"
    INCLUDE_DIRS
        "include"
    REQUIRES
        esp_driver_gpio
        esp_event
        esp_netif
        esp_wifi
        nvs_flash
)

# Auto-generate version header before every build
add_custom_target(version_bump ALL
    COMMAND ${PYTHON} ${CMAKE_SOURCE_DIR}/version_bump.py
    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
    COMMENT "Bumping firmware version"
)
add_dependencies(${COMPONENT_LIB} version_bump)
