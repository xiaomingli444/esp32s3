set(src_dirs
            SYS
            KEY
            LCD
            LED
            RGB
            CAM
            USART
            WIFI
            SD
            LVGL
            )

set(include_dirs
            SYS
            KEY
            LCD
            LED
            RGB
            CAM
            USART
            WIFI
            SD
            LVGL
            ${CMAKE_SOURCE_DIR}/main/include
            )

set(requires
            esp32-camera  
            apriltag-master
            esp-ui
            esp_wifi
            spi_flash
            esp_event
            esp_http_server
            esp_http_client
            esp_netif
            esp_timer
            esp_lcd
            app_update
            json
            mbedtls
            nvs_flash
            mqtt
            vfs
            freertos
            fatfs
            spiffs
            driver)

idf_component_register(SRC_DIRS ${src_dirs} INCLUDE_DIRS ${include_dirs} REQUIRES ${requires})

component_compile_options(-ffast-math -O3 -Wno-error=format=-Wno-format)



