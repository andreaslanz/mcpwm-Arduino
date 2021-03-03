# The following lines of boilerplate have to be in your project's
# CMakeLists in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.5)
add_compile_definitions(
        #ESP32
        #WIFI_LoRa_32_V2
        #ARDUINO_WIFI_LORA_32_V2
        #F_CPU=240000000L
        #ARDUINO=10813
        #CORE_DEBUG_LEVEL=0
        #ARDUINO_LIB_DISCOVERY_PHASE
        #ARDUINO_VARIANT=\"wifi_lora_32_V2\"

        #-Wno-missing-field-initializers

        #oled
        #ESP_PLATFORM
        #MBEDTLS_CONFIG_FILE=\"mbedtls/esp_config.h\"
        #REGION_EU868
        #LoRaWAN_DEBUG_LEVEL=0

        #ACTIVE_REGION=LORAMAC_REGION_EU868
        #HAVE_CONFIG_H
        #WITH_POSIX
        #CONFIG_AUTOSTART_ARDUINO
        )
list(APPEND EXTRA_COMPONENT_DIRS "C:/ESP/arduino-esp32")
#set(COMPONENTS "arduino-esp32")

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(mcpwm-arduino)
