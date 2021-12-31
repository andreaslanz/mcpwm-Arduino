idf_component_register(
        #SRC_DIRS

        SRCS
            main.cpp
#            web.cpp
            zeit.c
            serial.c
#            display.cpp
            utility.cpp
            web2.c

            #                "C:/ESP/LIB/ThingPulse/esp8266-oled-ssd1306/src/OLEDDisplay.cpp"
            #                "C:/ESP/LIB/sandeepmistry/arduino-LoRa/src/LoRa.cpp"
            #                "C:/ESP/LIB/sandeepmistry/arduino-LoRa/examples/LoRaDuplex/LoRaDuplex.cpp"
            #                "C:/ESP/LIB/ThingPulse/esp8266-oled-ssd1306/src/OLEDDisplayUi.cpp"
            #                "C:/ESP/LIB/ERROPiX/ESP32_AnalogWrite/src/analogWrite.cpp"

            #                "C:/ESP/heltec/Heltec_ESP32/src/heltec.cpp"
            #                "C:/ESP/heltec/Heltec_ESP32/src/BMP180.cpp"
            #                "C:/ESP/heltec/Heltec_ESP32/src/lora/LoRa.cpp"
            #                "C:/ESP/heltec/Heltec_ESP32/src/oled/OLEDDisplay.cpp"
            #                "C:/ESP/heltec/Heltec_ESP32/src/oled/OLEDDisplayUi.cpp"
            #                "C:/Users/andi/AppData/Local/Arduino15/packages/esp32/hardware/esp32/1.0.4/boards.txt"

        INCLUDE_DIRS
        "."
            #                "C:/ESP/arduino-esp32/variants/heltec_wifi_lora_32_V2"
            #                "C:/ESP/LIB/ArduinoJson/ArduinoJson/src"
            #                "C:/ESP/LIB/ThingPulse/esp8266-oled-ssd1306/src"
            #                "C:/ESP/esp-idf/examples/common_components/protocol_examples_common/include"   #webserver


            #                "C:/ESP/LIB/sandeepmistry/arduino-LoRa/src"
            #                "C:/ESP/LIB/ERROPiX/ESP32_AnalogWrite/src"
            #                "C:/ESP/heltec/WiFi_Kit_series/esp32/variants/wifi_lora_32_V2"
            #
            #                "C:/ESP/heltec/Heltec_ESP32/src"
            #                "C:/ESP/heltec/Heltec_ESP32/src/lora"
            #                "C:/ESP/heltec/Heltec_ESP32/src/oled"
            #                "C:/ESP/heltec/Heltec_ESP32/examples/Factory_Test/WiFi_LoRa_32FactoryTest"

        PRIV_REQUIRES arduino-esp32
        REQUIRES nvs_flash esp_http_server esp_http_client protocol_examples_common json #webserver
)


target_compile_options(${COMPONENT_LIB} PRIVATE -Wno-unused-variable -Wno-missing-field-initializers -Wno-unused-function -Wno-unused-but-set-variable -DESP32
        -DARDUINO # wichtig für OLED Lib.
        -DARDUINO_ARCH_ESP32 # wichtig für OLED Lib.
        -DARDUINO=10812
        )

