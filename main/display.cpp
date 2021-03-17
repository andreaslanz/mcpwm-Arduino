//
// Created by andi on 17.03.2021.
//

#include <SPI.h>              // include libraries
#include <SSD1306.h>
#include "include/display.h"

SSD1306Wire display(OLED_SSD1306_DISPLAY_ADRESS,
                    OLED_SSD1306_DISPLAY_SDA,
                    OLED_SSD1306_DISPLAY_SCL,
                    GEOMETRY_128_64);



void display_setup(){

    // Vext power (für OLED) on
//    pinMode(OLED_SSD1306_DISPLAY_VEXT, OUTPUT);
//    digitalWrite(OLED_SSD1306_DISPLAY_VEXT, LOW);
    //reset oled
    pinMode(OLED_SSD1306_DISPLAY_RST, OUTPUT);
    digitalWrite(OLED_SSD1306_DISPLAY_RST, LOW);
    delay(50);
    digitalWrite(OLED_SSD1306_DISPLAY_RST, HIGH);
    display.init();
    display.flipScreenVertically();
    display.setFont(ArialMT_Plain_24);
    display.drawString(5, 20, "LoRa Initializing OK!");
    display.display();
}
