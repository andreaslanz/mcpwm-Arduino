//
// Created by andi on 17.03.2021.
//

#ifndef MCPWM_ARDUINO_DISPLAY_H
#define MCPWM_ARDUINO_DISPLAY_H



#ifdef __cplusplus
extern "C" {
#endif
//#include "Arduino.h"
//#include <SSD1306.h>


#define OLED_SSD1306_DISPLAY_ADRESS 0x3c
#define OLED_SSD1306_DISPLAY_SDA 4
#define OLED_SSD1306_DISPLAY_SCL 15
#define OLED_SSD1306_DISPLAY_RST 16

//HELLTEC hat manchmal ein V ext Anschluss
#define OLED_SSD1306_DISPLAY_VEXT 21

void display_setup();

#ifdef __cplusplus
}
#endif

#endif //MCPWM_ARDUINO_DISPLAY_H
