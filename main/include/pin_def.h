//
// Created by andi on 29.03.2021.
//

#ifndef MCPWM_ARDUINO_PIN_DEF_H
#define MCPWM_ARDUINO_PIN_DEF_H

#include <esp_bit_defs.h>

/// Gebrauchte Anschlüsse
/////////////////////////

///EINGAENGE

//Lichtschranken Eingänge
#define DRAGRACE_PIN_LICHTSCHRANKE_L1_INPUT  23
#define DRAGRACE_PIN_LICHTSCHRANKE_L2_INPUT  19
#define DRAGRACE_PIN_LICHTSCHRANKE_L3_INPUT  22
#define DRAGRACE_PIN_LICHTSCHRANKE_R1_INPUT  32
#define DRAGRACE_PIN_LICHTSCHRANKE_R2_INPUT  33
#define DRAGRACE_PIN_LICHTSCHRANKE_R3_INPUT  25

//Neues Rennen (reset)
#define DRAGRACE_PIN_NEU_INPUT               34

//Starte Rennen
#define DRAGRACE_PIN_START_INPUT             35

//Positions Lichtschranke Eingänge
#define DRAGRACE_PIN_POSITION_LINKS_INPUT    36   //Position L Eing.
#define DRAGRACE_PIN_POSITION_RECHTS_INPUT   39   //Position R Eing.


///AUSGAENGE

//Fehlstart Lampe Ausgänge
#define DRAGRACE_PIN_FEHLSTART_LINKS_OUTPUT  26 //Fehlstart L Lampe
#define DRAGRACE_PIN_FEHLSTART_RECHTS_OUTPUT 27 //Fehlstart L Lampe

//Positions Warn-Lampe Ausgang
#define DRAGRACE_PIN_POSITION_LAMPE_L_OUTPUT 13
#define DRAGRACE_PIN_POSITION_LAMPE_R_OUTPUT 12

//Test-Ausgang (für Hardware Start mit L3/R3 verbinden
#define DRAGRACE_PIN_TEST_L1_OUTPUT          18

//Sieg
#define DRAGRACE_PIN_SIEG_LAMPE_R_OUTPUT     14
#define DRAGRACE_PIN_SIEG_LAMPE_L_OUTPUT     21

//START
#define DRAGRACE_PIN_ORANG1_LAMPE_OUTPUT     5
#define DRAGRACE_PIN_ORANG2_LAMPE_OUTPUT     4
#define DRAGRACE_PIN_GRUEN_LAMPE_OUTPUT      2


#endif //MCPWM_ARDUINO_PIN_DEF_H
