#ifndef advancedwebserver
#define advancedwebserver


#include <WebServer.h>


extern WebServer server;

void Webserver_Setup(void);
void Webserver_loop(void);
static void drawGraph();


#endif