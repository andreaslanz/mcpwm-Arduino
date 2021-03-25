#ifndef advancedwebserver
#define advancedwebserver
#ifdef __cplusplus
extern "C" {
#endif


//#include <WebServer.h>


//extern WebServer server;

void Webserver_Setup(void);
void Webserver_loop(void);
static void drawGraph();

#ifdef __cplusplus
}
#endif


#endif