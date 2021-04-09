#ifndef advancedwebserver
#define advancedwebserver
#ifdef __cplusplus
extern "C" {
#endif


//Webserver1 (Arduino-Lib)
void Webserver_Setup(void);
void Webserver_loop(void);
static void drawGraph();


//Webserver2 (idf)
void dragrace_webserver(); //start





#ifdef __cplusplus
}
#endif


#endif