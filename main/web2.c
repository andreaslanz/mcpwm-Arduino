//
// Created by andi on 31.03.2021.
//
/* Simple HTTP Server Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

#include <esp_wifi.h>
#include <esp_event.h>
//#define  LOG_LOCAL_LEVEL ESP_LOG_DEBUG
#define  LOG_LOCAL_LEVEL ESP_LOG_INFO

#include <esp_log.h>
#include <esp_system.h>
#include <nvs_flash.h>
#include <sys/param.h>
#include "nvs_flash.h"
//#include "tcpip_adapter.h"
#include "esp_netif.h"

#include "esp_eth.h"
#include "protocol_examples_common.h"
#include <esp_http_server.h>
#include "include/zeit.h"

/* A simple example that demonstrates how to create GET and POST
 * handlers for the web server.
 */

static const char *TAG = "example";

void setCrossOrigin(httpd_req_t *req){

    httpd_resp_set_hdr(req,"Access-Control-Allow-Origin","*");
    httpd_resp_set_hdr(req,"Access-Control-Max-Age","600");
    httpd_resp_set_hdr(req,"Access-Control-Allow-Methods","PUT,POST,GET,OPTIONS");
    httpd_resp_set_hdr(req,"Access-Control-Allow-Headers","*");
}



/*******************************
 *
 *     URL Parameter Handler
 *
 *******************************/
static void url_param_handler(httpd_req_t *req){

    char*  buf;
    size_t buf_len;

    buf_len = httpd_req_get_url_query_len(req) + 1;
    if (buf_len > 1) {
        buf = malloc(buf_len);
        if (httpd_req_get_url_query_str(req, buf, buf_len) == ESP_OK) {
            ESP_LOGI(TAG, "Found URL query => %s", buf);
            char param[32];
            /** action */
            if (httpd_query_key_value(buf, "action", param, sizeof(param)) == ESP_OK) {
                ESP_LOGI(TAG, "Found URL query parameter => query1=%s", param);
                /** start*/
                if(!strcmp(param,"start")){
                    ESP_LOGI(TAG, "Rennen gestartet");
                    action (drag_start); // Rennen starten
                }
                /** neu*/
                if(!strcmp(param,"neu")){
                    ESP_LOGI(TAG, "neues Rennen");
                    action(neu);
                }
                /** L1*/
                if(!strcmp(param,"L1")){
                    ESP_LOGI(TAG, "L1");
                    L1(); // neues Rennen
                }
                /** L2*/
                if(!strcmp(param,"L2")){
                    ESP_LOGI(TAG, "L2");
                    L2(); // neues Rennen
                }
                /** L3*/
                if(!strcmp(param,"L3")){
                    ESP_LOGI(TAG, "L3");
                    L3(); // neues Rennen
                }

            }
            /*!!
             * WARTEN bis Antwort fertig berechnet
             * */
            vTaskDelay(10);

        }
        free(buf);
    }
}

/*********************************
 *   URL:    /zahl
 *
 *   (Daten Senden)
 **********************************/
static esp_err_t zahl_handler(httpd_req_t *req)
{
    setCrossOrigin(req);

    /**URL-Parameter*/
    url_param_handler(req);


    httpd_resp_set_type(req,"application/json");

    if( xSemaphoreTake( xSemaphore, ( TickType_t ) 10 ) == pdTRUE )
    {
        /* We were able to obtain the semaphore and can now access the
        shared resource. */

        httpd_resp_send(req, dragrace.dragrace_Json_String, strlen(dragrace.dragrace_Json_String));


        /* We have finished accessing the shared resource.  Release the
        semaphore. */
        xSemaphoreGive( xSemaphore );
    }


    return ESP_OK;
}
static const httpd_uri_t zahl = {
        .uri       = "/zahl",
        .method    = HTTP_GET,
        .handler   = zahl_handler,
        /* Let's pass response string in user
         * context to demonstrate it's usage */
        .user_ctx  = NULL
};




static httpd_handle_t start_webserver(void)
{
    httpd_handle_t server = NULL;
//    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    httpd_config_t config =  {.task_priority=((UBaseType_t) 0U) + 2,
                              .stack_size=4096, .core_id=0x7fffffff,
                              .server_port=80, .ctrl_port=32768,
                              .max_open_sockets=7, .max_uri_handlers=8,
                              .max_resp_headers=8, .backlog_conn=5,
                              .lru_purge_enable=0, .recv_wait_timeout=5,
                              .send_wait_timeout=5, .global_user_ctx=((void *) 0),
                              .global_user_ctx_free_fn=((void *) 0),
                              .global_transport_ctx=((void *) 0),
                              .global_transport_ctx_free_fn=((void *) 0),
                              .open_fn=((void *) 0),
                              .close_fn=((void *) 0),
                              .uri_match_fn=((void *) 0)
    };

    // Start the httpd server
    ESP_LOGI(TAG, "Starting server on port: '%d'", config.server_port);
    if (httpd_start(&server, &config) == ESP_OK) {
        // Set URI handlers
        ESP_LOGI(TAG, "Registering URI handlers");
        httpd_register_uri_handler(server, &zahl);
        return server;
    }

    ESP_LOGI(TAG, "Error starting server!");
    return NULL;
}

static void stop_webserver(httpd_handle_t server)
{
    // Stop the httpd server
    httpd_stop(server);
}

static void disconnect_handler(void* arg, esp_event_base_t event_base,
                               int32_t event_id, void* event_data)
{
    httpd_handle_t* server = (httpd_handle_t*) arg;
    if (*server) {
        ESP_LOGI(TAG, "Stopping webserver");
        stop_webserver(*server);
        *server = NULL;
    }
}

static void connect_handler(void* arg, esp_event_base_t event_base,
                            int32_t event_id, void* event_data)
{
    httpd_handle_t* server = (httpd_handle_t*) arg;
    if (*server == NULL) {
        ESP_LOGI(TAG, "Starting webserver");
        *server = start_webserver();
    }
}

void dragrace_webserver()
{
    static httpd_handle_t server = NULL;

    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
//    tcpip_adapter_init();
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    /* This helper function configures Wi-Fi or Ethernet, as selected in menuconfig.
     * Read "Establishing Wi-Fi or Ethernet Connection" section in
     * examples/protocols/README.md for more information about this function.
     */
    ESP_ERROR_CHECK(example_connect());

    /* Register event handlers to stop the server when Wi-Fi or Ethernet is disconnected,
     * and re-start it upon connection.
     */
#ifdef CONFIG_EXAMPLE_CONNECT_WIFI
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &connect_handler, &server));
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, WIFI_EVENT_STA_DISCONNECTED, &disconnect_handler, &server));
#endif // CONFIG_EXAMPLE_CONNECT_WIFI
#ifdef CONFIG_EXAMPLE_CONNECT_ETHERNET
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_ETH_GOT_IP, &connect_handler, &server));
    ESP_ERROR_CHECK(esp_event_handler_register(ETH_EVENT, ETHERNET_EVENT_DISCONNECTED, &disconnect_handler, &server));
#endif // CONFIG_EXAMPLE_CONNECT_ETHERNET

    /* Start the server for the first time */
    server = start_webserver();
}

