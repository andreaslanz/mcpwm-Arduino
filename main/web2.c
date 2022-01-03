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
                    drag_start(); // Rennen starten
                }
                /** neu*/
                if(!strcmp(param,"neu")){
                    ESP_LOGI(TAG, "neues Rennen");
                    neu(); // neues Rennen
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
    httpd_resp_send(req, dragrace.dragrace_Json_String, strlen(dragrace.dragrace_Json_String));

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

/*********************************
 *   URL:    /start
 *
 *   (Rennen starten)
 **********************************/
static esp_err_t start_handler(httpd_req_t *req){


    setCrossOrigin(req);

    /**URL-Parameter*/
    url_param_handler(req);


    httpd_resp_set_type(req,"application/json");
    httpd_resp_send(req, dragrace.dragrace_Json_String, strlen(dragrace.dragrace_Json_String));

    return ESP_OK;
}
/** An HTTP GET handler */
static esp_err_t hello_get_handler(httpd_req_t *req)
{
    char*  buf;
    size_t buf_len;

    /* Get header value string length and allocate memory for length + 1,
     * extra byte for null termination */
    buf_len = httpd_req_get_hdr_value_len(req, "Host") + 1;
    if (buf_len > 1) {
        buf = malloc(buf_len);
        /* Copy null terminated value string into buffer */
        if (httpd_req_get_hdr_value_str(req, "Host", buf, buf_len) == ESP_OK) {
            ESP_LOGI(TAG, "Found header => Host: %s", buf);
        }
        free(buf);
    }

    buf_len = httpd_req_get_hdr_value_len(req, "Test-Header-2") + 1;
    if (buf_len > 1) {
        buf = malloc(buf_len);
        if (httpd_req_get_hdr_value_str(req, "Test-Header-2", buf, buf_len) == ESP_OK) {
            ESP_LOGI(TAG, "Found header => Test-Header-2: %s", buf);
        }
        free(buf);
    }

    buf_len = httpd_req_get_hdr_value_len(req, "Test-Header-1") + 1;
    if (buf_len > 1) {
        buf = malloc(buf_len);
        if (httpd_req_get_hdr_value_str(req, "Test-Header-1", buf, buf_len) == ESP_OK) {
            ESP_LOGI(TAG, "Found header => Test-Header-1: %s", buf);
        }
        free(buf);
    }

    /* Read URL query string length and allocate memory for length + 1,
     * extra byte for null termination */
    buf_len = httpd_req_get_url_query_len(req) + 1;
    if (buf_len > 1) {
        buf = malloc(buf_len);
        if (httpd_req_get_url_query_str(req, buf, buf_len) == ESP_OK) {
            ESP_LOGI(TAG, "Found URL query => %s", buf);
            char param[32];
            /* Get value of expected key from query string */
            if (httpd_query_key_value(buf, "query1", param, sizeof(param)) == ESP_OK) {
                ESP_LOGI(TAG, "Found URL query parameter => query1=%s", param);
            }
            if (httpd_query_key_value(buf, "query3", param, sizeof(param)) == ESP_OK) {
                ESP_LOGI(TAG, "Found URL query parameter => query3=%s", param);
            }
            if (httpd_query_key_value(buf, "query2", param, sizeof(param)) == ESP_OK) {
                ESP_LOGI(TAG, "Found URL query parameter => query2=%s", param);
            }
        }
        free(buf);
    }

    /* Set some custom headers */
    httpd_resp_set_hdr(req, "Custom-Header-1", "Custom-Value-1");
    httpd_resp_set_hdr(req, "Custom-Header-2", "Custom-Value-2");
    //a.l.
    setCrossOrigin(req);

    /* Send response with custom headers and body set as the
     * string passed in user context*/
    const char* resp_str = (const char*) req->user_ctx;
    //a.l.
    httpd_resp_set_type(req,"type");
    httpd_resp_send(req, resp_str, strlen(resp_str));

    /* After sending the HTTP response the old HTTP request
     * headers are lost. Check if HTTP request headers can be read now. */
    if (httpd_req_get_hdr_value_len(req, "Host") == 0) {
        ESP_LOGI(TAG, "Request headers lost");
    }
    return ESP_OK;
}

static const httpd_uri_t hello = {
        .uri       = "/hello",
        .method    = HTTP_GET,
        .handler   = hello_get_handler,
        /* Let's pass response string in user
         * context to demonstrate it's usage */
        .user_ctx  = "Hello World!"
};

/** An HTTP POST handler */
static esp_err_t echo_post_handler(httpd_req_t *req)
{
    char buf[100];
    int ret, remaining = req->content_len;

    while (remaining > 0) {
        /* Read the data for the request */
        if ((ret = httpd_req_recv(req, buf,
                                  MIN(remaining, sizeof(buf)))) <= 0) {
            if (ret == HTTPD_SOCK_ERR_TIMEOUT) {
                /* Retry receiving if timeout occurred */
                continue;
            }
            return ESP_FAIL;
        }

        /* Send back the same data */
        httpd_resp_send_chunk(req, buf, ret);
        remaining -= ret;

        /* Log data received */
        ESP_LOGI(TAG, "=========== RECEIVED DATA ==========");
        ESP_LOGI(TAG, "%.*s", ret, buf);
        ESP_LOGI(TAG, "====================================");
    }

    // End response
    httpd_resp_send_chunk(req, NULL, 0);
    return ESP_OK;
}

static const httpd_uri_t echo = {
        .uri       = "/echo",
        .method    = HTTP_POST,
        .handler   = echo_post_handler,
        .user_ctx  = NULL
};

/* This handler allows the custom error handling functionality to be
 * tested from client side. For that, when a PUT request 0 is sent to
 * URI /ctrl, the /hello and /echo URIs are unregistered and following
 * custom error handler http_404_error_handler() is registered.
 * Afterwards, when /hello or /echo is requested, this custom error
 * handler is invoked which, after sending an error message to client,
 * either closes the underlying socket (when requested URI is /echo)
 * or keeps it open (when requested URI is /hello). This allows the
 * client to infer if the custom error handler is functioning as expected
 * by observing the socket state.
 */
esp_err_t http_404_error_handler(httpd_req_t *req, httpd_err_code_t err)
{
    if (strcmp("/hello", req->uri) == 0) {
        httpd_resp_send_err(req, HTTPD_404_NOT_FOUND, "/hello URI is not available");
        /* Return ESP_OK to keep underlying socket open */
        return ESP_OK;
    } else if (strcmp("/echo", req->uri) == 0) {
        httpd_resp_send_err(req, HTTPD_404_NOT_FOUND, "/echo URI is not available");
        /* Return ESP_FAIL to close underlying socket */
        return ESP_FAIL;
    }
    /* For any other URI send 404 and close socket */
    httpd_resp_send_err(req, HTTPD_404_NOT_FOUND, "Some 404 error message");
    return ESP_FAIL;
}

/* An HTTP PUT handler. This demonstrates realtime
 * registration and deregistration of URI handlers
 */


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
        httpd_register_uri_handler(server, &hello);
        httpd_register_uri_handler(server, &echo);
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

