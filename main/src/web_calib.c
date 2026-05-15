#include "web_calib.h"
#include "servo_driver.h"
#include "antenna_control.h"
#include "gps_tracker.h"
#include "config_store.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include "cJSON.h"
#include <string.h>
#include <stdlib.h>

static const char *TAG = "web_calib";

/* Embedded HTML from main/web/calib.html via EMBED_FILES in CMakeLists */
extern const uint8_t calib_html_start[] asm("_binary_calib_html_start");
extern const uint8_t calib_html_end[]   asm("_binary_calib_html_end");

static httpd_handle_t s_server    = NULL;
static servo_calib_t  s_calib     = {0};

/* ---- Helpers ---- */

static cJSON *make_status_json(void)
{
    cJSON *root = cJSON_CreateObject();
    cJSON_AddNumberToObject(root, "az_raw",      servo_get_raw(SERVO_AZ));
    cJSON_AddNumberToObject(root, "el_raw",      servo_get_raw(SERVO_EL));
    cJSON_AddNumberToObject(root, "az_offset",   s_calib.az_offset_deg);
    cJSON_AddNumberToObject(root, "el_offset",   s_calib.el_offset_deg);
    cJSON_AddBoolToObject  (root, "az_reversed", s_calib.az_reversed);
    cJSON_AddBoolToObject  (root, "el_reversed", s_calib.el_reversed);
    cJSON_AddBoolToObject  (root, "paused",      antenna_control_is_paused());
    cJSON_AddBoolToObject  (root, "home_ready",  gps_tracker_home_ready());
    return root;
}

static esp_err_t send_json(httpd_req_t *req, cJSON *root)
{
    char *body = cJSON_PrintUnformatted(root);
    cJSON_Delete(root);
    if (!body) return ESP_ERR_NO_MEM;
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    esp_err_t err = httpd_resp_sendstr(req, body);
    free(body);
    return err;
}

static esp_err_t read_body(httpd_req_t *req, char *buf, size_t bufsz)
{
    if (req->content_len == 0 || req->content_len >= bufsz) return ESP_FAIL;
    int received = httpd_req_recv(req, buf, req->content_len);
    if (received <= 0) return ESP_FAIL;
    buf[received] = '\0';
    return ESP_OK;
}

/* ---- URI Handlers ---- */

static esp_err_t handler_root(httpd_req_t *req)
{
    size_t html_len = calib_html_end - calib_html_start;
    httpd_resp_set_type(req, "text/html");
    httpd_resp_send(req, (const char *)calib_html_start, (ssize_t)html_len);
    return ESP_OK;
}

static esp_err_t handler_status(httpd_req_t *req)
{
    return send_json(req, make_status_json());
}

static esp_err_t handler_servo(httpd_req_t *req)
{
    char buf[128];
    if (read_body(req, buf, sizeof(buf)) != ESP_OK) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "bad body");
        return ESP_FAIL;
    }
    cJSON *root = cJSON_Parse(buf);
    if (!root) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "invalid JSON");
        return ESP_FAIL;
    }
    cJSON *az_j = cJSON_GetObjectItem(root, "az");
    cJSON *el_j = cJSON_GetObjectItem(root, "el");
    if (cJSON_IsNumber(az_j)) servo_set_raw(SERVO_AZ, (float)az_j->valuedouble);
    if (cJSON_IsNumber(el_j)) servo_set_raw(SERVO_EL, (float)el_j->valuedouble);
    cJSON_Delete(root);
    return send_json(req, make_status_json());
}

static esp_err_t handler_calib_save(httpd_req_t *req)
{
    char buf[256];
    if (read_body(req, buf, sizeof(buf)) != ESP_OK) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "bad body");
        return ESP_FAIL;
    }
    cJSON *root = cJSON_Parse(buf);
    if (!root) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "invalid JSON");
        return ESP_FAIL;
    }
    cJSON *j;
    if ((j = cJSON_GetObjectItem(root, "az_offset"))   && cJSON_IsNumber(j))
        s_calib.az_offset_deg = (float)j->valuedouble;
    if ((j = cJSON_GetObjectItem(root, "el_offset"))   && cJSON_IsNumber(j))
        s_calib.el_offset_deg = (float)j->valuedouble;
    if ((j = cJSON_GetObjectItem(root, "az_reversed")) && cJSON_IsBool(j))
        s_calib.az_reversed = cJSON_IsTrue(j);
    if ((j = cJSON_GetObjectItem(root, "el_reversed")) && cJSON_IsBool(j))
        s_calib.el_reversed = cJSON_IsTrue(j);
    cJSON_Delete(root);

    servo_calib_update(&s_calib);
    config_save(&s_calib);
    ESP_LOGI(TAG, "Calibration saved");
    return send_json(req, make_status_json());
}

static esp_err_t handler_calib_reset(httpd_req_t *req)
{
    config_reset(&s_calib);
    servo_calib_update(&s_calib);
    config_save(&s_calib);
    ESP_LOGI(TAG, "Calibration reset");
    return send_json(req, make_status_json());
}

static esp_err_t handler_track_pause(httpd_req_t *req)
{
    char buf[64];
    if (read_body(req, buf, sizeof(buf)) == ESP_OK) {
        cJSON *root = cJSON_Parse(buf);
        if (root) {
            cJSON *j = cJSON_GetObjectItem(root, "pause");
            if (cJSON_IsBool(j)) antenna_control_pause(cJSON_IsTrue(j));
            cJSON_Delete(root);
        }
    }
    return send_json(req, make_status_json());
}

/* ---- Init ---- */

esp_err_t web_calib_init(void)
{
    /* Load saved calibration and apply immediately */
    config_load(&s_calib);
    servo_calib_update(&s_calib);

    httpd_config_t cfg = HTTPD_DEFAULT_CONFIG();
    cfg.max_uri_handlers = 8;

    esp_err_t err = httpd_start(&s_server, &cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "httpd_start failed: %s", esp_err_to_name(err));
        return err;
    }

    static const httpd_uri_t uris[] = {
        {.uri = "/",                .method = HTTP_GET,  .handler = handler_root},
        {.uri = "/api/status",      .method = HTTP_GET,  .handler = handler_status},
        {.uri = "/api/servo",       .method = HTTP_POST, .handler = handler_servo},
        {.uri = "/api/calib/save",  .method = HTTP_POST, .handler = handler_calib_save},
        {.uri = "/api/calib/reset", .method = HTTP_POST, .handler = handler_calib_reset},
        {.uri = "/api/track/pause", .method = HTTP_POST, .handler = handler_track_pause},
    };
    for (size_t i = 0; i < sizeof(uris) / sizeof(uris[0]); i++) {
        httpd_register_uri_handler(s_server, &uris[i]);
    }

    ESP_LOGI(TAG, "Calibration server at http://192.168.4.1/");
    return ESP_OK;
}
