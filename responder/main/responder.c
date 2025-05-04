#include <stdio.h>
#include <string.h>
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/task.h"

#define TAG_AP "ftm_responder"
#define MAX_SSID_LEN 32
#define MAX_PASSPHRASE_LEN 64
#define DEFAULT_AP_SSID "FTM"
#define DEFAULT_AP_PASSWORD "ftmftmftm"
#define DEFAULT_AP_CHANNEL 1
#define DEFAULT_AP_BANDWIDTH 20

wifi_config_t g_ap_config = {
        .ap.max_connection = 4,
        .ap.authmode = WIFI_AUTH_WPA2_PSK,
        .ap.ftm_responder = true
};

static bool s_reconnect = true;
static bool s_ap_started = false;
static EventGroupHandle_t s_ftm_event_group;
static EventGroupHandle_t s_wifi_event_group;
static const char *TAG = "responder";

static void event_handler(void *arg, esp_event_base_t event_base,
                          int32_t event_id, void *event_data)
{
    if (event_id == WIFI_EVENT_AP_START) {
        s_ap_started = true;
    } else if (event_id == WIFI_EVENT_AP_STOP) {
        s_ap_started = false;
    }
}

static void initialise_wifi(void)
{
    esp_log_level_set("wifi", ESP_LOG_WARN);
    static bool initialized = false;

    if (initialized) {
        return;
    }

    ESP_ERROR_CHECK(esp_netif_init());
    s_wifi_event_group = xEventGroupCreate();
    s_ftm_event_group = xEventGroupCreate();
    ESP_ERROR_CHECK( esp_event_loop_create_default() );
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    esp_event_handler_instance_t instance_any_id;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &event_handler,
                                                        NULL,
                                                        &instance_any_id));

    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM) );
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_NULL) );
    ESP_ERROR_CHECK(esp_wifi_start() );
    initialized = true;
}

esp_err_t wifi_add_mode(wifi_mode_t mode)
{
    wifi_mode_t cur_mode, new_mode = mode;
    esp_err_t err = esp_wifi_get_mode(&cur_mode);

    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Failed to get current WiFi mode, proceeding with default");
    }

    if (mode == WIFI_MODE_AP) {
        if (cur_mode == WIFI_MODE_AP || cur_mode == WIFI_MODE_APSTA) {
            /* Do nothing */
            return ESP_OK;
        } else if (cur_mode == WIFI_MODE_STA) {
            new_mode = WIFI_MODE_APSTA;
        } else {
            new_mode = WIFI_MODE_AP;
        }
    }

    ESP_ERROR_CHECK( esp_wifi_set_mode(new_mode) );
    return ESP_OK;
}

static bool wifi_cmd_ap_set(const char* ssid, const char* pass, uint8_t channel, uint8_t bw)
{
    s_reconnect = false;
    strlcpy((char*) g_ap_config.ap.ssid, ssid, MAX_SSID_LEN);
    if (pass) {
        if (strlen(pass) != 0 && strlen(pass) < 8) {
            s_reconnect = true;
            ESP_LOGE(TAG_AP, "password cannot be less than 8 characters long");
            return false;
        }
        strlcpy((char*) g_ap_config.ap.password, pass, MAX_PASSPHRASE_LEN);
    }
    if (!(channel >=1 && channel <= 14)) {
        ESP_LOGE(TAG_AP, "Channel cannot be %d!", channel);
        return false;
    }
    if (bw != 20 && bw != 40) {
        ESP_LOGE(TAG_AP, "Cannot set %d MHz bandwidth!", bw);
        return false;
    }

    if (ESP_OK != wifi_add_mode(WIFI_MODE_AP)) {
        return false;
    }
    if (strlen(pass) == 0) {
        g_ap_config.ap.authmode = WIFI_AUTH_OPEN;
    }
    g_ap_config.ap.channel = channel;
    ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_AP, &g_ap_config));
    if (bw == 40) {
        esp_wifi_set_bandwidth(ESP_IF_WIFI_AP, WIFI_BW_HT40);
    } else {
        esp_wifi_set_bandwidth(ESP_IF_WIFI_AP, WIFI_BW_HT20);
    }
    ESP_LOGI(TAG_AP, "Starting SoftAP with FTM Responder support, SSID - %s, Password - %s, Primary Channel - %d, Bandwidth - %dMHz",
             ssid, pass, channel, bw);

    return true;
}

void my_loop_task(void *arg)
{
    int counter = 0;
    while (true) {
        counter++;
        ESP_LOGI(TAG, "Counter = %d", counter);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void app_main(void)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    initialise_wifi();

    if (!wifi_cmd_ap_set(DEFAULT_AP_SSID, DEFAULT_AP_PASSWORD, DEFAULT_AP_CHANNEL, DEFAULT_AP_BANDWIDTH)) {
        ESP_LOGE(TAG_AP, "Failed to start SoftAP!");
        return;
    }

    ESP_LOGI(TAG_AP, "FTM Responder is ready, waiting for FTM requests...");

    xTaskCreate(my_loop_task, "loop_task", 2048, NULL, 5, NULL);
}
