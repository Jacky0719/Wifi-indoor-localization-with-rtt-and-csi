#include <stdio.h>
#include <string.h>
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <inttypes.h>
#include "esp_mac.h"
#include "esp_now.h"

#define TAG_STA "ftm_initiator"
#define MAX_SSID_LEN 32
#define MAX_PASSPHRASE_LEN 64
#define DEFAULT_AP_SSID "FTM"
#define DEFAULT_AP_PASSWORD "ftmftmftm"
#define MIN_WAIT_TIME_MS 100
#define MAX_FTM_BURSTS 16
#define ETH_ALEN 6
#define MAX_CONNECT_RETRY_ATTEMPTS 5

static uint8_t s_ap_channel;
static uint8_t s_ap_bssid[ETH_ALEN];

static EventGroupHandle_t s_wifi_event_group;
static const int CONNECTED_BIT = BIT0;
static const int DISCONNECTED_BIT = BIT1;

static EventGroupHandle_t s_ftm_event_group;
static const int FTM_REPORT_BIT = BIT0;
static const int FTM_FAILURE_BIT = BIT1;

static uint32_t s_rtt_est, s_dist_est;
static bool s_reconnect = true;
static int s_retry_num = 0;
static uint8_t s_ftm_report_num_entries;

static const uint8_t CONFIG_CSI_SEND_MAC[] = {0x1a, 0x00, 0x00, 0x00, 0x00, 0x00};

static esp_now_peer_info_t csi_peer = {
        .channel   = 1,
        .ifidx     = WIFI_IF_STA,
        .encrypt   = false,
        .peer_addr = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff},
};

static void event_handler(void *arg, esp_event_base_t event_base,
                          int32_t event_id, void *event_data)
{
    if (event_id == WIFI_EVENT_STA_CONNECTED) {
        wifi_event_sta_connected_t *event = (wifi_event_sta_connected_t *)event_data;

        ESP_LOGI(TAG_STA, "Connected to %s (BSSID: " MACSTR ", Channel: %d)",
                event->ssid, MAC2STR(event->bssid), event->channel);

        memcpy(s_ap_bssid, event->bssid, ETH_ALEN);
        s_ap_channel = event->channel;
        xEventGroupClearBits(s_wifi_event_group, DISCONNECTED_BIT);
        xEventGroupSetBits(s_wifi_event_group, CONNECTED_BIT);
    } else if (event_id == WIFI_EVENT_STA_DISCONNECTED) {
        if (s_reconnect && ++s_retry_num < MAX_CONNECT_RETRY_ATTEMPTS) {
            ESP_LOGI(TAG_STA, "sta disconnect, retry attempt %d...", s_retry_num);
            esp_wifi_connect();
        } else {
            ESP_LOGI(TAG_STA, "sta disconnected");
        }
        xEventGroupClearBits(s_wifi_event_group, CONNECTED_BIT);
        xEventGroupSetBits(s_wifi_event_group, DISCONNECTED_BIT);
    } else if (event_id == WIFI_EVENT_FTM_REPORT) {
        wifi_event_ftm_report_t *event = (wifi_event_ftm_report_t *) event_data;

        s_rtt_est = event->rtt_est;
        s_dist_est = event->dist_est;
        s_ftm_report_num_entries = event->ftm_report_num_entries;
        if (event->status == FTM_STATUS_SUCCESS) {
            xEventGroupSetBits(s_ftm_event_group, FTM_REPORT_BIT);
        } else if (event->status == FTM_STATUS_USER_TERM) {
            ESP_LOGI(TAG_STA, "User terminated FTM procedure");
        } else {
            ESP_LOGI(TAG_STA, "FTM procedure with Peer("MACSTR") failed! (Status - %d)",
                    MAC2STR(event->peer_mac), event->status);
            xEventGroupSetBits(s_ftm_event_group, FTM_FAILURE_BIT);
        }
    }
}

void initialise_wifi(void)
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

    if (esp_wifi_get_mode(&cur_mode)) {
        ESP_LOGE(TAG_STA, "Failed to get mode!");
        return ESP_FAIL;
    }

    if (mode == WIFI_MODE_STA) {
        if (cur_mode == WIFI_MODE_STA || cur_mode == WIFI_MODE_APSTA) {
            int bits = xEventGroupWaitBits(s_wifi_event_group, CONNECTED_BIT, 0, 1, 0);
            if (bits & CONNECTED_BIT) {
                s_reconnect = false;
                xEventGroupClearBits(s_wifi_event_group, CONNECTED_BIT);
                ESP_ERROR_CHECK( esp_wifi_disconnect() );
                xEventGroupWaitBits(s_wifi_event_group, DISCONNECTED_BIT, 0, 1, portMAX_DELAY);
            }
            return ESP_OK;
        } else if (cur_mode == WIFI_MODE_AP) {
            new_mode = WIFI_MODE_APSTA;
        } else {
            new_mode = WIFI_MODE_STA;
        }
    }

    ESP_ERROR_CHECK( esp_wifi_set_mode(new_mode) );

    ESP_ERROR_CHECK(esp_wifi_set_mac(WIFI_IF_STA, CONFIG_CSI_SEND_MAC));
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));
    ESP_ERROR_CHECK(esp_wifi_config_espnow_rate(ESP_IF_WIFI_STA, WIFI_PHY_RATE_MCS0_SGI));

    ESP_ERROR_CHECK(esp_now_init());

    return ESP_OK;
}

static bool wifi_cmd_sta_join(const char *ssid, const char *pass)
{

    if (ESP_OK != wifi_add_mode(WIFI_MODE_STA)) {
        return false;
    }

    wifi_config_t wifi_config = { 0 };
    strlcpy((char *) wifi_config.sta.ssid, ssid, sizeof(wifi_config.sta.ssid));
    if (pass) {
        strlcpy((char *) wifi_config.sta.password, pass, sizeof(wifi_config.sta.password));
    }
    ESP_ERROR_CHECK( esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config) );
    ESP_ERROR_CHECK( esp_wifi_connect() );
    s_reconnect = true;
    s_retry_num = 0;

    xEventGroupWaitBits(s_wifi_event_group, CONNECTED_BIT, 0, 1, 5000 / portTICK_PERIOD_MS);

    return true;
}

void print_ftm_data(int rtt, int distance)
{
    ESP_LOGI(TAG_STA, "FTM Data: RTT = %d nSec, Distance = %d.%02d meters",
             rtt, distance / 100, distance % 100);
}

static int wifi_cmd_ftm_initiator(int argc, char **argv)
{
    int nerrors = 0;
    uint32_t wait_time_ms = MIN_WAIT_TIME_MS;
    EventBits_t bits;

    wifi_ftm_initiator_cfg_t ftmi_cfg = {
            .frm_count = 32,
            .burst_period = 2,   // 默认间隔
            .use_get_report_api = true,
    };

    if (nerrors != 0) {
        ESP_LOGE(TAG_STA, "Error in argument parsing");
        return 0;
    }

    bits = xEventGroupWaitBits(s_wifi_event_group, CONNECTED_BIT, 0, 1, 0);
    if (bits & CONNECTED_BIT) {
        memcpy(ftmi_cfg.resp_mac, s_ap_bssid, 6);
        ftmi_cfg.channel = s_ap_channel;
    } else {
        ESP_LOGE(TAG_STA, "Not connected to WiFi, cannot start FTM");
        return 0;
    }

    ESP_LOGI(TAG_STA, "Requesting FTM session with Frame Count - %d, Burst Period - %dmSec",
             ftmi_cfg.frm_count, ftmi_cfg.burst_period);

    if (ESP_OK != esp_wifi_ftm_initiate_session(&ftmi_cfg)) {
        ESP_LOGE(TAG_STA, "Failed to start FTM session");
        return 0;
    }

    if (ftmi_cfg.burst_period) {
        wait_time_ms = (ftmi_cfg.burst_period * 100) * MAX_FTM_BURSTS * 2;
    }

    bits = xEventGroupWaitBits(s_ftm_event_group, FTM_REPORT_BIT | FTM_FAILURE_BIT,
                               pdTRUE, pdFALSE, wait_time_ms / portTICK_PERIOD_MS);

    if (bits & FTM_REPORT_BIT) {
        ESP_LOGI(TAG_STA, "FTM session complete. Estimated RTT - %" PRId32 " nSec, Estimated Distance - %" PRId32 ".%02" PRId32 " meters",
                s_rtt_est, s_dist_est / 100, s_dist_est % 100);

        print_ftm_data(s_rtt_est, s_dist_est);
    } else if (bits & FTM_FAILURE_BIT) {
        ESP_LOGE(TAG_STA, "FTM session failed!");
    } else {
        ESP_LOGE(TAG_STA, "FTM session timed out!");
        esp_wifi_ftm_end_session();
    }

    return 0;
}

static void ftm_measurement_task(void *pvParameter)
{
    while (true) {
        wifi_cmd_ftm_initiator(0, NULL);
        vTaskDelay(pdMS_TO_TICKS(MIN_WAIT_TIME_MS));

        ESP_ERROR_CHECK(esp_now_add_peer(&csi_peer));
        uint8_t payload[] = {0x01};
        esp_err_t ret = esp_now_send(csi_peer.peer_addr, payload, sizeof(payload));
        if(ret != ESP_OK) {
            ESP_LOGW("CSI sender", "<%s> ESP-NOW send error", esp_err_to_name(ret));
        }
        ESP_ERROR_CHECK(esp_now_del_peer(csi_peer.peer_addr));

        vTaskDelay(pdMS_TO_TICKS(MIN_WAIT_TIME_MS));
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

    if (!wifi_cmd_sta_join(DEFAULT_AP_SSID, DEFAULT_AP_PASSWORD)) {
        ESP_LOGE(TAG_STA, "Failed to join wifi!");
        return;
    }

    xTaskCreate(ftm_measurement_task, "ftm_measurement_task", 4096, NULL, 5, NULL);
}

