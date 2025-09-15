// ===== ANCHOR / RESPONDER: SoftAP + FTM Responder + Periodic ESP-NOW TX =====
#include <stdio.h>
#include <string.h>
#include "esp_log.h"
#include "esp_system.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "nvs_flash.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_wifi.h"
#include "esp_now.h"

#define TAG                "ANCHOR"

// -------- AP/FTM 参数（按现场修改） --------
#define AP_SSID            "FTM"
#define AP_PASS            "ftmftmftm"
#define AP_CHANNEL         6              // 1..13/14，确保非 0
#define AP_BW_20MHZ        1              // 20MHz
#define MAX_STA_CONN       8

// -------- ESP-NOW 周期发送设置 --------
#define ESPNOW_HZ          10
#define ESPNOW_PERIOD_MS   (1000 / ESPNOW_HZ)
#define ESPNOW_PAYLOAD_LEN 10

// 速率设置（11n 帧，便于 CSI）
#define ESPNOW_RATE        WIFI_PHY_RATE_MCS0_SGI

static const uint8_t ESPNOW_PEER_BROADCAST[6] = {0xff,0xff,0xff,0xff,0xff,0xff};

// ---------- 事件回调 ----------
static void wifi_event_handler(void *arg, esp_event_base_t base, int32_t id, void *data)
{
    if (base == WIFI_EVENT && id == WIFI_EVENT_AP_START) {
        wifi_config_t cfg;
        esp_wifi_get_config(WIFI_IF_AP, &cfg);

        uint8_t mac[6];
        esp_wifi_get_mac(WIFI_IF_AP, mac);

        uint8_t pri = 0;
        wifi_second_chan_t sec = WIFI_SECOND_CHAN_NONE;
        esp_wifi_get_channel(&pri, &sec);

        ESP_LOGI(TAG,
                 "AP started. SSID:%s CH:%u BW:%s AP_MAC:%02X:%02X:%02X:%02X:%02X:%02X (sec=%d, FTM ON)",
                 (char*)cfg.ap.ssid, pri, (AP_BW_20MHZ ? "20" : "40"),
                 mac[0],mac[1],mac[2],mac[3],mac[4],mac[5], (int)sec);
    }
}

// ---------- 初始化 SoftAP + FTM Responder ----------
static void wifi_ap_init(void)
{
    esp_log_level_set("wifi", ESP_LOG_DEBUG); // 可选：便于排查

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_ap();

    wifi_init_config_t wcfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&wcfg));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(
            WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL, NULL));

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));

    // 建议：设置国家码与 11n 协议
    wifi_country_t ctry = { .cc = "CN", .schan = 1, .nchan = 13, .policy = WIFI_COUNTRY_POLICY_MANUAL };
    ESP_ERROR_CHECK(esp_wifi_set_country(&ctry));
    ESP_ERROR_CHECK(esp_wifi_set_protocol(WIFI_IF_AP,
                                          WIFI_PROTOCOL_11B | WIFI_PROTOCOL_11G | WIFI_PROTOCOL_11N));

    wifi_config_t ap_cfg = { 0 };
    strncpy((char*)ap_cfg.ap.ssid, AP_SSID, sizeof(ap_cfg.ap.ssid)-1);
    strncpy((char*)ap_cfg.ap.password, AP_PASS, sizeof(ap_cfg.ap.password)-1);
    ap_cfg.ap.ssid_len        = 0;
    ap_cfg.ap.channel         = AP_CHANNEL;                // 明确主信道（非 0）
    ap_cfg.ap.max_connection  = MAX_STA_CONN;
    ap_cfg.ap.authmode        = (strlen(AP_PASS) >= 8) ? WIFI_AUTH_WPA2_PSK : WIFI_AUTH_OPEN;
    ap_cfg.ap.beacon_interval = 100;                       // ms
    ap_cfg.ap.ftm_responder   = true;                      // 开启 FTM Responder

    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &ap_cfg));

    if (AP_BW_20MHZ) ESP_ERROR_CHECK(esp_wifi_set_bandwidth(WIFI_IF_AP, WIFI_BW_HT20));
    else             ESP_ERROR_CHECK(esp_wifi_set_bandwidth(WIFI_IF_AP, WIFI_BW_HT40));

    ESP_ERROR_CHECK(esp_wifi_start());

    // 启动后兜底：如果当前信道=0，强制设置为 AP_CHANNEL
    uint8_t pri = 0;
    wifi_second_chan_t sec = WIFI_SECOND_CHAN_NONE;
    ESP_ERROR_CHECK(esp_wifi_get_channel(&pri, &sec));
    if (pri == 0) {
        ESP_LOGW(TAG, "current channel=0 ! Forcing AP_CHANNEL=%d", AP_CHANNEL);
        ESP_ERROR_CHECK(esp_wifi_set_channel(AP_CHANNEL, WIFI_SECOND_CHAN_NONE));
        ESP_ERROR_CHECK(esp_wifi_get_channel(&pri, &sec));
    }
    ESP_LOGI(TAG, "AP up on channel=%u (sec=%d)", pri, (int)sec);
}

// ---------- 初始化 ESP-NOW 并添加广播 Peer ----------
static void espnow_init(void)
{
    ESP_ERROR_CHECK(esp_now_init());

    // 旧接口：在 AP 接口上统一设置 ESP-NOW 速率（v5.4 仍可用，可能会有弃用告警，但功能正常）
    ESP_ERROR_CHECK(esp_wifi_config_espnow_rate(WIFI_IF_AP, ESPNOW_RATE));

    esp_now_peer_info_t peer = {0};
    memcpy(peer.peer_addr, ESPNOW_PEER_BROADCAST, 6);
    peer.ifidx   = WIFI_IF_AP;
    peer.channel = AP_CHANNEL;     // 关键：明确非 0
    peer.encrypt = false;

    esp_err_t err = esp_now_add_peer(&peer);
    if (err != ESP_OK && err != ESP_ERR_ESPNOW_EXIST) {
        ESP_ERROR_CHECK(err);
    }
}

// ---------- 周期性发送 ESPNOW 小包（用于触发对端 CSI） ----------
static void espnow_tx_task(void *arg)
{
    uint32_t seq = 0;
    uint8_t  payload[ESPNOW_PAYLOAD_LEN];

    for (;;) {
        memcpy(payload, &seq, sizeof(seq));
        for (int i = 4; i < ESPNOW_PAYLOAD_LEN; ++i) payload[i] = (uint8_t)i;

        esp_err_t r = esp_now_send(ESPNOW_PEER_BROADCAST, payload, ESPNOW_PAYLOAD_LEN);
        if (r != ESP_OK) ESP_LOGW(TAG, "esp_now_send err=%s", esp_err_to_name(r));
        seq++;

        vTaskDelay(pdMS_TO_TICKS(ESPNOW_PERIOD_MS)); // 10Hz
    }
}

void app_main(void)
{
    ESP_ERROR_CHECK(nvs_flash_init());

    wifi_ap_init();              // SoftAP + FTM Responder
    espnow_init();               // ESP-NOW TX
    xTaskCreate(espnow_tx_task, "espnow_tx", 2048, NULL, 5, NULL);

    ESP_LOGI(TAG, "Anchor ready: SoftAP+FTM Responder, ESPNOW %d Hz on CH%d, BW=%s",
             ESPNOW_HZ, AP_CHANNEL, (AP_BW_20MHZ ? "20" : "40"));
}
