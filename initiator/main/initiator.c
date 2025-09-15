// ===== TAG / INITIATOR: FTM (bursty) + CSI RX (1Hz print, anchor-filtered) =====
#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include <math.h>
#include <stdbool.h>
#include "esp_err.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_heap_caps.h"
#include "nvs_flash.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "freertos/queue.h"

#include "esp_wifi.h"
#include "esp_mac.h"
#include "driver/uart.h"

#include "lwip/sockets.h"
#include "lwip/inet.h"
#include "lwip/ip4_addr.h"
#include "lwip/netdb.h"

#define TAG   "TAG"
#define SSID  "FTM"
#define PASS  "ftmftmftm"

#define FTM_FRM_COUNT      8
#define FTM_BURST_PERIOD   3
#define FTM_RECYCLE_GAP_MS 200

// ===================== Anchor config =====================
typedef struct {
    const char *name;         // 锚点名（日志友好）
    uint8_t     bssid[6];     // 目标锚点 BSSID
    uint8_t     channel;      // 主信道（不确定可填 0，落回当前连接 ch）
    float       gt_dist_m;    // 现场“真值”距离（未知填负数）
} anchor_t;

static anchor_t g_anchors[] = {
        { "A1", {0x24,0xEC,0x4A,0x03,0x58,0x5D}, 1, -1.0f },
        { "A2", {0x24,0xEC,0x4A,0x04,0x39,0x15}, 6, -1.0f },
        { "A3", {0x24,0xEC,0x4A,0x03,0x56,0x41}, 11, -1.0f }
        // TODO: 若有真值距离，改成正数（例如 3.60f）
        // 后续你可以在这里继续追加：{ "A2", {..}, ch, gt }
};
static int g_anchor_count = sizeof(g_anchors) / sizeof(g_anchors[0]);
static const anchor_t *g_curr = &g_anchors[0]; // 当前锚点（轮询时会切换）
// ========================================================

// ---- Events ----
static EventGroupHandle_t s_wifi_eg;
static EventGroupHandle_t s_ftm_eg;
static const int WIFI_CONNECTED_BIT = BIT0;
static const int FTM_REPORT_BIT     = BIT1;
static const int FTM_FAIL_BIT       = BIT2;
static const int WIFI_DISCONNECTED_BIT = BIT3;

// ---- FTM snapshot from event ----
static volatile uint32_t g_rtt_raw_ns_min = 0;
static volatile uint32_t g_rtt_est_ns     = 0;
static volatile uint32_t g_dist_cm        = 0;

// ---- 保存 AP BSSID 与信道（用于兜底）----
static uint8_t g_ap_bssid[6] = {0};
static uint8_t g_ap_channel  = 0;

// ====== Anchor snapshot for ISR-safe filtering (避免轮询竞态) ======
static volatile uint8_t s_curr_bssid[6]   = {0};
static volatile uint8_t s_curr_channel    = 0;
static volatile int     s_curr_anchor_idx = 0;

// ---- CSI queue（带长度+锚点索引；只收锚点的帧）----
typedef struct {
    uint16_t len;
    int8_t   data[128];
    uint8_t  anchor_idx;   // 该帧归属的锚点（由 ISR 按快照写入）
} csi_item_t;
static QueueHandle_t s_csi_q;

static int s_ftm_fail_streak = 0;

static esp_netif_t *s_sta_netif = NULL;

static bool is_connected_to(const uint8_t bssid[6]);

static inline void kick_udp_to_gw(void) {
    int s = socket(AF_INET, SOCK_DGRAM, 0);
    if (s < 0) return;
    struct sockaddr_in to = {0};
    to.sin_family = AF_INET;
    to.sin_port   = htons(33333);
    to.sin_addr.s_addr = inet_addr("192.168.4.1");  // 你的网关（按日志一直是这个）
    uint8_t b = 0;
    (void)sendto(s, &b, 1, 0, (struct sockaddr*)&to, sizeof(to));
    close(s);
}

static void wifi_event_handler(void *arg, esp_event_base_t base, int32_t id, void *data) {
    if (base == WIFI_EVENT && id == WIFI_EVENT_STA_CONNECTED) {
        wifi_event_sta_connected_t *ev = (wifi_event_sta_connected_t*)data;

        // 保存“已连接 AP”的 BSSID 与信道（有时可作为 FTM/CSI 的兜底参数）
        memcpy((void*)g_ap_bssid, ev->bssid, 6);
        g_ap_channel = ev->channel;

        uint8_t pri = 0; wifi_second_chan_t sec = WIFI_SECOND_CHAN_NONE;
        if (esp_wifi_get_channel(&pri, &sec) == ESP_OK && pri != 0) g_ap_channel = pri;

        xEventGroupSetBits(s_wifi_eg, WIFI_CONNECTED_BIT);
        ESP_LOGI(TAG, "STA connected to " MACSTR ", ch=%u (sec=%d)",
                MAC2STR(g_ap_bssid), g_ap_channel, (int)sec);

    } else if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED) {
        g_ap_channel = 0;
        memset((void*)g_ap_bssid, 0, 6);
        xEventGroupClearBits(s_wifi_eg, WIFI_CONNECTED_BIT);
        xEventGroupSetBits(s_wifi_eg, WIFI_DISCONNECTED_BIT);

    } else if (base == WIFI_EVENT && id == WIFI_EVENT_FTM_REPORT) {
        wifi_event_ftm_report_t *ev = (wifi_event_ftm_report_t*)data;
        g_rtt_raw_ns_min = ev->rtt_raw;
        g_rtt_est_ns     = ev->rtt_est;
        g_dist_cm        = ev->dist_est;

        if (ev->status == FTM_STATUS_SUCCESS) xEventGroupSetBits(s_ftm_eg, FTM_REPORT_BIT);
        else                                  xEventGroupSetBits(s_ftm_eg, FTM_FAIL_BIT);
    }else if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED) {
        g_ap_channel = 0;
        memset((void*)g_ap_bssid, 0, 6);
        xEventGroupClearBits(s_wifi_eg, WIFI_CONNECTED_BIT);
        xEventGroupSetBits(s_wifi_eg, WIFI_DISCONNECTED_BIT);
        esp_wifi_connect();
    }
}

static bool wait_sta_connected(uint32_t timeout_ms)
{
    EventBits_t b = xEventGroupWaitBits(
            s_wifi_eg, WIFI_CONNECTED_BIT, pdFALSE, pdTRUE, pdMS_TO_TICKS(timeout_ms)
    );
    return (b & WIFI_CONNECTED_BIT) != 0;
}

static bool wait_sta_connected(uint32_t timeout_ms);

static esp_err_t assoc_to_anchor(const anchor_t* a, uint32_t timeout_ms)
{
    if (is_connected_to(a->bssid)) return ESP_OK;

    // 只改 bssid/channel，避免反复重置其它 STA 参数
    wifi_config_t sta = {0};
    ESP_ERROR_CHECK(esp_wifi_get_config(WIFI_IF_STA, &sta));
    sta.sta.bssid_set = true;
    memcpy(sta.sta.bssid, a->bssid, 6);
    sta.sta.channel = a->channel;
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &sta));

    // 先断开，并等待“真的断开”
    xEventGroupClearBits(s_wifi_eg, WIFI_DISCONNECTED_BIT);
    (void)esp_wifi_disconnect();
    (void)xEventGroupWaitBits(s_wifi_eg, WIFI_DISCONNECTED_BIT,
                              pdTRUE, pdTRUE, pdMS_TO_TICKS(1500));

    vTaskDelay(pdMS_TO_TICKS(50)); // 小缓冲，避免立刻撞状态机

    // 再连接：不要用 ESP_ERROR_CHECK，容忍“正在连接”返回值
    esp_err_t rc = esp_wifi_connect();
    if (rc != ESP_OK && rc != ESP_ERR_WIFI_CONN) {
        return rc;  // 其他错误直接返回
    }

    // 等待连上
    return wait_sta_connected(timeout_ms) ? ESP_OK : ESP_ERR_TIMEOUT;
}

static void switch_snapshot_to(int idx)
{
    g_curr = &g_anchors[idx];
    for (int i = 0; i < 6; ++i) s_curr_bssid[i] = g_curr->bssid[i];
    s_curr_channel    = (g_curr->channel != 0) ? g_curr->channel : g_ap_channel;
    s_curr_anchor_idx = idx;

    // ★ 清空上一轮残留，避免打印“旧 CSI”
    xQueueReset(s_csi_q);
}

// ===== Init =====
static void set_uart_baudrate(void) {
    uart_set_baudrate(UART_NUM_0, 921600);   // PC 端串口同样设为 921600
}

static bool is_connected_to(const uint8_t bssid[6]) {
    wifi_ap_record_t ap = {0};
    if (esp_wifi_sta_get_ap_info(&ap) == ESP_OK) {
        return memcmp(ap.bssid, bssid, 6) == 0;
    }
    return false;
}

static void wifi_init_and_connect(void) {
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    s_sta_netif = esp_netif_create_default_wifi_sta();   // ★ 保存 netif 句柄

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(
            WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL, NULL));

    // ★ 先关 DHCP，并设置静态 IP（SoftAP 网段按你的日志：192.168.4.x）
    esp_netif_ip_info_t ip;
    IP4_ADDR(&ip.ip,      192,168,4,2);
    IP4_ADDR(&ip.netmask, 255,255,255,0);
    IP4_ADDR(&ip.gw,      192,168,4,1);
    ESP_ERROR_CHECK(esp_netif_dhcpc_stop(s_sta_netif));
    ESP_ERROR_CHECK(esp_netif_set_ip_info(s_sta_netif, &ip));

    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));

    wifi_config_t sta = (wifi_config_t){0};
    strncpy((char*)sta.sta.ssid, SSID, sizeof(sta.sta.ssid)-1);
    strncpy((char*)sta.sta.password, PASS, sizeof(sta.sta.password)-1);
    sta.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &sta));

    ESP_ERROR_CHECK(esp_wifi_start());

    // ★ 启用 11n（HT），CSI 需要 HT-LTF
    ESP_ERROR_CHECK(esp_wifi_set_protocol(
            WIFI_IF_STA, WIFI_PROTOCOL_11B | WIFI_PROTOCOL_11G | WIFI_PROTOCOL_11N));

    // 首次连接（后续切锚点你自己控制）
    esp_err_t rc = esp_wifi_connect();
    if (rc != ESP_OK && rc != ESP_ERR_WIFI_CONN) {
        ESP_ERROR_CHECK(rc);
    }
}

static void csi_enable(void) {
    wifi_promiscuous_filter_t f = {
            .filter_mask = WIFI_PROMIS_FILTER_MASK_MGMT | WIFI_PROMIS_FILTER_MASK_DATA
    };
    esp_wifi_set_promiscuous_filter(&f);
    ESP_ERROR_CHECK(esp_wifi_set_promiscuous(true));

    wifi_csi_config_t cfg = {
            .lltf_en = true, .htltf_en = true, .stbc_htltf2_en = true,
            .ltf_merge_en = true, .channel_filter_en = true,
            .manu_scale = false, .shift = false,
    };
    ESP_ERROR_CHECK(esp_wifi_set_csi_config(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_csi(true));
}

// ===== CSI RX callback: filter by anchor BSSID (snapshot), enqueue =====
static void csi_rx_cb(void *ctx, wifi_csi_info_t *info) {
    if (!info || !info->buf) return;
    if (memcmp(info->mac, (const void*)s_curr_bssid, 6) != 0) return;

    csi_item_t it = {0};
    it.len = (info->len > 128) ? 128 : info->len;
    for (int i = 0; i < it.len; ++i) it.data[i] = ((const int8_t*)info->buf)[i];
    it.anchor_idx = (uint8_t)s_curr_anchor_idx;

    (void)xQueueOverwriteFromISR(s_csi_q, &it, NULL);  // ★ 只保留最新
}

static inline void csi_pause(bool pause) {
    if (pause) {
        esp_wifi_set_csi(false);
        esp_wifi_set_promiscuous(false);
    } else {
        esp_wifi_set_promiscuous(true);
        esp_wifi_set_csi(true); // 配置已保留，不必重复 set_csi_config
    }
}

static esp_err_t do_ftm_once_and_print_frames(void) {
    EventBits_t b = xEventGroupWaitBits(s_wifi_eg, WIFI_CONNECTED_BIT,
                                        pdFALSE, pdTRUE, pdMS_TO_TICKS(2000));
    if (!(b & WIFI_CONNECTED_BIT)) return ESP_FAIL;

    // RAM 余量太低直接拒绝，避免雪崩
    size_t free_int = heap_caps_get_free_size(MALLOC_CAP_INTERNAL);
    if (free_int < 24 * 1024) {
        ESP_LOGW(TAG, "Low INT heap (%uB), skip FTM.", (unsigned)free_int);
        return ESP_ERR_NO_MEM;
    }

    uint8_t ch = s_curr_channel ? s_curr_channel : g_ap_channel;
    if (ch == 0) {
        ESP_LOGW(TAG, "Anchor channel unknown; skip.");
        return ESP_FAIL;
    }

    wifi_ftm_initiator_cfg_t cfg = {
            .frm_count    = FTM_FRM_COUNT,     // ★ 缩小 burst
            .burst_period = FTM_BURST_PERIOD,  // ★ 300ms
    };
    memcpy(cfg.resp_mac, g_curr->bssid, 6);
    cfg.channel = ch;

    xEventGroupClearBits(s_ftm_eg, FTM_REPORT_BIT | FTM_FAIL_BIT);

    // ★ FTM 期间暂停 CSI/混杂模式，释放 RX/管理缓冲
    csi_pause(true);

    esp_err_t r = esp_wifi_ftm_initiate_session(&cfg);
    if (r != ESP_OK) {
        vTaskDelay(pdMS_TO_TICKS(100));
        csi_pause(false);
        return r;
    }

    EventBits_t e = xEventGroupWaitBits(s_ftm_eg, FTM_REPORT_BIT | FTM_FAIL_BIT,
                                        pdTRUE, pdFALSE, pdMS_TO_TICKS(4000));

    // ★ 统一清理（成功/失败/超时都调用）
    (void)esp_wifi_ftm_end_session();

    // ★ 给驱动回收一个小Gap（对管理缓冲很有用）
    vTaskDelay(pdMS_TO_TICKS(FTM_RECYCLE_GAP_MS));

    // ★ 恢复 CSI
    csi_pause(false);

    if (e & FTM_FAIL_BIT) return ESP_FAIL;
    if (!(e & FTM_REPORT_BIT)) return ESP_ERR_TIMEOUT;

    float est_m = (g_dist_cm > 0) ? (g_dist_cm / 100.0f) : -1.0f;
    float err_m = (g_curr->gt_dist_m > 0 && est_m > 0) ? (est_m - g_curr->gt_dist_m) : NAN;

    printf("anchor,%s," MACSTR ",ch=%u\n", g_curr->name, MAC2STR(g_curr->bssid), (unsigned)ch);
    printf("ftm_report,%s,%u,%u,%.3f", g_curr->name,
           (unsigned)g_rtt_raw_ns_min, (unsigned)g_rtt_est_ns, est_m);
    if (!isnan(err_m)) printf(",err=%.3f", err_m);
    putchar('\n');

    return ESP_OK;
}

// ===== CSI logger task: print ~1Hz (take latest only) =====
static void csi_logger_task(void *arg) {
    const TickType_t period = pdMS_TO_TICKS(1000);
    csi_item_t it, last; bool have = false;

    for (;;) {
        TickType_t t0 = xTaskGetTickCount();
        while (xQueueReceive(s_csi_q, &it, 0) == pdTRUE) { last = it; have = true; }
        if (have) {
            // 输出：csi_data,<anchor_name>,[v0,v1,...]  —— 仅来自锚点
            fputs("csi_data,", stdout);
            const char *aname = g_anchors[last.anchor_idx].name;  // 用队列中的归属索引，不受轮询切换影响
            fputs(aname, stdout);
            putchar_unlocked(','); putchar_unlocked('[');
            for (int i = 0; i < last.len; ++i) {
                char buf[8];
                int len = snprintf(buf, sizeof(buf), "%d", (int)last.data[i]);
                for (int k = 0; k < len; ++k) putchar_unlocked(buf[k]);
                if (i != last.len - 1) putchar_unlocked(',');
            }
            putchar_unlocked(']'); putchar_unlocked('\n');
            fflush(stdout);
            have = false;
        }
        vTaskDelayUntil(&t0, period);
    }
}

static void wifi_soft_reset_and_reconnect(void)
{
    ESP_LOGW(TAG, "WiFi soft reset to recover buffers...");

    // 确保先断开，并等待真正断开，避免状态机撞车
    xEventGroupClearBits(s_wifi_eg, WIFI_DISCONNECTED_BIT);
    (void)esp_wifi_disconnect();
    (void)xEventGroupWaitBits(s_wifi_eg, WIFI_DISCONNECTED_BIT,
                              pdTRUE, pdTRUE, pdMS_TO_TICKS(1500));

    // 重启 Wi-Fi
    esp_wifi_stop();
    vTaskDelay(pdMS_TO_TICKS(200));
    esp_wifi_start();
    esp_wifi_set_ps(WIFI_PS_NONE);

    // 重新连接：容忍“正在连接”状态
    esp_err_t rc = esp_wifi_connect();
    if (rc != ESP_OK && rc != ESP_ERR_WIFI_CONN) {
        ESP_LOGW(TAG, "connect rc=%s", esp_err_to_name(rc));
        return;
    }

    // 等待连上
    if (!wait_sta_connected(8000)) {
        ESP_LOGW(TAG, "Reconnect timeout.");
    } else {
        ESP_LOGI(TAG, "STA reconnected.");
    }
}


// ===== Measure loop: round-robin anchors (currently 1) =====
static void measure_task(void *arg) {
    int idx = 0;
    for (;;) {
        // (a) 先关联到该锚点
        if (assoc_to_anchor(&g_anchors[idx], 6000) != ESP_OK) {
            ESP_LOGW(TAG, "Assoc to %s timeout, skip.", g_anchors[idx].name);
            idx = (idx + 1) % g_anchor_count;
            vTaskDelay(pdMS_TO_TICKS(300));
            continue;
        }

        // (b) 更新快照并清空队列
        switch_snapshot_to(idx);
        vTaskDelay(pdMS_TO_TICKS(20)); // 快照生效

        // (c) 让 CSI 先抓一小段（同信道下能抓到该锚点的 beacon/data）
        // 如果你一直开着 CSI，这一步可以只留个很短的过渡
        vTaskDelay(pdMS_TO_TICKS(120));

        // (d) 发起该锚点 FTM（函数里会 暂停CSI→FTM→恢复CSI）
        esp_err_t r = do_ftm_once_and_print_frames();

        kick_udp_to_gw();
        vTaskDelay(pdMS_TO_TICKS(30));

        if (r == ESP_OK) {
            s_ftm_fail_streak = 0;
        } else {
            s_ftm_fail_streak++;
            ESP_LOGW(TAG, "FTM to %s failed (%s), streak=%d",
                     g_curr->name, esp_err_to_name(r), s_ftm_fail_streak);
            if (s_ftm_fail_streak >= 3) {
                wifi_soft_reset_and_reconnect(); // 会重新连到上一次配置的 BSSID
                s_ftm_fail_streak = 0;
                vTaskDelay(pdMS_TO_TICKS(300));
            }
        }

        // (e) 下一锚点 & 间隔
        idx = (idx + 1) % g_anchor_count;
        vTaskDelay(pdMS_TO_TICKS(500)); // 可按需要调
    }
}

void app_main(void) {
    setvbuf(stdout, NULL, _IONBF, 0);
    setvbuf(stderr, NULL, _IONBF, 0);

    esp_log_level_set("wifi", ESP_LOG_ERROR);
    esp_log_level_set("phy",  ESP_LOG_ERROR);

    ESP_ERROR_CHECK(nvs_flash_init());
    set_uart_baudrate();  // 避免未用函数告警

    s_wifi_eg = xEventGroupCreate();
    s_ftm_eg  = xEventGroupCreate();
    s_csi_q   = xQueueCreate(1, sizeof(csi_item_t));

    wifi_init_and_connect();
    wait_sta_connected(8000);

    // CSI
    csi_enable();
    ESP_ERROR_CHECK(esp_wifi_set_csi_rx_cb(csi_rx_cb, NULL));

    // Tasks
    xTaskCreate(csi_logger_task, "csi_logger", 4096, NULL, 4, NULL);
    xTaskCreate(measure_task,   "measure",     4096, NULL, 5, NULL);

    ESP_LOGI(TAG, "Ready (multi-anchor): FTM burst + CSI 1Hz, UART=921600.");
}
