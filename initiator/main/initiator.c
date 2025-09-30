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
#include <stdlib.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "freertos/queue.h"

#include "esp_wifi.h"
#include "esp_wifi_types.h"
#include "esp_mac.h"
#include "esp_timer.h"
#include "driver/uart.h"

#include "lwip/sockets.h"
#include "lwip/inet.h"
#include "lwip/ip4_addr.h"
#include "lwip/netdb.h"

#define TAG   "TAG"
#define SSID  "FTM"
#define PASS  "ftmftmftm"

#define FTM_FRM_COUNT      32
#define FTM_BURST_PERIOD   3
#define FTM_RECYCLE_GAP_MS 200

#ifndef MAX_FRAMES_USED
#define MAX_FRAMES_USED 32
#endif

#ifndef EXPECT_FRAMES
#define EXPECT_FRAMES   32
#endif

// ===================== Anchor config =====================
typedef struct {
    const char *name;         // 锚点名（日志友好）
    uint8_t     bssid[6];     // 目标锚点 BSSID
    uint8_t     channel;      // 主信道（不确定可填 0，落回当前连接 ch）
    float       gt_dist_m;    // 现场“真值”距离（未知填负数）
} anchor_t;

static anchor_t g_anchors[] = {
        //{ "A1", {0x24,0xEC,0x4A,0x03,0x58,0x5D}, 1, -1.0f },
        //{ "A2", {0x24,0xEC,0x4A,0x04,0x39,0x15}, 6, -1.0f },
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
static volatile uint8_t g_ftm_entries = 0;

static wifi_ftm_report_entry_t *s_ftm_copy = NULL;
static volatile uint8_t         s_ftm_copy_num = 0;

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
    uint8_t  anchor_idx;
    int8_t   rssi;
} csi_item_t;

static inline bool keep_sc(int k){
    // 20 MHz, HT-LTF: keep ±1..±28；若想用 LLTF 改成 ±1..±26
    if (k >= -28 && k <= -1) return true;
    if (k >=  +1 && k <= +28) return true;
    return false;
}

static inline bool is_pilot_k(int k) {
    // 802.11n HT-LTF pilots (20 MHz): k = ±7, ±21
    return (k == -21 || k == -7 || k == 7 || k == 21);
}

static void detrend_phase(float *y, const float *x, int n){
    double Sx=0,Sy=0,Sxx=0,Sxy=0;
    for(int i=0;i<n;++i){ double xi=x[i], yi=y[i]; Sx+=xi; Sy+=yi; Sxx+=xi*xi; Sxy+=xi*yi; }
    double den = n*Sxx - Sx*Sx;
    double a = (den!=0) ? (n*Sxy - Sx*Sy)/den : 0.0;
    double b = (Sy - a*Sx)/n;
    double m=0;
    for(int i=0;i<n;++i){ y[i] = (float)(y[i] - (a*x[i] + b)); m += y[i]; }
    m /= n;
    for(int i=0;i<n;++i) y[i] -= (float)m;
}

static QueueHandle_t s_csi_q;

static void print_csi_features_from_iq(const int8_t *iq, int nbytes,
                                       int rssi_dbm, const char *aname)
{
    if (!iq || nbytes < 4) return;
    const int nbin = nbytes / 2; // 期望 64 (I,Q 交错)
    float mag[64], ph[64], xk[64];
    int   keep = 0;

    // 先构建所有“保留子载波”的幅度/相位/索引
    // 同时收集导频相位用于估计 phi0（圆均值）
    double csum = 0.0, ssum = 0.0;
    int pilot_cnt = 0;

    for (int i = 0; i < nbin; ++i) {
        int k = i - 32;                 // map to [-32..+31], 0 为 DC
        if (k == 0) continue;
        if (!keep_sc(k)) continue;

        float I = (float)iq[2*i + 0];
        float Q = (float)iq[2*i + 1];

        mag[keep] = sqrtf(I*I + Q*Q);
        ph[keep]  = atan2f(Q, I);       // 原始相位（未展开）
        xk[keep]  = (float)k;

        if (is_pilot_k(k)) {
            csum += cos((double)ph[keep]);
            ssum += sin((double)ph[keep]);
            pilot_cnt++;
        }
        keep++;
    }
    if (keep <= 0) return;

    // 用导频做“全局相位补偿” (公共相位偏移)
    if (pilot_cnt >= 2) {
        float phi0 = (float)atan2(ssum, csum);   // 圆均值相位
        for (int i = 0; i < keep; ++i) {
            ph[i] -= phi0;
            // 规约到 [-pi, pi]
            if (ph[i] >  (float)M_PI)  ph[i] -= 2.0f*(float)M_PI;
            if (ph[i] < -(float)M_PI)  ph[i] += 2.0f*(float)M_PI;
        }
    }

    // 对补偿后的相位做 unwrap（保持与子载波顺序一致）
    for (int i = 1; i < keep; ++i) {
        float d = ph[i] - ph[i-1];
        if (d >  (float)M_PI)  ph[i] -= 2.0f*(float)M_PI;
        if (d < -(float)M_PI)  ph[i] += 2.0f*(float)M_PI;
    }

    // 去趋势 + 去均值（消除线性斜率与整体偏置）
    detrend_phase(ph, xk, keep);

    // 只打印一次“分析行”
    printf("csi_feat,%s,rssi=%d,mag=[", aname, rssi_dbm);
    for (int i = 0; i < keep; ++i) { if (i) putchar(','); printf("%.3f", mag[i]); }
    fputs("],phi=[", stdout);
    for (int i = 0; i < keep; ++i) { if (i) putchar(','); printf("%.4f", ph[i]); }
    fputs("]\n", stdout);
}


// —— CSI 一次性打印门控 ——
static volatile bool     g_csi_print_armed   = false;   // 是否武装“下一次只打一次”
static volatile uint64_t g_csi_earliest_us   = 0;       // 允许打印的最早时间
static volatile int      g_csi_expected_idx  = -1;      // 期望打印的锚点索引

static int s_ftm_fail_streak = 0;

static esp_netif_t *s_sta_netif = NULL;

static bool is_connected_to(const uint8_t bssid[6]);

// ===== RTT/RSSI 统计（与Python一致：p分位采用线性插值 idx=p*(n-1)） =====
static int cmp_u32(const void *a, const void *b){
    uint32_t x=*(const uint32_t*)a, y=*(const uint32_t*)b;
    return (x>y)-(x<y);
}

static int cmp_double(const void *a, const void *b) {
    double x = *(const double*)a;
    double y = *(const double*)b;
    return (x > y) - (x < y);
}

// 线性插值分位（uint32数组已排序，返回double）
static double quantile_u32_lin(const uint32_t *xs, int n, double p){
    if(n<=0) return NAN;
    if(n==1) return (double)xs[0];
    if(p<=0) return (double)xs[0];
    if(p>=1) return (double)xs[n-1];
    double idx = p*(n-1);
    int lo = (int)floor(idx), hi = (int)ceil(idx);
    double w = idx - lo;
    return (1.0-w)*xs[lo] + w*xs[hi];
}

// population std (ddof=0)
static double mean_double(const double *x, int n){
    if(n<=0) return NAN;
    double s=0; for(int i=0;i<n;++i) s+=x[i]; return s/n;
}
static double std_double(const double *x, int n){
    if(n<=1) return 0.0;
    double mu = mean_double(x,n), acc=0;
    for(int i=0;i<n;++i){ double d=x[i]-mu; acc+=d*d; }
    return sqrt(acc/n);
}

typedef struct {
    // RTT (ps)
    double rtt_min, rtt_k2, rtt_median, rtt_p10, rtt_p25, rtt_p75;
    double rtt_iqr, rtt_mad, rtt_mean_trim, rtt_std, rtt_valid_ratio;
    // RSSI (dBm)
    double rssi_mean, rssi_min, rssi_std;
} rtt_feat_t;

// 传入：rtt_ps[]、rssi_dbm[]，len=nframes（<=16）; 期望 burst=16 → valid_ratio=len/16
static rtt_feat_t make_rtt_feats(const uint32_t *rtt_ps, const int32_t *rssi_dbm, int len){
    rtt_feat_t f = {0};
    if(len<=0){ f.rtt_valid_ratio = 0.0; f.rssi_min = NAN; f.rssi_mean = NAN; return f; }

    // 拷贝+排序（RTT升序）；以避免破坏原数组
    uint32_t tmp[MAX_FRAMES_USED];
    for(int i=0;i<len;++i) tmp[i]=rtt_ps[i];
    qsort(tmp, len, sizeof(uint32_t), cmp_u32);

    // 基本分位（ns）
    f.rtt_min    = (double)tmp[0];
    f.rtt_k2     = (double)(len>=2 ? tmp[1] : tmp[0]);
    f.rtt_median = quantile_u32_lin(tmp, len, 0.5);
    f.rtt_p10    = quantile_u32_lin(tmp, len, 0.10);
    f.rtt_p25    = quantile_u32_lin(tmp, len, 0.25);
    f.rtt_p75    = quantile_u32_lin(tmp, len, 0.75);
    f.rtt_iqr    = f.rtt_p75 - f.rtt_p25;

    // MAD = median(|x - median|)
    double med = f.rtt_median;
    double absdev[MAX_FRAMES_USED];
    for(int i=0;i<len;++i) absdev[i] = fabs(((double)tmp[i]) - med);
    qsort(absdev, len, sizeof(double), cmp_double);
    if(len==1) f.rtt_mad = 0;
    else{
        int lo = (len-1)/2, hi = len/2;
        f.rtt_mad = (absdev[lo]+absdev[hi])*0.5; // 线性插值中位
    }

    // trimmed mean (10%~90%); 若不足10个样本→用普通均值
    if(len>=10){
        int a = (int)(0.10*len), b = (int)(0.90*len);
        if(b<=a) b=a+1;
        double s=0; int c=0;
        for(int i=a;i<b;++i){ s += (double)tmp[i]; ++c; }
        f.rtt_mean_trim = (c>0) ? s/c : (double)tmp[0];
    }else{
        double s=0; for(int i=0;i<len;++i) s+=(double)tmp[i];
        f.rtt_mean_trim = s/len;
    }

    // std (用未裁剪的)
    double xd[MAX_FRAMES_USED];
    for(int i=0;i<len;++i) xd[i] = (double)rtt_ps[i];
    f.rtt_std = std_double(xd, len);

    f.rtt_valid_ratio = (double)len / ((double)EXPECT_FRAMES); // = n / 15.0

    // RSSI
    int32_t rmin = rssi_dbm[0];
    double  rs = 0.0;
    double  rssi_d[MAX_FRAMES_USED];
    for (int i = 0; i < len; ++i) {
        if (rssi_dbm[i] < rmin) rmin = rssi_dbm[i];
        rs += (double)rssi_dbm[i];
        rssi_d[i] = (double)rssi_dbm[i];
    }
    f.rssi_min  = (double)rmin;
    f.rssi_mean = rs / len;
    f.rssi_std  = std_double(rssi_d, len);

    return f;
}

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
        g_ftm_entries    = ev->ftm_report_num_entries;     // ★ 记录条目数

        // ★ 兜底：事件里自带逐帧指针，仅在回调里安全，立刻拷贝一份
        if (ev->ftm_report_num_entries > 0 && ev->ftm_report_data) {
            size_t sz = sizeof(wifi_ftm_report_entry_t) * ev->ftm_report_num_entries;
            // 如果之前有残留，先释放
            if (s_ftm_copy) { free(s_ftm_copy); s_ftm_copy = NULL; s_ftm_copy_num = 0; }
            s_ftm_copy = (wifi_ftm_report_entry_t*)heap_caps_malloc(sz, MALLOC_CAP_8BIT);
            if (s_ftm_copy) {
                memcpy(s_ftm_copy, ev->ftm_report_data, sz);
                s_ftm_copy_num = ev->ftm_report_num_entries;
            }
        }

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

/* static void print_ftm_frames(const wifi_ftm_report_entry_t *rep, uint8_t n, const char *aname) {
    for (uint8_t i = 0; i < n; ++i) {
        const wifi_ftm_report_entry_t *e = &rep[i];
        printf("ftm_frame,%s,idx=%u,dtoken=%u,rssi=%d,rtt_ps=%u\n",
               aname,
               (unsigned)i,
               (unsigned)e->dlog_token,
               (int)e->rssi,
               (unsigned)e->rtt);
    }
} */

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
    g_csi_print_armed = false;
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
    it.rssi = (int8_t)info->rx_ctrl.rssi;

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

    // ★ 先尝试官方 API 抓报告（建议姿势）
    wifi_ftm_report_entry_t *frames = NULL;
    uint8_t nframes = 0;
    esp_err_t grc = ESP_FAIL;

    if ((e & FTM_REPORT_BIT) && g_ftm_entries > 0) {
        nframes = g_ftm_entries;
        size_t sz = sizeof(wifi_ftm_report_entry_t) * nframes;
        frames = (wifi_ftm_report_entry_t*)heap_caps_malloc(sz, MALLOC_CAP_8BIT);
        if (frames) {
            memset(frames, 0, sz);
            grc = esp_wifi_ftm_get_report(frames, nframes);
            if (grc != ESP_OK) {
                free(frames); frames = NULL; nframes = 0;
            }
        }
    }

    // ★ 统一清理（不动你的设计）
    (void)esp_wifi_ftm_end_session();
    vTaskDelay(pdMS_TO_TICKS(FTM_RECYCLE_GAP_MS));
    csi_pause(false);

    g_csi_earliest_us  = esp_timer_get_time() + 30 * 1000;
    g_csi_expected_idx = s_curr_anchor_idx;
    g_csi_print_armed  = true;

    if (e & FTM_FAIL_BIT) return ESP_FAIL;
    if (!(e & FTM_REPORT_BIT)) return ESP_ERR_TIMEOUT;

    // —— 你的会话摘要保留不变 ——
    float est_m = (g_dist_cm > 0) ? (g_dist_cm / 100.0f) : -1.0f;
    float err_m = (g_curr->gt_dist_m > 0 && est_m > 0) ? (est_m - g_curr->gt_dist_m) : NAN;
    printf("anchor,%s," MACSTR ",ch=%u\n", g_curr->name, MAC2STR(g_curr->bssid), (unsigned)cfg.channel);
    printf("ftm_report,%s,%u,%u,%.3f", g_curr->name,
           (unsigned)g_rtt_raw_ns_min, (unsigned)g_rtt_est_ns, est_m);
    if (!isnan(err_m)) printf(",err=%.3f", err_m);
    putchar('\n');

    // ★ 优先用 get_report 的帧；如果没有，且兜底副本有，就用兜底
    if (!frames || nframes == 0) {
        if (s_ftm_copy && s_ftm_copy_num > 0) {
            frames  = s_ftm_copy;
            nframes = s_ftm_copy_num;
            // 用完清掉指针，避免二次 free
            s_ftm_copy = NULL;
            s_ftm_copy_num = 0;
        }
    }

    if (frames && nframes) {
        // 取前 MAX_FRAMES_USED 帧用于统计（目标固定16帧）
        int use = (nframes > MAX_FRAMES_USED) ? MAX_FRAMES_USED : nframes;

        // 收集 ps 数组（保持原始精度）
        uint32_t rtt_ps[MAX_FRAMES_USED];
        int32_t  rssi_dbm[MAX_FRAMES_USED];
        for (int i=0;i<use;++i){
            rtt_ps[i]   = frames[i].rtt;   // 单位：ps
            rssi_dbm[i] = frames[i].rssi;  // dBm
        }

        // 计算特征（输入 ps）
        rtt_feat_t F = make_rtt_feats(rtt_ps, rssi_dbm, use);

        // ★ 输出“板端特征行”：单位=ps（与统计一致）
        printf("ftm_feats,%s,", g_curr->name);
        printf("rtt_ps_min=%.0f,k2=%.0f,median=%.0f,p10=%.0f,p25=%.0f,p75=%.0f,",
               F.rtt_min,F.rtt_k2,F.rtt_median,F.rtt_p10,F.rtt_p25,F.rtt_p75);
        printf("iqr=%.0f,mad=%.0f,mean_trim=%.1f,std=%.1f,valid_ratio=%.3f,",
               F.rtt_iqr,F.rtt_mad,F.rtt_mean_trim,F.rtt_std,F.rtt_valid_ratio);
        printf("rssi_mean=%.2f,rssi_min=%.0f,rssi_std=%.2f,",
               F.rssi_mean, F.rssi_min, F.rssi_std);

        // 固定长度序列（ps），便于训练或排查
        putchar_unlocked('[');
        for(int i=0;i<use;++i){
            if(i) putchar_unlocked(',');
            printf("%u", (unsigned)rtt_ps[i]);
        }
        putchar_unlocked(']');
        putchar_unlocked('\n');

        // 只有在 frames 不是 fallback 时我们才 free（前面已有处理）
        free(frames);
    }
    return ESP_OK;
}

// ===== CSI logger task: print ~1Hz (take latest only) =====
static void csi_logger_task(void *arg) {
    const TickType_t poll = pdMS_TO_TICKS(20); // 快速轮询即可
    csi_item_t it;

    for (;;) {
        if (g_csi_print_armed) {
            bool have = false;
            csi_item_t last;
            while (xQueueReceive(s_csi_q, &it, 0) == pdTRUE) { last = it; have = true; }

            if (have) {
                uint64_t now = esp_timer_get_time();
                if (now >= g_csi_earliest_us && last.anchor_idx == g_csi_expected_idx) {
                    // 只打印“特征行”，不再打印原始 csi_data
                    print_csi_features_from_iq(
                            last.data,
                            last.len,
                            last.rssi,
                            g_anchors[last.anchor_idx].name
                    );
                    g_csi_print_armed = false; // 打一次就解除武装
                }
            }
        } else {
            // 未武装：清空队列，避免积压
            while (xQueueReceive(s_csi_q, &it, 0) == pdTRUE) { /* drop */ }
        }

        vTaskDelay(poll);
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
        vTaskDelay(pdMS_TO_TICKS(200));

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
        vTaskDelay(pdMS_TO_TICKS(300)); // 可按需要调
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
    xTaskCreate(measure_task,   "measure",     8192, NULL, 5, NULL);

    ESP_LOGI(TAG, "Ready (multi-anchor): FTM burst + CSI 1Hz, UART=921600.");
}
