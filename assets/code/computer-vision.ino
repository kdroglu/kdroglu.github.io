#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <esp_camera.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_wifi.h>
#include "driver/ledc.h"
#include <math.h>

const char* AP_SSID     = "RobotXY";
const char* AP_PASSWORD = "mktmkt98";

#define PWDN_GPIO_NUM  -1
#define RESET_GPIO_NUM -1
#define XCLK_GPIO_NUM  10
#define SIOD_GPIO_NUM  40
#define SIOC_GPIO_NUM  39
#define Y9_GPIO_NUM    48
#define Y8_GPIO_NUM    11
#define Y7_GPIO_NUM    12
#define Y6_GPIO_NUM    14
#define Y5_GPIO_NUM    16
#define Y4_GPIO_NUM    18
#define Y3_GPIO_NUM    17
#define Y2_GPIO_NUM    15
#define VSYNC_GPIO_NUM 38
#define HREF_GPIO_NUM  47
#define PCLK_GPIO_NUM  13

#define PAN_PIN            1
#define TILT_PIN           2
#define LASER_PIN          3
#define SERVO_FREQ_HZ      300
#define SERVO_RES_BITS     13
#define SERVO_MIN_US       500
#define SERVO_MAX_US       2500
#define SERVO_LEDC_TIMER   LEDC_TIMER_1
#define PAN_LEDC_CHANNEL   LEDC_CHANNEL_1
#define TILT_LEDC_CHANNEL  LEDC_CHANNEL_2

#define STREAM_FPS          20
#define WIFI_CHANNEL         1
#define MAX_CLIENTS          1
#define MAX_FRAME_BYTES    (60 * 1024)
#define FAIL_RESTART_LIMIT  10
#define CAM_WARMUP_FRAMES    3

AsyncWebServer server(80);
AsyncWebSocket ws("/ws");
AsyncWebSocket ctrl("/ctrl");

volatile bool  cameraActive    = false;
volatile bool  pendingStart    = false;
volatile bool  pendingStop     = false;
volatile bool  camStateChanged = false;

TaskHandle_t cameraTaskHandle = NULL;

static uint32_t diagLastMs    = 0;
static uint32_t diagBytesSent = 0;
static uint32_t diagByteMark  = 0;
static uint32_t diagDrops     = 0;
static uint32_t diagRestarts  = 0;

static void pushCamState() {
  if (ws.count() == 0) return;
  char buf[56];
  snprintf(buf, sizeof(buf), "{\"type\":\"diag\",\"camActive\":%s}",
    cameraActive ? "true" : "false");
  ws.textAll(buf);
}
void sendDiag() {
  if (camStateChanged) { camStateChanged = false; pushCamState(); }
  uint32_t now = millis();
  if (now - diagLastMs < 5000) return;
  float kbps = (diagBytesSent - diagByteMark) * 8.0f / (now - diagLastMs);
  Serial.printf("[DIAG] %.0f kbps atla:%u yb:%u v:%u c:%u\n",
    kbps, diagDrops, diagRestarts,
    (uint32_t)ws.count(), (uint32_t)ctrl.count());
  diagByteMark = diagBytesSent;
  diagLastMs   = now;
}

// ── Servo ─────────────────────────────────────────────────────────────────────
static uint32_t angleToDuty(float deg) {
  if (deg < 0.0f)   deg = 0.0f;
  if (deg > 180.0f) deg = 180.0f;
  const float    period_us = 1000000.0f / (float)SERVO_FREQ_HZ;
  const uint32_t maxDuty   = (1u << SERVO_RES_BITS);
  float us = SERVO_MIN_US + (deg / 180.0f) * (SERVO_MAX_US - SERVO_MIN_US);
  return (uint32_t)((us / period_us) * (float)maxDuty + 0.5f);
}
void servoInit() {
  ledc_timer_config_t t = {};
  t.speed_mode      = LEDC_LOW_SPEED_MODE;
  t.duty_resolution = (ledc_timer_bit_t)SERVO_RES_BITS;
  t.timer_num       = SERVO_LEDC_TIMER;
  t.freq_hz         = SERVO_FREQ_HZ;
  t.clk_cfg         = LEDC_AUTO_CLK;
  ledc_timer_config(&t);
  ledc_channel_config_t ch = {};
  ch.speed_mode = LEDC_LOW_SPEED_MODE;
  ch.timer_sel  = SERVO_LEDC_TIMER;
  ch.intr_type  = LEDC_INTR_DISABLE;
  ch.duty       = angleToDuty(90.0f);
  ch.hpoint     = 0;
  ch.gpio_num = PAN_PIN;  ch.channel = PAN_LEDC_CHANNEL;  ledc_channel_config(&ch);
  ch.gpio_num = TILT_PIN; ch.channel = TILT_LEDC_CHANNEL; ledc_channel_config(&ch);
  Serial.printf("[SERVO] freq:%d Hz res:%d bit PAN:GPIO%d TILT:GPIO%d\n",
    SERVO_FREQ_HZ, SERVO_RES_BITS, PAN_PIN, TILT_PIN);
}
static inline void setServoDirect(ledc_channel_t ch, float deg) {
  ledc_set_duty(LEDC_LOW_SPEED_MODE, ch, angleToDuty(deg));
  ledc_update_duty(LEDC_LOW_SPEED_MODE, ch);
}

// ── Kamera ────────────────────────────────────────────────────────────────────
bool startCamera() {
  camera_config_t c;
  c.ledc_channel = LEDC_CHANNEL_0; c.ledc_timer = LEDC_TIMER_0;
  c.pin_d0=Y2_GPIO_NUM; c.pin_d1=Y3_GPIO_NUM; c.pin_d2=Y4_GPIO_NUM; c.pin_d3=Y5_GPIO_NUM;
  c.pin_d4=Y6_GPIO_NUM; c.pin_d5=Y7_GPIO_NUM; c.pin_d6=Y8_GPIO_NUM; c.pin_d7=Y9_GPIO_NUM;
  c.pin_xclk=XCLK_GPIO_NUM; c.pin_pclk=PCLK_GPIO_NUM;
  c.pin_vsync=VSYNC_GPIO_NUM; c.pin_href=HREF_GPIO_NUM;
  c.pin_sccb_sda=SIOD_GPIO_NUM; c.pin_sccb_scl=SIOC_GPIO_NUM;
  c.pin_pwdn=PWDN_GPIO_NUM; c.pin_reset=RESET_GPIO_NUM;
  c.xclk_freq_hz = 20000000; c.pixel_format = PIXFORMAT_JPEG;
  c.fb_location  = CAMERA_FB_IN_PSRAM;
  c.frame_size   = FRAMESIZE_SVGA; c.jpeg_quality = 12;
  c.fb_count     = 2; c.grab_mode = CAMERA_GRAB_LATEST;
  if (esp_camera_init(&c) != ESP_OK) {
    Serial.println("[CAM] Init başarısız!"); return false;
  }
  sensor_t *s = esp_camera_sensor_get();
  s->set_vflip(s,0); s->set_hmirror(s,1); s->set_brightness(s,0);
  s->set_saturation(s,0); s->set_contrast(s,0);
  s->set_awb_gain(s,1); s->set_wb_mode(s,0);
  for (int i = 0; i < CAM_WARMUP_FRAMES; i++) {
    camera_fb_t *fb = esp_camera_fb_get();
    if (fb) esp_camera_fb_return(fb);
    vTaskDelay(pdMS_TO_TICKS(50));
  }
  Serial.println("[CAM] Hazır."); return true;
}

static AsyncWebSocketClient* getActiveVideoClient() {
  for (auto &c : ws.getClients())
    if (c.status() == WS_CONNECTED) return &c;
  return nullptr;
}

void cameraTask(void *pv) {
  TickType_t lastWake = xTaskGetTickCount();
  int failStreak = 0;
  while (true) {
    static uint8_t   softStartRemain = 0;
    const uint8_t    SOFT_START_N    = 3;
    const TickType_t SOFT_PERIOD     = pdMS_TO_TICKS(100);
    const TickType_t FULL_PERIOD     = pdMS_TO_TICKS(1000 / STREAM_FPS);

    if (pendingStart) {
      pendingStart = false;
      if (!cameraActive) { cameraActive = startCamera(); camStateChanged = true; }
      softStartRemain = SOFT_START_N;
      lastWake = xTaskGetTickCount();
    }
    if (pendingStop) {
      pendingStop = false;
      if (cameraActive) {
        cameraActive = false; camStateChanged = true;
        vTaskDelay(pdMS_TO_TICKS(60)); esp_camera_deinit(); failStreak = 0;
      }
      lastWake = xTaskGetTickCount();
    }
    if (!cameraActive) {
      vTaskDelay(pdMS_TO_TICKS(50)); lastWake = xTaskGetTickCount(); continue;
    }
    AsyncWebSocketClient *client = getActiveVideoClient();
    if (!client) {
      camera_fb_t *fb = esp_camera_fb_get(); if (fb) esp_camera_fb_return(fb);
      vTaskDelay(pdMS_TO_TICKS(50)); lastWake = xTaskGetTickCount(); continue;
    }
    if (client->queueIsFull()) {
      camera_fb_t *fb = esp_camera_fb_get();
      if (fb) esp_camera_fb_return(fb);
      diagDrops++;
      vTaskDelay(pdMS_TO_TICKS(40));
      lastWake = xTaskGetTickCount();
      continue;
    }
    TickType_t period = (softStartRemain > 0) ? SOFT_PERIOD : FULL_PERIOD;
    camera_fb_t *fb = esp_camera_fb_get();
    if (!fb) {
      failStreak++;
      if (failStreak >= FAIL_RESTART_LIMIT) {
        cameraActive = false; camStateChanged = true; esp_camera_deinit();
        vTaskDelay(pdMS_TO_TICKS(300));
        cameraActive = startCamera(); camStateChanged = true;
        failStreak = 0; diagRestarts++; lastWake = xTaskGetTickCount();
      } else { vTaskDelay(pdMS_TO_TICKS(20)); lastWake = xTaskGetTickCount(); }
      continue;
    }
    failStreak = 0;
    if (fb->len == 0 || fb->len > MAX_FRAME_BYTES) {
      esp_camera_fb_return(fb); vTaskDelayUntil(&lastWake, period); continue;
    }
    client->binary(fb->buf, (size_t)fb->len);
    diagBytesSent += fb->len;
    esp_camera_fb_return(fb);
    if (softStartRemain > 0) softStartRemain--;
    TickType_t now = xTaskGetTickCount();
    if ((now - lastWake) > period) lastWake = now - period;
    vTaskDelayUntil(&lastWake, period);
  }
}

// ═══════════════════════════════════════════════════════════════════════════════
// HTML
// ═══════════════════════════════════════════════════════════════════════════════
const char* HTML_PAGE = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
  <meta charset="UTF-8">
  <meta name="viewport"
    content="width=device-width,initial-scale=1,maximum-scale=1,user-scalable=no,viewport-fit=cover">
  <title>XIAO ESP32S3</title>
  <style>
    *,*::before,*::after{box-sizing:border-box;margin:0;padding:0}
    :root{
      --bg:#06080f;--surface:#0d1117;--s2:#161b22;
      --border:#30363d;--border2:#21262d;
      --accent:#4cc9f0;--green:#06d6a0;--red:#f85149;--yellow:#e3b341;
      --t1:#e6edf3;--t2:#8b949e;--t3:#484f58;--r:8px;
      --safe-b:env(safe-area-inset-bottom,0px);
      --safe-l:env(safe-area-inset-left,0px);
      --safe-r:env(safe-area-inset-right,0px);
    }
    html{height:100%;background:var(--bg)}
    body{background:var(--bg);display:flex;flex-direction:column;align-items:center;
      min-height:100%;
      padding:10px max(8px,var(--safe-r)) max(20px,var(--safe-b)) max(8px,var(--safe-l));
      font-family:'Segoe UI',system-ui,sans-serif;color:var(--t1);gap:8px;overflow-x:hidden}
    .top-bar{width:100%;max-width:820px;display:flex;align-items:center;
      justify-content:space-between;gap:8px;flex-shrink:0}
    h1{font-size:.95rem;color:var(--accent);letter-spacing:2px;white-space:nowrap}
    .conn-badge{display:flex;align-items:center;gap:5px;font-size:.72rem;color:var(--t2);
      padding:4px 10px;background:rgba(255,255,255,.04);
      border:1px solid var(--border2);border-radius:20px}
    .conn-dot{width:8px;height:8px;border-radius:50%;background:var(--t3);
      transition:background .3s,box-shadow .3s;flex-shrink:0}
    .conn-dot.ok  {background:var(--green);box-shadow:0 0 8px var(--green)}
    .conn-dot.warn{background:var(--yellow);box-shadow:0 0 6px var(--yellow)}
    .video-outer{width:100%;max-width:820px;flex-shrink:0}
    .video-wrap{position:relative;width:100%;padding-top:75%;
      background:#08090e;border:2px solid var(--border);
      border-radius:var(--r);overflow:hidden;transition:border-color .3s}
    .video-wrap.streaming{border-color:var(--accent)}
    .video-inner{position:absolute;inset:0}
    #canvas   {display:block;width:100%;height:100%;position:absolute;inset:0}
    #detCanvas{display:block;width:100%;height:100%;position:absolute;inset:0;pointer-events:none}
    #offMsg{position:absolute;inset:0;display:flex;align-items:center;justify-content:center;
      font-size:1.1rem;color:var(--t3);letter-spacing:2px;background:#08090e;pointer-events:none}
    .overlay-top{position:absolute;top:8px;right:8px;left:8px;
      display:flex;justify-content:space-between;align-items:flex-start;pointer-events:none}
    .b-fps{color:rgba(76,201,240,.7);font-size:.74rem;font-weight:700;
      text-shadow:0 1px 4px rgba(0,0,0,.9);pointer-events:none}
    .btn-stream{pointer-events:all;border:1px solid rgba(255,255,255,.22);border-radius:20px;
      padding:5px 16px;font-size:.8rem;font-weight:700;cursor:pointer;
      background:rgba(0,0,0,.58);backdrop-filter:blur(6px);transition:all .15s}
    .btn-stream.start{color:var(--green)}.btn-stream.stop{color:var(--red)}
    .btn-stream:disabled{opacity:.35;cursor:not-allowed}
    .det-row{width:100%;max-width:820px;display:flex;align-items:center;
      gap:8px;flex-shrink:0;flex-wrap:wrap}
    .det-status{display:flex;align-items:center;gap:6px;font-size:.72rem;
      padding:5px 12px;border-radius:20px;border:1px solid var(--border2);
      background:rgba(255,255,255,.03);color:var(--t2);flex-shrink:0;white-space:nowrap}
    .det-dot{width:7px;height:7px;border-radius:50%;background:var(--t3);
      flex-shrink:0;transition:background .3s,box-shadow .3s}
    .det-dot.loading{background:var(--yellow);animation:pulse 1.1s ease-in-out infinite}
    .det-dot.ready  {background:var(--green);box-shadow:0 0 6px var(--green)}
    .det-dot.active {background:var(--red);box-shadow:0 0 6px var(--red);
      animation:pulse 1.1s ease-in-out infinite}
    .det-dot.error  {background:var(--red)}
    @keyframes pulse{0%,100%{opacity:1}50%{opacity:.35}}
    .btn-laser{border:1px solid var(--border);background:var(--s2);color:var(--t2);
      border-radius:20px;padding:5px 10px;font-size:.78rem;font-weight:700;
      cursor:pointer;transition:all .15s;white-space:nowrap}
    .btn-laser.on{border-color:var(--yellow);color:var(--yellow);
      box-shadow:0 0 8px rgba(227,179,65,.4)}
    .det-fps{font-size:.68rem;color:var(--t3);margin-left:auto;white-space:nowrap;
      font-variant-numeric:tabular-nums}
    .joy-block{width:100%;max-width:820px;flex-shrink:0;
      background:var(--surface);border:1px solid var(--border2);
      border-radius:var(--r);padding:16px;
      display:flex;justify-content:center;position:relative}
    .joy-angles{position:absolute;top:10px;right:12px;
      display:flex;flex-direction:column;gap:3px;align-items:flex-end}
    .joy-angle-row{display:flex;align-items:baseline;gap:4px}
    .joy-angle-ax {font-size:.6rem;font-weight:700;color:var(--t3);letter-spacing:.5px}
    .joy-angle-val{font-size:.78rem;font-weight:600;color:var(--t1);
      font-variant-numeric:tabular-nums;min-width:40px;text-align:right}
    .joy-wrap{position:relative;width:220px;height:220px;border-radius:50%;
      background:var(--s2);border:1.5px solid var(--border);
      cursor:crosshair;touch-action:none;user-select:none;-webkit-user-select:none;flex-shrink:0}
    .joy-line{position:absolute;background:rgba(255,255,255,.06);pointer-events:none}
    .joy-line-h{top:50%;left:10%;width:80%;height:1px;transform:translateY(-50%)}
    .joy-line-v{left:50%;top:10%;height:80%;width:1px;transform:translateX(-50%)}
    .joy-inner{position:absolute;width:50%;height:50%;border-radius:50%;
      border:1px dashed rgba(255,255,255,.07);top:25%;left:25%;pointer-events:none}
    .joy-center{position:absolute;width:6px;height:6px;border-radius:50%;
      background:rgba(255,255,255,.18);top:50%;left:50%;
      transform:translate(-50%,-50%);pointer-events:none}
    .joy-dir{position:absolute;font-size:.58rem;font-weight:700;
      letter-spacing:.5px;color:rgba(255,255,255,.22);pointer-events:none}
    .joy-dir-n{top:6px; left:50%;transform:translateX(-50%)}
    .joy-dir-s{bottom:6px;left:50%;transform:translateX(-50%)}
    .joy-dir-w{left:7px; top:50%;transform:translateY(-50%)}
    .joy-dir-e{right:7px;top:50%;transform:translateY(-50%)}
    .joy-knob{position:absolute;width:44px;height:44px;border-radius:50%;
      background:radial-gradient(circle at 38% 35%,
        rgba(76,201,240,.95) 0%,rgba(30,130,175,1) 100%);
      border:2px solid rgba(255,255,255,.18);
      box-shadow:0 0 0 3px rgba(76,201,240,.18),0 4px 16px rgba(0,0,0,.55);
      top:calc(50% - 22px);left:calc(50% - 22px);
      transform:translate(0,0);pointer-events:none;will-change:transform}
    .joy-wrap.active .joy-knob{
      box-shadow:0 0 0 6px rgba(76,201,240,.28),0 4px 20px rgba(0,0,0,.6)}
    #crosshair{position:absolute;top:50%;left:50%;
      transform:translate(-50%,-50%);
      width:30px;height:30px;pointer-events:none;z-index:10}
    #crosshair.hit::before,#crosshair.hit::after{background:rgba(6,214,160,.9)!important}
    #crosshair.hit .ch-dot{background:rgba(6,214,160,1)!important}
    #crosshair::before,#crosshair::after{
      content:'';position:absolute;background:rgba(220,30,30,.72);border-radius:1px}
    #crosshair::before{top:50%;left:0;width:100%;height:1.5px;transform:translateY(-50%)}
    #crosshair::after {left:50%;top:0;height:100%;width:1.5px;transform:translateX(-50%)}
    .ch-dot{position:absolute;top:50%;left:50%;transform:translate(-50%,-50%);
      width:4px;height:4px;border-radius:50%;background:rgba(220,30,30,.9)}
    .btn-track{border:1px solid var(--border);background:var(--s2);color:var(--t2);
      border-radius:20px;padding:5px 10px;font-size:.78rem;font-weight:700;
      cursor:pointer;transition:all .15s;white-space:nowrap}
    .btn-track.on{border-color:#a78bfa;color:#a78bfa;
      box-shadow:0 0 8px rgba(167,139,250,.35)}
    .btn-lj{border:1px solid var(--border);background:var(--s2);color:var(--t2);
      border-radius:20px;padding:5px 10px;font-size:.78rem;font-weight:700;
      cursor:pointer;transition:all .15s;white-space:nowrap}
    .btn-lj.running{border-color:var(--red);color:var(--red);
      box-shadow:0 0 8px rgba(248,81,73,.35);animation:pulse 1.1s ease-in-out infinite}
    .lj-wrap{position:relative}
    /* ── LJ menüsü: aşağı açılır, ekranda kalır ── */
    #ljMenu{position:absolute;bottom:calc(100% + 6px);right:0;
      background:#0d1117;border:1px solid #30363d;border-radius:9px;
      padding:6px;display:none;flex-direction:column;gap:4px;z-index:300;
      box-shadow:0 8px 24px rgba(0,0,0,.8);min-width:170px;
      max-height:calc(100vh - 120px);overflow-y:auto}
    #ljMenu.open{display:flex}
    .lj-sec{font-size:.57rem;font-weight:700;letter-spacing:.8px;color:var(--t3);
      text-transform:uppercase;padding-bottom:2px;border-bottom:1px solid var(--border2)}
    .lj-row{display:flex;gap:3px;flex-wrap:wrap}
    .lj-opt{background:none;border:1px solid #21262d;border-radius:6px;
      color:#8b949e;font-size:.67rem;font-weight:700;padding:3px 7px;
      cursor:pointer;transition:all .12s;white-space:nowrap}
    .lj-opt:hover{border-color:var(--accent);color:var(--accent)}
    .lj-size-row{display:flex;align-items:center;gap:5px}
    .lj-size-lbl{font-size:.57rem;color:var(--t3);white-space:nowrap;flex-shrink:0}
    .lj-slider{flex:1;-webkit-appearance:none;appearance:none;height:3px;
      border-radius:2px;outline:none;cursor:pointer;
      background:linear-gradient(to right,var(--accent) 0%,var(--accent) var(--pct,37%),#21262d var(--pct,37%),#21262d 100%)}
    .lj-slider::-webkit-slider-thumb{-webkit-appearance:none;width:10px;height:10px;
      border-radius:50%;background:var(--accent);cursor:pointer;border:2px solid #0d1117}
    .lj-size-val{font-size:.63rem;color:var(--t1);min-width:24px;text-align:right;
      font-variant-numeric:tabular-nums;flex-shrink:0}
    #settingsOverlay{position:fixed;inset:0;background:rgba(0,0,0,.45);
      z-index:200;opacity:0;pointer-events:none;transition:opacity .25s}
    #settingsOverlay.open{opacity:1;pointer-events:all}
    #settingsPanel{position:fixed;top:0;right:-320px;width:310px;height:100%;
      background:#0d1117;border-left:1px solid #30363d;
      z-index:201;overflow-y:auto;padding:16px 16px 40px;
      transition:right .25s cubic-bezier(.4,0,.2,1)}
    #settingsPanel.open{right:0}
    .pid-header{display:flex;align-items:center;justify-content:space-between;margin-bottom:16px}
    .pid-title{font-size:.9rem;font-weight:700;color:#4cc9f0;letter-spacing:1px}
    .pid-close{background:none;border:none;color:#8b949e;font-size:1.2rem;
      cursor:pointer;padding:4px 8px;border-radius:6px}
    .pid-close:hover{color:#e6edf3}
    .btn-pid{background:none;border:1px solid #30363d;border-radius:8px;
      color:#8b949e;font-size:.8rem;cursor:pointer;padding:4px 9px;
      transition:all .15s;white-space:nowrap}
    .btn-pid:hover{border-color:#4cc9f0;color:#4cc9f0}
    #trackMenu{position:absolute;bottom:calc(100% + 6px);right:0;
      background:#0d1117;border:1px solid #30363d;border-radius:10px;
      padding:6px;display:none;flex-direction:column;gap:4px;z-index:300;
      box-shadow:0 8px 24px rgba(0,0,0,.7);min-width:120px}
    #trackMenu.open{display:flex}
    .track-opt{background:none;border:1px solid #21262d;border-radius:7px;
      color:#8b949e;font-size:.76rem;font-weight:700;padding:6px 12px;
      cursor:pointer;text-align:left;transition:all .12s;white-space:nowrap}
    .track-opt:hover{border-color:#a78bfa;color:#a78bfa}
    .track-opt.sel{border-color:#a78bfa;color:#a78bfa;background:rgba(167,139,250,.12)}
    .track-wrap{position:relative}
    .toast{position:fixed;bottom:max(28px,var(--safe-b));left:50%;
      transform:translateX(-50%) translateY(10px);padding:8px 20px;
      border-radius:22px;font-size:.8rem;font-weight:600;z-index:400;
      opacity:0;transition:opacity .2s,transform .2s;pointer-events:none;white-space:nowrap}
    .toast.ok  {background:var(--green);color:#0a0a0f}
    .toast.err {background:var(--red);color:#fff}
    .toast.warn{background:var(--yellow);color:#0a0a0f}
    .toast.show{opacity:1;transform:translateX(-50%) translateY(0)}
  </style>
</head>
<body>

<div class="top-bar">
  <h1>📷 XIAO</h1>
  <div style="display:flex;align-items:center;gap:8px">
    <button class="btn-pid" onclick="toggleSettingsPanel()">⚙ Ayarlar</button>
    <div class="conn-badge">
      <div class="conn-dot" id="connDot"></div>
      <span id="connLabel">Bağlanıyor…</span>
    </div>
  </div>
</div>

<div class="video-outer">
  <div class="video-wrap" id="videoWrap">
    <div class="video-inner" id="videoInner" style="touch-action:none">
      <canvas id="canvas"    width="800" height="600"></canvas>
      <canvas id="detCanvas" width="800" height="600"></canvas>
      <canvas id="touchCanvas" width="800" height="600"
        style="position:absolute;inset:0;width:100%;height:100%;pointer-events:none"></canvas>
      <div id="crosshair"><div class="ch-dot"></div></div>
      <div id="offMsg">⏸ Kamera kapalı</div>
      <div class="overlay-top">
        <button id="streamBtn" class="btn-stream start" disabled
                onclick="toggleStream()">▶ Başlat</button>
        <span class="b-fps" id="fpsBadge">– fps</span>
      </div>
    </div>
  </div>
</div>

<div class="det-row">
  <div class="det-status">
    <div class="det-dot" id="detDot"></div>
    <span id="detLabel">Tespite hazır</span>
  </div>
  <button id="laserBtn" class="btn-laser" onclick="toggleLaser()">🔴 Lazer</button>
  <div class="track-wrap">
    <button id="trackBtn" class="btn-track" onclick="openTrackMenu()">🎯 Takip</button>
    <div id="trackMenu">
      <button class="track-opt sel" id="opt-object" onclick="selectTrackTarget('object')">📦 Obje</button>
      <button class="track-opt"    id="opt-face"   onclick="selectTrackTarget('face')">😊 Yüz</button>
      <div id="objInputWrap" style="display:flex;gap:4px;margin-top:2px">
        <input id="objInput" type="text" placeholder="önce model seçin" autocomplete="off"
          disabled
          style="flex:1;background:#161b22;border:1px solid #30363d;border-radius:7px;
                 color:#e6edf3;font-size:.76rem;padding:5px 8px;outline:none;min-width:0;
                 opacity:.4;transition:opacity .2s"
          onfocus="this.style.borderColor='#4cc9f0'"
          onblur="this.style.borderColor='#30363d'"
          onkeydown="if(event.key==='Enter'){event.stopPropagation();startObjectTracking();}">
        <button id="objGoBtn" onclick="startObjectTracking()" disabled
          style="background:#4cc9f0;color:#06080f;border:none;border-radius:7px;
                 font-size:.76rem;font-weight:700;padding:5px 10px;cursor:pointer;
                 white-space:nowrap;opacity:.4;transition:opacity .2s">▶</button>
      </div>
    </div>
  </div>
  <div class="lj-wrap">
    <button id="ljBtn" class="btn-lj" onclick="toggleLjMenu()">✦ LJ</button>
    <div id="ljMenu">
      <div style="display:flex;align-items:center;margin-bottom:2px">
        <span class="lj-sec" style="border-bottom:none;padding-bottom:0">✦ LJ</span>
      </div>
      <div class="lj-sec">⚡ Lazer Show</div>
      <div class="lj-row">
        <button class="lj-opt" onclick="runLaserShape('square')">▣ Kare</button>
        <button class="lj-opt" onclick="runLaserShape('circle')">○ Daire</button>
        <button class="lj-opt" onclick="runLaserShape('triangle')">△ Üçgen</button>
        <button class="lj-opt" onclick="runLaserShape('infinity')">∞ Sonsuz</button>
      </div>
      <div class="lj-size-row">
        <span class="lj-size-lbl">Boyut</span>
        <input class="lj-slider" type="range" id="ljSize" min="5" max="45" step="1" value="20"
               oninput="onLjSlider(this)">
        <span class="lj-size-val" id="ljSizeVal">20°</span>
      </div>
      <div class="lj-size-row">
        <span class="lj-size-lbl">Hız</span>
        <input class="lj-slider" type="range" id="ljSpeed" min="1" max="20" step="1" value="16"
               oninput="onLjSpeedSlider(this)">
        <span class="lj-size-val" id="ljSpeedVal">5ms</span>
      </div>
      <div class="lj-sec" style="margin-top:2px">🤖 Jestler</div>
      <div class="lj-row">
        <button class="lj-opt" onclick="runGesture('evet')">✓ Evet</button>
        <button class="lj-opt" onclick="runGesture('hayir')">✗ Hayır</button>
        <button class="lj-opt" onclick="runGesture('kusme')">😔 Küsme</button>
        <button class="lj-opt" onclick="runGesture('korku')">😱 Korku</button>
      </div>
      <div class="lj-row">
        <button class="lj-opt" onclick="runGesture('uyanis')">👁 Uyanış</button>
        <button class="lj-opt" onclick="runGesture('uyku')">😴 Uyku</button>
        <button class="lj-opt" onclick="runGesture('spiral')">◎ Spiral</button>
      </div>
      <div class="lj-row">
        <button class="lj-opt" onclick="runGesture('dans')">♪ Dans 1</button>
        <button class="lj-opt" onclick="runGesture('dans2')">♫ Dans 2</button>
        <button class="lj-opt" onclick="runGesture('dans3')">🎵 Dans 3</button>
      </div>
      <div style="display:flex;justify-content:flex-end;margin-top:4px">
        <button onclick="document.getElementById('ljMenu').classList.remove('open')"
          style="background:none;border:1px solid #30363d;border-radius:7px;
                 color:#484f58;font-size:.72rem;font-weight:700;cursor:pointer;
                 padding:4px 12px;transition:all .15s"
          onmouseover="this.style.borderColor='#8b949e';this.style.color='#8b949e'"
          onmouseout="this.style.borderColor='#30363d';this.style.color='#484f58'">✕</button>
      </div>
    </div>
  </div>
  <span class="det-fps" id="detFpsLabel"></span>
</div>

<div id="settingsOverlay" onclick="toggleSettingsPanel()"></div>
<div id="settingsPanel">
  <div class="pid-header">
    <span class="pid-title">⚙ AYARLAR</span>
    <button class="pid-close" onclick="toggleSettingsPanel()">✕</button>
  </div>
</div>

<div class="joy-block">
  <div class="joy-angles">
    <div class="joy-angle-row">
      <span class="joy-angle-ax">P</span>
      <span class="joy-angle-val" id="panVal">90.0°</span>
    </div>
    <div class="joy-angle-row">
      <span class="joy-angle-ax">T</span>
      <span class="joy-angle-val" id="tiltVal">90.0°</span>
    </div>
  </div>
  <div class="joy-wrap" id="joyWrap">
    <div class="joy-line joy-line-h"></div>
    <div class="joy-line joy-line-v"></div>
    <div class="joy-inner"></div>
    <div class="joy-center"></div>
    <span class="joy-dir joy-dir-n">T+</span>
    <span class="joy-dir joy-dir-s">T−</span>
    <span class="joy-dir joy-dir-w">P−</span>
    <span class="joy-dir joy-dir-e">P+</span>
    <div class="joy-knob" id="joyKnob"></div>
  </div>
</div>

<div class="toast" id="toast"></div>

<script>
const WORKER_SRC = `
self.onmessage = async (e) => {
  const msg = e.data;
  if (msg.type === 'init') {
    const modelType = msg.modelType;
    try {
      importScripts('https://cdn.jsdelivr.net/npm/@tensorflow/tfjs@4.20.0/dist/tf.min.js');
      let usedBackend = 'cpu';
      for (const b of ['webgl', 'wasm', 'cpu']) {
        try { await tf.setBackend(b); await tf.ready(); usedBackend = b; break; } catch(_) {}
      }
      tf.enableProdMode();
      if (modelType === 'person') {
        importScripts('https://cdn.jsdelivr.net/npm/@tensorflow-models/coco-ssd@2.2.3/dist/coco-ssd.min.js');
        self._det = await cocoSsd.load({ base: 'lite_mobilenet_v2' });
        const wc = new OffscreenCanvas(256, 192);
        const wctx = wc.getContext('2d');
        wctx.fillStyle = 'rgb(80,80,80)'; wctx.fillRect(0,0,256,192);
        const wid = wctx.getImageData(0,0,256,192);
        await self._det.detect(wid); await self._det.detect(wid);
      } else if (modelType === 'face') {
        importScripts('https://cdn.jsdelivr.net/npm/@tensorflow-models/blazeface@0.0.7/dist/blazeface.min.js');
        self._det = await blazeface.load({ maxFaces: 5 });
        const wc = new OffscreenCanvas(256, 192);
        await self._det.estimateFaces(wc, false); await self._det.estimateFaces(wc, false);
      }
      self._modelType = modelType;
      postMessage({ type: 'ready', backend: usedBackend, modelType });
    } catch(err) { postMessage({ type: 'error', msg: String(err), modelType }); }
    return;
  }
  if (msg.type === 'setFilter') { self._filterClasses = msg.filterClasses; return; }
  if (msg.type === 'detect') {
    const bmp = msg.bitmap; const bw = bmp.width, bh = bmp.height;
    try {
      let results = [];
      if (self._modelType === 'person') {
        const oc = new OffscreenCanvas(bw, bh);
        oc.getContext('2d').drawImage(bmp, 0, 0); bmp.close();
        const imageData = oc.getContext('2d').getImageData(0,0,bw,bh);
        const preds = await self._det.detect(imageData);
        const filterAll = !self._filterClasses || self._filterClasses.length === 0;
        for (const p of preds) {
          if (p.score < 0.35) continue;
          if (!filterAll && !self._filterClasses.includes(p.class)) continue;
          results.push({ class: p.class, score: p.score,
            bbox: [p.bbox[0]/bw, p.bbox[1]/bh, p.bbox[2]/bw, p.bbox[3]/bh] });
        }
      } else if (self._modelType === 'face') {
        const raw = await self._det.estimateFaces(bmp, false); bmp.close();
        for (const r of raw) {
          const x0=r.topLeft[0], y0=r.topLeft[1];
          const w=r.bottomRight[0]-x0, h=r.bottomRight[1]-y0;
          results.push({ class:'face', score:r.probability?.[0]??0.9,
            bbox:[x0/bw, y0/bh, w/bw, h/bh] });
        }
      }
      postMessage({ type: 'detections', data: results });
    } catch(_) { try{bmp.close();}catch(__){} postMessage({type:'detections',data:[]}); }
  }
};
`;

(() => {
  'use strict';

  const canvas      = document.getElementById('canvas');
  const ctx         = canvas.getContext('2d', { alpha: false });
  const detCanvas   = document.getElementById('detCanvas');
  const detCtx      = detCanvas.getContext('2d');
  const connDot     = document.getElementById('connDot');
  const connLabel   = document.getElementById('connLabel');
  const streamBtn   = document.getElementById('streamBtn');
  const videoWrap   = document.getElementById('videoWrap');
  const offMsg      = document.getElementById('offMsg');
  const fpsBadge    = document.getElementById('fpsBadge');
  const joyWrap     = document.getElementById('joyWrap');
  const joyKnob     = document.getElementById('joyKnob');
  const panValEl    = document.getElementById('panVal');
  const tiltValEl   = document.getElementById('tiltVal');
  const detDot      = document.getElementById('detDot');
  const detLabel    = document.getElementById('detLabel');
  const detFpsLabel = document.getElementById('detFpsLabel');
  const laserBtn    = document.getElementById('laserBtn');
  const trackBtn    = document.getElementById('trackBtn');
  const crosshairEl = document.getElementById('crosshair');

  let wsConnected=false, camOn=false, userStarted=false;
  let vidSock=null, ctrlSock=null;
  let vidRTimer=null, ctrlRTimer=null;
  let vidDelay=500, ctrlDelay=500;
  const DET_W=256, DET_H=192;

  let fCnt=0, fTime=performance.now();
  setInterval(()=>{
    const dt=(performance.now()-fTime)/1000;
    fpsBadge.textContent=(dt>0?Math.round(fCnt/dt):0)+' fps';
    fCnt=0; fTime=performance.now();
  },1000);

  let toastT=null;
  function toast(msg,type='ok'){
    const el=document.getElementById('toast');
    el.textContent=msg; el.className='toast '+type+' show';
    clearTimeout(toastT);
    toastT=setTimeout(()=>el.className='toast '+type,2800);
  }
  function clamp(v,lo,hi){return v<lo?lo:v>hi?hi:v;}
  function applyState(){
    streamBtn.disabled=!wsConnected;
    videoWrap.className='video-wrap'+(camOn?' streaming':'');
    offMsg.style.display=camOn?'none':'flex';
    streamBtn.textContent=camOn?'⏹ Durdur':'▶ Başlat';
    streamBtn.className='btn-stream '+(camOn?'stop':'start');
  }

  let currentPan=90.0, currentTilt=90.0;
  let pendingPan=null, pendingTilt=null;
  setInterval(()=>{
    if(pendingPan===null||!ctrlSock||ctrlSock.readyState!==WebSocket.OPEN)return;
    ctrlSock.send('pan:'+pendingPan.toFixed(1));
    ctrlSock.send('tilt:'+pendingTilt.toFixed(1));
    pendingPan=null; pendingTilt=null;
  },3);
  function sendServoTarget(pan,tilt){
    pan=clamp(pan,0,180); tilt=clamp(tilt,0,180);
    currentPan=pan; currentTilt=tilt;
    panValEl.textContent=pan.toFixed(1)+'°';
    tiltValEl.textContent=tilt.toFixed(1)+'°';
    pendingPan=pan; pendingTilt=tilt;
  }

  let laserOn=false;
  window.toggleLaser=()=>{
    if(!ctrlSock||ctrlSock.readyState!==WebSocket.OPEN){toast('Kontrol bağlantısı yok','err');return;}
    laserOn=!laserOn;
    ctrlSock.send('laser:'+(laserOn?'1':'0'));
    laserBtn.classList.toggle('on',laserOn);
    toast('Lazer '+(laserOn?'açıldı':'kapatıldı'),'ok');
  };

  class EMAFilter{
    constructor(a){this.alpha=a;this.state=null;}
    reset(){this.state=null;}
    filter(x){if(this.state===null){this.state=x;return x;}this.state=this.alpha*x+(1-this.alpha)*this.state;return this.state;}
  }
  class PIDController{
    constructor(kp,ki,kd,oMin,oMax){this.kp=kp;this.ki=ki;this.kd=kd;this.outMin=oMin;this.outMax=oMax;this.N=30;this.integral=0;this.prevErr=0;this.dFiltered=0;this.lastTime=null;}
    reset(){this.integral=0;this.prevErr=0;this.dFiltered=0;this.lastTime=null;}
    compute(err){
      const now=performance.now();
      if(this.lastTime===null){this.lastTime=now;this.prevErr=err;return 0;}
      const dt=(now-this.lastTime)/1000; this.lastTime=now;
      if(dt<=0||dt>0.5){this.lastTime=now;return 0;}
      this.integral+=err*dt;
      const iL=this.ki>1e-9?Math.abs(this.outMax)/this.ki:1e6;
      this.integral=Math.max(-iL,Math.min(iL,this.integral));
      this.dFiltered=(this.N*(err-this.prevErr)+this.dFiltered)/(1+this.N*dt);
      this.prevErr=err;
      return Math.max(this.outMin,Math.min(this.outMax,this.kp*err+this.ki*this.integral+this.kd*this.dFiltered));
    }
  }

  const CAM_W=800, CAM_H=600;
  const FOCAL_PX_X=(CAM_W/2)/Math.tan((69/2)*Math.PI/180);
  const FOCAL_PX_Y=(CAM_H/2)/Math.tan((56/2)*Math.PI/180);
  const FOCAL_TOUCH_X=(CAM_W/2)/Math.tan((55/2)*Math.PI/180);
  const FOCAL_TOUCH_Y=(CAM_H/2)/Math.tan((46/2)*Math.PI/180);
  function px2deg(pxErr,focal){return Math.atan(pxErr/focal)*(180/Math.PI);}

  const pidPan=new PIDController(0.20,0.007,0.017,-20,20);
  const pidTilt=new PIDController(0.20,0.007,0.017,-20,20);
  const emaPan=new EMAFilter(0.4), emaTilt=new EMAFilter(0.4);

  let trackingActive=false, desiredMode=null, filterClasses=['person'], trackTarget='person';
  const COCO_CLASSES=new Set(['person','bicycle','car','motorcycle','airplane','bus','train','truck','boat','traffic light','fire hydrant','stop sign','parking meter','bench','bird','cat','dog','horse','sheep','cow','elephant','bear','zebra','giraffe','backpack','umbrella','handbag','tie','suitcase','frisbee','skis','snowboard','sports ball','kite','baseball bat','baseball glove','skateboard','surfboard','tennis racket','bottle','wine glass','cup','fork','knife','spoon','bowl','banana','apple','sandwich','orange','broccoli','carrot','hot dog','pizza','donut','cake','chair','couch','potted plant','bed','dining table','toilet','tv','laptop','mouse','remote','keyboard','cell phone','microwave','oven','toaster','sink','refrigerator','book','clock','vase','scissors','teddy bear','hair drier','toothbrush']);

  function getTrackLabel(){if(desiredMode==='face')return 'Yüz';if(filterClasses.length===0)return 'Tümü';return filterClasses.join(', ');}
  function startTracking(){
    trackingActive=true; pidPan.reset();pidTilt.reset();emaPan.reset();emaTilt.reset();
    trackBtn.textContent='🎯 '+getTrackLabel()+' ●'; trackBtn.classList.add('on');
    toast('Takip: '+getTrackLabel(),'ok');
    const entry=workerCache[activeModelType];
    if(entry&&entry.ready&&desiredMode==='object')entry.worker.postMessage({type:'setFilter',filterClasses});
  }
  function stopTracking(){trackingActive=false;trackBtn.textContent='🎯 Takip';trackBtn.classList.remove('on');pidPan.reset();pidTilt.reset();emaPan.reset();emaTilt.reset();}
  function setObjInputReady(ready){
    const inp=document.getElementById('objInput'),btn=document.getElementById('objGoBtn');
    inp.disabled=!ready;btn.disabled=!ready;
    inp.style.opacity=ready?'1':'0.4';btn.style.opacity=ready?'1':'0.4';
    inp.placeholder=ready?'örn: cat, dog  veya  .':'model yükleniyor…';
    if(ready)setTimeout(()=>inp.focus(),50);
  }
  function selectWorker(workerKey){
    if(detActive||trackingActive)deactivateDetection();
    if(pendingDetBmp){pendingDetBmp.close();pendingDetBmp=null;}
    tracked=[];resetClassIds(); activeModelType=workerKey;
    const entry=workerCache[workerKey];
    if(entry&&entry.ready)return 'ready';
    detDot.className='det-dot loading'; detLabel.textContent='Model yükleniyor…';
    ensureWorker(workerKey); return 'loading';
  }
  window.openTrackMenu=()=>{
    if(trackingActive){stopTracking();deactivateDetection();}
    document.getElementById('trackMenu').classList.toggle('open');
  };
  document.addEventListener('click',e=>{
    if(!e.target.closest('.track-wrap'))document.getElementById('trackMenu').classList.remove('open');
    if(!e.target.closest('.lj-wrap'))document.getElementById('ljMenu').classList.remove('open');
  });
  window.selectTrackTarget=(target)=>{
    desiredMode=target;
    ['object','face'].forEach(t=>{const el=document.getElementById('opt-'+t);if(el)el.classList.toggle('sel',t===target);});
    if(target==='face'){
      filterClasses=[];trackTarget='face';
      document.getElementById('trackMenu').classList.remove('open');
      const state=selectWorker('face');
      if(state==='ready'){activateDetection();startTracking();}
    } else {
      setObjInputReady(false);
      const state=selectWorker('person');
      if(state==='ready')setObjInputReady(true);
    }
  };
  window.startObjectTracking=()=>{
    if(desiredMode!=='object')return;
    const raw=document.getElementById('objInput').value.trim();
    if(!raw){toast('Sınıf adı girin','warn');return;}
    if(raw==='.')filterClasses=[];
    else{
      const parts=raw.split(',').map(s=>s.trim().toLowerCase()).filter(Boolean);
      const invalid=parts.filter(p=>!COCO_CLASSES.has(p));
      if(invalid.length){toast('Bilinmeyen: '+invalid.join(', '),'err');return;}
      filterClasses=parts;
    }
    trackTarget=filterClasses.length===0?'__all__':filterClasses.length===1?filterClasses[0]:'__multi__';
    document.getElementById('trackMenu').classList.remove('open');
    if(!detActive)activateDetection();
    startTracking();
  };
  window.toggleSettingsPanel=()=>{
    document.getElementById('settingsPanel').classList.toggle('open');
    document.getElementById('settingsOverlay').classList.toggle('open');
  };
  setTimeout(()=>{
    pidPan.kp=pidTilt.kp=0.20; pidPan.ki=pidTilt.ki=0.007; pidPan.kd=pidTilt.kd=0.017;
    emaPan.alpha=emaTilt.alpha=0.40; pidPan.N=pidTilt.N=40;
  },0);

  // ── Joystick ───────────────────────────────────────────────────────────────
  let JOY_R=88, dragging=false;
  function computeJoyR(){const r=joyWrap.getBoundingClientRect(),k=joyKnob.getBoundingClientRect();JOY_R=(r.width/2)-(k.width/2);}
  function applyKnob(dx,dy){
    if(ljRunning)return;
    const dist=Math.sqrt(dx*dx+dy*dy);
    if(dist>JOY_R){dx=dx/dist*JOY_R;dy=dy/dist*JOY_R;}
    joyKnob.style.transform='translate('+dx.toFixed(1)+'px,'+dy.toFixed(1)+'px)';
    sendServoTarget(clamp(90-(dx/JOY_R)*90,0,180),clamp(90+(dy/JOY_R)*90,0,180));
  }
  joyWrap.addEventListener('pointerdown',e=>{computeJoyR();dragging=true;joyWrap.classList.add('active');joyWrap.setPointerCapture(e.pointerId);const r=joyWrap.getBoundingClientRect();applyKnob(e.clientX-(r.left+r.width/2),e.clientY-(r.top+r.height/2));e.preventDefault();},{passive:false});
  joyWrap.addEventListener('pointermove',e=>{if(!dragging)return;const r=joyWrap.getBoundingClientRect();applyKnob(e.clientX-(r.left+r.width/2),e.clientY-(r.top+r.height/2));e.preventDefault();},{passive:false});
  const onUp=()=>{dragging=false;joyWrap.classList.remove('active');};
  joyWrap.addEventListener('pointerup',onUp); joyWrap.addEventListener('pointercancel',onUp);

  // ══════════════════════════════════════════════════════════════════════════
  // LJ
  // ══════════════════════════════════════════════════════════════════════════
  let ljRunning=false, ljTimer=null, ljLaserWasOn=false, homeTimer=null;
  const ljBtn=document.getElementById('ljBtn');
  const ljSizeEl=document.getElementById('ljSize');
  const ljSizeValEl=document.getElementById('ljSizeVal');
  const ljSpeedEl=document.getElementById('ljSpeed');
  const ljSpeedValEl=document.getElementById('ljSpeedVal');

  function getLjStepMs(){return 21-parseInt(ljSpeedEl.value);}
  window.onLjSlider=(el)=>{const v=parseInt(el.value);ljSizeValEl.textContent=v+'°';el.style.setProperty('--pct',((v-5)/40*100).toFixed(1)+'%');};
  window.onLjSpeedSlider=(el)=>{const v=parseInt(el.value);ljSpeedValEl.textContent=(21-v)+'ms';el.style.setProperty('--pct',((v-1)/19*100).toFixed(1)+'%');};
  setTimeout(()=>{if(ljSizeEl)window.onLjSlider(ljSizeEl);if(ljSpeedEl)window.onLjSpeedSlider(ljSpeedEl);},20);

  window.toggleLjMenu=()=>{if(ljRunning){stopLJ();return;}document.getElementById('ljMenu').classList.toggle('open');};

  function stopLJ(){
    if(homeTimer){clearTimeout(homeTimer);homeTimer=null;}
    if(ljTimer){clearTimeout(ljTimer);ljTimer=null;}
    ljRunning=false;
    if(ljLaserWasOn){laserOn=false;if(ctrlSock&&ctrlSock.readyState===WebSocket.OPEN)ctrlSock.send('laser:0');laserBtn.classList.remove('on');ljLaserWasOn=false;}
    ljBtn.textContent='✦ LJ'; ljBtn.classList.remove('running');
  }

  function lj_lerp(a,b,t){return a+(b-a)*t;}
  function lj_path(waypoints,stepDeg=0.5){
    const out=[];
    for(let i=0;i<waypoints.length-1;i++){
      const[p0,t0]=waypoints[i],[p1,t1]=waypoints[i+1];
      const dist=Math.sqrt((p1-p0)**2+(t1-t0)**2);
      const steps=Math.max(1,Math.round(dist/stepDeg));
      for(let s=0;s<steps;s++){const f=s/steps;out.push([lj_lerp(p0,p1,f),lj_lerp(t0,t1,f)]);}
    }
    out.push(waypoints[waypoints.length-1]);
    return out;
  }

  function ljPlay(positions,stepMs,onDone){
    if(homeTimer){clearTimeout(homeTimer);homeTimer=null;}
    if(trackingActive)stopTracking();
    if(detActive)deactivateDetection();
    if(ljRunning)stopLJ();
    ljRunning=true; ljBtn.textContent='■ Dur'; ljBtn.classList.add('running');
    let idx=0;
    function tick(){
      if(!ljRunning||idx>=positions.length){
        ljRunning=false;ljTimer=null;ljBtn.textContent='✦ LJ';ljBtn.classList.remove('running');
        if(onDone)onDone(); return;
      }
      sendServoTarget(positions[idx][0],positions[idx][1]); idx++;
      ljTimer=setTimeout(tick,stepMs);
    }
    tick();
  }

  // ── Lazer Şekilleri ───────────────────────────────────────────────────────
  window.runLaserShape=(shape)=>{
    const half=Math.min(parseFloat(ljSizeEl.value)/2,22.5);
    const stepMs=getLjStepMs();
    const cx=currentPan, cy=currentTilt;
    const REPS=5;
    if(!laserOn){laserOn=true;ljLaserWasOn=true;if(ctrlSock&&ctrlSock.readyState===WebSocket.OPEN)ctrlSock.send('laser:1');laserBtn.classList.add('on');}
    let oneLap=[];
    if(shape==='square'){
      const wp=[[cx-half,cy-half],[cx+half,cy-half],[cx+half,cy+half],[cx-half,cy+half],[cx-half,cy-half]].map(p=>[clamp(p[0],cx-22.5,cx+22.5),clamp(p[1],cy-22.5,cy+22.5)]);
      oneLap=lj_path(wp);
    } else if(shape==='circle'){
      const N=Math.max(24,Math.ceil(2*Math.PI*half/2.0));
      for(let i=0;i<=N;i++){const a=(i/N)*2*Math.PI;oneLap.push([clamp(cx+half*Math.cos(a),cx-22.5,cx+22.5),clamp(cy+half*Math.sin(a),cy-22.5,cy+22.5)]);}
    } else if(shape==='triangle'){
      const wp=[[cx,cy-half],[cx+half*0.866,cy+half*0.5],[cx-half*0.866,cy+half*0.5],[cx,cy-half]].map(p=>[clamp(p[0],cx-22.5,cx+22.5),clamp(p[1],cy-22.5,cy+22.5)]);
      oneLap=lj_path(wp);
    } else if(shape==='infinity'){
      const N=Math.max(120,Math.ceil(2*Math.PI*half/0.6));
      for(let i=0;i<=N;i++){
        const t=(i/N)*2*Math.PI;
        const s=Math.sin(t), c=Math.cos(t);
        const denom=1+s*s;
        oneLap.push([
          clamp(cx+half*c/denom,         cx-22.5,cx+22.5),
          clamp(cy+half*2.2*s*c/denom,   cy-22.5,cy+22.5)
        ]);
      }
    }
    let pts=[];
    for(let r=0;r<REPS;r++)pts=pts.concat(oneLap);
    ljPlay(pts,stepMs,()=>{
      if(ljLaserWasOn){laserOn=false;if(ctrlSock&&ctrlSock.readyState===WebSocket.OPEN)ctrlSock.send('laser:0');laserBtn.classList.remove('on');ljLaserWasOn=false;}
      sendServoTarget(cx,cy);
    });
  };

  // ── Jestler ───────────────────────────────────────────────────────────────
  window.runGesture=(type)=>{
    const cx=currentPan, cy=currentTilt;
    let pts=[];
    const home=()=>{homeTimer=setTimeout(()=>sendServoTarget(clamp(cx,0,180),clamp(cy,0,180)),1000);};

    if(type==='evet'){
      const wp=[[cx,cy],[cx,cy+24],[cx,cy-6],[cx,cy+20],[cx,cy-4],[cx,cy]]
        .map(p=>[clamp(p[0],0,180),clamp(p[1],0,180)]);
      pts=lj_path(wp,1.0);
      ljPlay(pts,2,home); return;

    } else if(type==='hayir'){
      const wp=[[cx,cy],[cx+22,cy],[cx-22,cy],[cx+14,cy],[cx-14,cy],[cx,cy]]
        .map(p=>[clamp(p[0],0,180),clamp(p[1],0,180)]);
      pts=lj_path(wp,1.6);
      ljPlay(pts,2,home); return;

    } else if(type==='kusme'){
      const STEP_MS=25;
      const sideX=18, dropY=30;
      for(let i=0;i<80;i++){
        const t=i/80;
        const ease=1-Math.pow(1-t,2.8);
        const resist=2.2*Math.sin(2*Math.PI*1.8*t)*Math.max(0,1-t*2.5);
        pts.push([
          clamp(cx+sideX*ease+resist,0,180),
          clamp(cy+dropY*ease+resist*0.6,0,180)
        ]);
      }
      const botP=clamp(cx+sideX,0,180), botT=clamp(cy+dropY,0,180);
      const sighWp=[
        [botP,botT],[botP,botT-5],[botP,botT],
        [botP+0.8,botT-3.2],[botP,botT],
        [botP,botT-1.8],[botP,botT]
      ].map(p=>[clamp(p[0],0,180),clamp(p[1],0,180)]);
      pts=pts.concat(lj_path(sighWp,0.38));
      for(let i=0;i<60;i++){
        const t=i/60;
        pts.push([
          clamp(botP+3.0*Math.sin(2*Math.PI*0.65*t),0,180),
          clamp(botT+1.8*Math.sin(2*Math.PI*1.0*t+0.5),0,180)
        ]);
      }
      pts=pts.concat(lj_path([[botP,botT],[clamp(botP+4,0,180),clamp(botT+7,0,180)]],1.8));
      for(let i=0;i<20;i++) pts.push([clamp(botP+4,0,180),clamp(botT+7,0,180)]);
      ljPlay(pts,STEP_MS,home); return;

    } else if(type==='korku'){
      const STEP_MS=13;
      pts=pts.concat(lj_path([
        [cx,cy],
        [clamp(cx-18,0,180),clamp(cy+28,0,180)]
      ],7.5));
      for(let i=0;i<92;i++){
        const t=i/92;
        const tr=5.5*Math.exp(-t*2.0)+1.8;
        pts.push([
          clamp(cx-17+tr*Math.sin(2*Math.PI*9.5*t),0,180),
          clamp(cy+27+tr*0.55*Math.cos(2*Math.PI*7.5*t),0,180)
        ]);
      }
      pts=pts.concat(lj_path([
        [clamp(cx-17,0,180),clamp(cy+27,0,180)],
        [cx,clamp(cy-4,0,180)]
      ],1.5));
      for(let i=0;i<20;i++){
        const t=i/20;
        pts.push([
          clamp(cx+1.5*Math.sin(2*Math.PI*1.2*t),0,180),
          clamp(cy-4+1.2*Math.sin(2*Math.PI*0.8*t+0.5),0,180)
        ]);
      }
      pts=pts.concat(lj_path([
        [cx,clamp(cy-4,0,180)],
        [clamp(cx-24,0,180),clamp(cy+32,0,180)]
      ],8.0));
      for(let i=0;i<85;i++){
        const t=i/85;
        const tr=7.5*Math.exp(-t*2.5)+2.2;
        pts.push([
          clamp(cx-22+tr*Math.sin(2*Math.PI*11*t),0,180),
          clamp(cy+30+tr*0.45*Math.cos(2*Math.PI*8.5*t+0.7),0,180)
        ]);
      }
      pts=pts.concat(lj_path([
        [clamp(cx-22,0,180),clamp(cy+30,0,180)],
        [clamp(cx+8,0,180),clamp(cy+15,0,180)]
      ],1.0));
      for(let i=0;i<18;i++){
        const t=i/18;
        pts.push([clamp(cx+8+1.2*Math.sin(t*2),0,180),clamp(cy+15+0.8*Math.cos(t*2),0,180)]);
      }
      pts=pts.concat(lj_path([
        [clamp(cx+8,0,180),clamp(cy+15,0,180)],
        [cx,cy]
      ],0.7));
      ljPlay(pts,STEP_MS,home); return;

    } else if(type==='dans'){
      const STEP_MS=20, TOTAL=250;
      for(let i=0;i<TOTAL;i++){
        const tMs=i*STEP_MS;
        let pan=cx, tilt=cy;
        if(tMs<2000){pan=cx+20*Math.sin(2*Math.PI*2*tMs/1000);tilt=cy+6*Math.sin(2*Math.PI*4*tMs/1000);}
        else if(tMs<3500){const ph=tMs-2000;pan=cx+8*Math.sin(2*Math.PI*2*ph/1000);tilt=cy+18*Math.sin(2*Math.PI*1.5*ph/1000);}
        else{const ph=tMs-3500;pan=cx+18*Math.sin(2*Math.PI*1.5*ph/1000);tilt=cy+12*Math.sin(2*Math.PI*3*ph/1000);}
        pts.push([clamp(pan,cx-45,cx+45),clamp(tilt,cy-45,cy+45)]);
      }
      const last=pts[pts.length-1];
      pts=pts.concat(lj_path([last,[cx,cy]]));
      ljPlay(pts,STEP_MS,home); return;

    } else if(type==='dans2'){
      const STEP_MS=13;
      const TOTAL_STEPS=400;
      const BPM=105;
      const beatHz=BPM/60;
      for(let i=0;i<TOTAL_STEPS;i++){
        const tSec=i*STEP_MS/1000;
        const tNorm=i/TOTAL_STEPS;
        const env=tNorm<0.1?(tNorm/0.1):tNorm>0.9?((1-tNorm)/0.1):1.0;
        const sway=26*Math.sin(2*Math.PI*(beatHz/2)*tSec);
        const bob=15*Math.abs(Math.sin(2*Math.PI*beatHz*tSec))-5;
        const shimmy=5*Math.sin(2*Math.PI*beatHz*2*tSec+0.4);
        pts.push([
          clamp(cx+env*(sway+shimmy*0.4),cx-45,cx+45),
          clamp(cy+env*bob,cy-25,cy+20)
        ]);
      }
      const lastD=pts[pts.length-1];
      pts=pts.concat(lj_path([lastD,[cx,cy]],2));
      ljPlay(pts,STEP_MS,home); return;

    } else if(type==='dans3'){
      const STEP_MS=10;
      const TOTAL=550;
      const BPM=128;
      const beatHz=BPM/60;
      const halfBeat=beatHz/2;
      for(let i=0;i<TOTAL;i++){
        const tSec=i*STEP_MS/1000;
        const tNorm=i/TOTAL;
        const env=tNorm<0.08?(tNorm/0.08):tNorm>0.92?((1-tNorm)/0.08):1.0;
        const sway=30*Math.sin(2*Math.PI*halfBeat*tSec+0.3);
        const bob=13*Math.sin(2*Math.PI*beatHz*tSec - Math.PI*0.4) - 4;
        const roll=10*Math.sin(2*Math.PI*(beatHz*0.25)*tSec + 0.8);
        const downbeatEnv=Math.abs(Math.sin(2*Math.PI*beatHz*tSec));
        const shimmy=5*Math.sin(2*Math.PI*beatHz*4*tSec) * downbeatEnv;
        pts.push([
          clamp(cx + env*(sway + shimmy*0.5 + roll*0.35), cx-45, cx+45),
          clamp(cy + env*(bob  + roll*0.6),                cy-28, cy+18)
        ]);
      }
      const lastD3=pts[pts.length-1];
      pts=pts.concat(lj_path([lastD3,[cx,cy]],1.5));
      ljPlay(pts,STEP_MS,home); return;

    } else if(type==='uyku'){
      const STEP_MS=20;
      for(let i=0;i<40;i++){
        const t=i/40;
        pts.push([
          clamp(cx+2.5*Math.sin(2*Math.PI*0.55*t),0,180),
          clamp(cy+1.8*Math.sin(2*Math.PI*1.1*t+0.3),0,180)
        ]);
      }
      for(let i=0;i<70;i++){
        const t=i/70;
        const fall=t*t*38;
        pts.push([
          clamp(cx+1.5*Math.sin(2*Math.PI*0.28*t),0,180),
          clamp(cy+fall,0,180)
        ]);
      }
      pts=pts.concat(lj_path([
        [cx,clamp(cy+38,0,180)],
        [cx,clamp(cy-10,0,180)]
      ],5.5));
      for(let i=0;i<20;i++){
        const t=i/20;
        pts.push([clamp(cx+1*Math.sin(t*2),0,180),clamp(cy-10+t*0.5*10,0,180)]);
      }
      for(let i=0;i<60;i++){
        const t=i/60;
        const fall=t*t*42;
        pts.push([
          clamp(cx-1.2*Math.sin(2*Math.PI*0.22*t),0,180),
          clamp(cy+fall,0,180)
        ]);
      }
      pts=pts.concat(lj_path([
        [cx,clamp(cy+42,0,180)],
        [cx,clamp(cy-14,0,180)]
      ],7.0));
      for(let i=0;i<15;i++){
        pts.push([clamp(cx+1.5*(i%2===0?1:-1),0,180),clamp(cy-14+i*0.5,0,180)]);
      }
      for(let i=0;i<30;i++){
        const t=i/30;
        pts.push([
          clamp(cx+0.8*Math.sin(2*Math.PI*0.2*t),0,180),
          clamp(cy+t*t*18,0,180)
        ]);
      }
      for(let i=0;i<12;i++){
        const jolt=3.8*Math.sin(2*Math.PI*3*(i/12))*Math.exp(-i/12*2.2);
        pts.push([clamp(cx+jolt,0,180),clamp(cy+18+jolt*0.35,0,180)]);
      }
      pts=pts.concat(lj_path([
        [cx,clamp(cy+18,0,180)],
        [cx,clamp(cy-10,0,180)]
      ],4.5));
      const wakeWp=[
        [cx,               clamp(cy-10,0,180)],
        [clamp(cx+22,0,180),clamp(cy-10,0,180)],
        [clamp(cx-22,0,180),clamp(cy-10,0,180)],
        [clamp(cx+13,0,180),clamp(cy-5, 0,180)],
        [clamp(cx-13,0,180),clamp(cy-5, 0,180)],
        [clamp(cx+5, 0,180), cy],
        [cx, cy]
      ];
      pts=pts.concat(lj_path(wakeWp,1.4));
      ljPlay(pts,STEP_MS,home); return;

    } else if(type==='uyanis'){
      const STEP_MS=12;
      for(let i=0;i<30;i++) pts.push([clamp(cx+3,0,180),clamp(cy+14,0,180)]);
      pts=pts.concat(lj_path([[clamp(cx+3,0,180),clamp(cy+14,0,180)],[cx,clamp(cy-32,0,180)]],5.5));
      for(let i=0;i<90;i++){
        const t=i/90;
        const tr=4.2*Math.exp(-t*3.0)+0.9;
        pts.push([
          clamp(cx+tr*Math.sin(2*Math.PI*9*t+0.3),0,180),
          clamp(cy-32+tr*0.5*Math.sin(2*Math.PI*7*t),0,180)
        ]);
      }
      pts=pts.concat(lj_path([[cx,clamp(cy-32,0,180)],[clamp(cx+40,0,180),clamp(cy-28,0,180)]],5.5));
      for(let i=0;i<65;i++){
        const t=i/65;
        pts.push([
          clamp(cx+38+2.5*Math.sin(2*Math.PI*1.8*t),0,180),
          clamp(cy-26+2.2*Math.sin(2*Math.PI*2.5*t+0.4),0,180)
        ]);
      }
      pts=pts.concat(lj_path([[clamp(cx+38,0,180),clamp(cy-26,0,180)],[clamp(cx-40,0,180),clamp(cy-28,0,180)]],6.0));
      for(let i=0;i<65;i++){
        const t=i/65;
        pts.push([
          clamp(cx-38+2.5*Math.sin(2*Math.PI*1.5*t+0.5),0,180),
          clamp(cy-24+3.5*Math.sin(2*Math.PI*2.0*t),0,180)
        ]);
      }
      pts=pts.concat(lj_path([
        [clamp(cx-38,0,180),clamp(cy-24,0,180)],
        [clamp(cx+30,0,180),clamp(cy+14,0,180)]
      ],2.8));
      for(let i=0;i<28;i++) pts.push([clamp(cx+30,0,180),clamp(cy+14,0,180)]);
      pts=pts.concat(lj_path([
        [clamp(cx+30,0,180),clamp(cy+14,0,180)],
        [clamp(cx-30,0,180),clamp(cy+14,0,180)]
      ],2.8));
      for(let i=0;i<28;i++) pts.push([clamp(cx-30,0,180),clamp(cy+14,0,180)]);
      pts=pts.concat(lj_path([
        [clamp(cx-30,0,180),clamp(cy+14,0,180)],
        [cx,clamp(cy-10,0,180)]
      ],2.2));
      for(let i=0;i<30;i++){
        const t=i/30;
        pts.push([
          clamp(cx+3.5*Math.sin(2*Math.PI*0.9*t),0,180),
          clamp(cy-10+2*Math.sin(2*Math.PI*1.4*t+0.5),0,180)
        ]);
      }
      pts=pts.concat(lj_path([[cx,clamp(cy-10,0,180)],[cx,cy]],1.0));
      ljPlay(pts,STEP_MS,home); return;

    } else if(type==='spiral'){
      const sp=60,st=60,ep=90,et=90;
      const rStart=Math.sqrt((sp-ep)**2+(st-et)**2);
      const aStart=Math.atan2(st-et,sp-ep);
      const movePts=lj_path([[cx,cy],[sp,st]],1.6);
      const spiralPts=[];
      const sSteps=Math.ceil(3*2*Math.PI*rStart/0.5);
      for(let i=0;i<=sSteps;i++){
        const prog=i/sSteps,r=rStart*(1-prog),a=aStart+3*2*Math.PI*prog;
        spiralPts.push([clamp(ep+r*Math.cos(a),45,135),clamp(et+r*Math.sin(a),45,135)]);
      }
      pts=movePts.concat(spiralPts);
      ljPlay(pts,2,home); return;
    }

    ljPlay(pts,2,home);
  };

  // ══════════════════════════════════════════════════════════════════════════
  // Worker Cache
  // ══════════════════════════════════════════════════════════════════════════
  const workerCache={};
  let activeModelType='person', detActive=false, pendingDetBmp=null;
  const WORKER_BLOB_URL=URL.createObjectURL(new Blob([WORKER_SRC],{type:'application/javascript'}));

  function ensureWorker(modelType){
    if(workerCache[modelType])return;
    const w=new Worker(WORKER_BLOB_URL);
    const entry={worker:w,ready:false,busy:false,dispatchTime:0};
    workerCache[modelType]=entry;
    w.onmessage=(e)=>onWorkerMsg(modelType,e);
    w.onerror=()=>{const u=modelType==='face'?'Yüz':'COCO-SSD';detDot.className='det-dot error';detLabel.textContent=u+' yüklenemedi';toast(u+' yüklenemedi','err');};
    w.postMessage({type:'init',modelType});
  }
  function onWorkerMsg(modelType,e){
    const msg=e.data,entry=workerCache[modelType];
    if(!entry)return;
    if(msg.type==='ready'){
      entry.ready=true;
      const u=modelType==='face'?'Yüz':'COCO-SSD';
      toast('Hazır: '+u+' — '+msg.backend.toUpperCase(),'ok');
      if(modelType!==activeModelType)return;
      if(desiredMode==='face'&&modelType==='face'){activateDetection();startTracking();document.getElementById('trackMenu').classList.remove('open');}
      else if(desiredMode==='object'&&modelType==='person'){detDot.className='det-dot ready';detLabel.textContent='Hazır — COCO-SSD';setObjInputReady(true);}
      return;
    }
    if(msg.type==='error'){const u=modelType==='face'?'Yüz':'COCO-SSD';if(modelType===activeModelType){detDot.className='det-dot error';detLabel.textContent=u+' yüklenemedi';}toast(u+' hata: '+msg.msg,'err');return;}
    if(msg.type==='detections'){
      const latencyMs=entry.dispatchTime?Math.round(performance.now()-entry.dispatchTime):0;
      entry.busy=false;
      if(detActive&&modelType===activeModelType){drawBoxes(updateTracker(msg.data));detFpsLabel.textContent=latencyMs+' ms';drainQueue();}
    }
  }
  function activateDetection(){detActive=true;tracked=[];resetClassIds();detDot.className='det-dot active';detLabel.textContent='Aktif — '+(desiredMode==='face'?'Yüz':getTrackLabel());detFpsLabel.textContent='';}
  function deactivateDetection(){
    if(trackingActive)stopTracking();
    detActive=false;detCtx.clearRect(0,0,800,600);tracked=[];resetClassIds();
    if(pendingDetBmp){pendingDetBmp.close();pendingDetBmp=null;}
    crosshairEl.classList.remove('hit');pidPan.reset();pidTilt.reset();emaPan.reset();emaTilt.reset();
    const entry=workerCache[activeModelType];
    detDot.className=(entry&&entry.ready)?'det-dot ready':'det-dot';
    detLabel.textContent=(entry&&entry.ready)?'Hazır — '+(desiredMode==='face'?'Yüz':'COCO-SSD'):'Tespite hazır';
    detFpsLabel.textContent='';
  }
  function tryDispatch(detBmp){
    const entry=workerCache[activeModelType];
    if(!entry||!entry.ready){detBmp.close();return 'not_ready';}
    if(entry.busy)return 'busy';
    entry.busy=true; entry.dispatchTime=performance.now();
    entry.worker.postMessage({type:'detect',bitmap:detBmp,filterClasses:desiredMode==='object'?filterClasses:null},[detBmp]);
    return 'sent';
  }
  function drainQueue(){if(!detActive||!pendingDetBmp)return;const bmp=pendingDetBmp;pendingDetBmp=null;const r=tryDispatch(bmp);if(r==='busy')pendingDetBmp=bmp;}

  // ── Tracker ────────────────────────────────────────────────────────────────
  const CH_X=400,CH_Y=300,CH_HIT_R=25;
  const DET_COLORS=['#4cc9f0','#06d6a0','#f59e0b','#a78bfa','#fb923c','#34d399','#60a5fa','#f472b6','#e879f9','#2dd4bf','#facc15','#f87171'];
  function clsIdColor(id){return DET_COLORS[(id-1)%DET_COLORS.length];}
  let tracked=[],activeClassIds={};
  function allocClassId(cls){if(!activeClassIds[cls])activeClassIds[cls]=new Set();let id=1;while(activeClassIds[cls].has(id))id++;activeClassIds[cls].add(id);return id;}
  function freeClassId(cls,id){activeClassIds[cls]?.delete(id);}
  function resetClassIds(){activeClassIds={};}
  function iou(a,b){const ix1=Math.max(a[0],b[0]),iy1=Math.max(a[1],b[1]),ix2=Math.min(a[0]+a[2],b[0]+b[2]),iy2=Math.min(a[1]+a[3],b[1]+b[3]);const inter=Math.max(0,ix2-ix1)*Math.max(0,iy2-iy1);return inter?inter/((a[2]*a[3])+(b[2]*b[3])-inter+1e-6):0;}
  function updateTracker(dets){
    const next=[],used=new Set();
    for(const d of dets){
      let best=null,bestScore=0.15;
      for(const t of tracked){if(used.has(t.key)||t.cls!==d.class)continue;const s=iou(d.bbox,t.bbox);if(s>bestScore){bestScore=s;best=t;}}
      if(best){used.add(best.key);next.push({cls:best.cls,clsId:best.clsId,key:best.key,bbox:d.bbox,score:d.score,age:0});}
      else{const clsId=allocClassId(d.class),key=d.class+':'+clsId;next.push({cls:d.class,clsId,key,bbox:d.bbox,score:d.score,age:0});}
    }
    for(const t of tracked){if(used.has(t.key))continue;if(t.age<4)next.push({...t,age:t.age+1});else freeClassId(t.cls,t.clsId);}
    tracked=next; return next.filter(t=>t.age===0);
  }
  function drawBoxes(objs){
    detCtx.clearRect(0,0,800,600);
    let hit=false,bestTarget=null;
    for(const obj of objs){const m=trackTarget==='__all__'||trackTarget==='__multi__'?(filterClasses.length===0||filterClasses.includes(obj.cls)):obj.cls===trackTarget;if(m&&(!bestTarget||obj.score>bestTarget.score))bestTarget=obj;}
    for(const obj of objs){
      const bx=obj.bbox[0]*800,by=obj.bbox[1]*600,bw=obj.bbox[2]*800,bh=obj.bbox[3]*600;
      const fbx=800-(bx+bw),col=clsIdColor(obj.clsId);
      const rx=Math.round(fbx),ry=Math.round(by),rw=Math.round(bw),rh=Math.round(bh);
      detCtx.strokeStyle=col;detCtx.lineWidth=2;detCtx.strokeRect(rx,ry,rw,rh);
      const lbl=obj.cls+' '+obj.clsId+' '+Math.round(obj.score*100)+'%';
      detCtx.font='bold 11px monospace';
      const tw=Math.ceil(detCtx.measureText(lbl).width)+6;
      const ly=Math.max(0,ry-16);
      detCtx.fillStyle=col;detCtx.fillRect(rx,ly,tw,16);
      detCtx.fillStyle='#000';detCtx.fillText(lbl,rx+3,ly+11);
      const cx2=Math.round(fbx+bw/2),cy2=Math.round(by+bh/2);
      detCtx.beginPath();detCtx.arc(cx2,cy2,3,0,Math.PI*2);detCtx.fillStyle=col;detCtx.fill();
      if((cx2-CH_X)**2+(cy2-CH_Y)**2<=CH_HIT_R**2)hit=true;
    }
    crosshairEl.classList.toggle('hit',hit);
    if(trackingActive&&detActive&&bestTarget){
      const bx=bestTarget.bbox[0]*800,by=bestTarget.bbox[1]*600,bw=bestTarget.bbox[2]*800,bh=bestTarget.bbox[3]*600;
      const cx2=Math.round(800-(bx+bw)+bw/2),cy2=Math.round(by+bh/2);
      const dist=Math.sqrt((cx2-CH_X)**2+(cy2-CH_Y)**2);
      if(dist>CH_HIT_R){
        const corrPan=pidPan.compute(emaPan.filter(px2deg(cx2-CH_X,FOCAL_PX_X)));
        const corrTilt=pidTilt.compute(emaTilt.filter(px2deg(cy2-CH_Y,FOCAL_PX_Y)));
        sendServoTarget(clamp(currentPan-corrPan,0,180),clamp(currentTilt+corrTilt,0,180));
        detCtx.beginPath();detCtx.arc(cx2,cy2,10,0,Math.PI*2);
        detCtx.strokeStyle='rgba(167,139,250,0.8)';detCtx.lineWidth=1.5;
        detCtx.setLineDash([4,3]);detCtx.stroke();detCtx.setLineDash([]);
      } else {pidPan.reset();pidTilt.reset();}
    } else if(trackingActive&&!bestTarget){pidPan.reset();pidTilt.reset();}
  }

  // ── Frame Pipeline ─────────────────────────────────────────────────────────
  let pendingBmp=null,rendering=false,decoding=0;
  function schedRender(){
    if(rendering)return;rendering=true;
    requestAnimationFrame(()=>{
      rendering=false;if(!pendingBmp)return;
      const bmp=pendingBmp;pendingBmp=null;
      ctx.save();ctx.translate(800,0);ctx.scale(-1,1);ctx.drawImage(bmp,0,0,800,600);ctx.restore();
      bmp.close();fCnt++;
    });
  }
  function onBinaryFrame(ab){
    if(decoding>=2)return;decoding++;
    const blob=new Blob([ab],{type:'image/jpeg'});
    const activeEntry=workerCache[activeModelType];
    const needDet=detActive&&activeEntry&&activeEntry.ready;
    Promise.all([
      createImageBitmap(blob),
      needDet?createImageBitmap(blob,{resizeWidth:DET_W,resizeHeight:DET_H}):Promise.resolve(null)
    ]).then(([displayBmp,detBmp])=>{
      decoding--;if(pendingBmp)pendingBmp.close();pendingBmp=displayBmp;schedRender();
      if(!detBmp)return;
      const r=tryDispatch(detBmp);
      if(r==='busy'){if(pendingDetBmp)pendingDetBmp.close();pendingDetBmp=detBmp;}
    }).catch(()=>decoding--);
  }

  // ── WebSocket: Video ────────────────────────────────────────────────────────
  function connectVideo(){
    clearTimeout(vidRTimer);
    if(vidSock){vidSock.onopen=vidSock.onmessage=vidSock.onclose=vidSock.onerror=null;try{vidSock.close();}catch(_){}vidSock=null;}
    try{vidSock=new WebSocket('ws://'+location.hostname+'/ws');}catch(e){schedVidReconnect();return;}
    vidSock.binaryType='arraybuffer';
    vidSock.onopen=()=>{wsConnected=true;vidDelay=500;connDot.className='conn-dot ok';connLabel.textContent='Bağlı';applyState();if(userStarted)vidSock.send('start');};
    vidSock.onmessage=evt=>{
      if(typeof evt.data==='string'){try{const d=JSON.parse(evt.data);if(d.type==='diag'&&typeof d.camActive==='boolean'){camOn=d.camActive;applyState();}}catch(_){}return;}
      if(evt.data instanceof ArrayBuffer&&evt.data.byteLength>0)onBinaryFrame(evt.data);
    };
    vidSock.onclose=()=>{wsConnected=false;camOn=false;connDot.className='conn-dot warn';connLabel.textContent='Yeniden bağlanıyor…';applyState();schedVidReconnect();};
    vidSock.onerror=()=>{try{vidSock.close();}catch(_){}};
  }
  function schedVidReconnect(){clearTimeout(vidRTimer);vidRTimer=setTimeout(()=>{connectVideo();vidDelay=Math.min(vidDelay*1.5,4000);},vidDelay);}

  // ── WebSocket: Kontrol ──────────────────────────────────────────────────────
  function connectCtrl(){
    clearTimeout(ctrlRTimer);
    if(ctrlSock){ctrlSock.onopen=ctrlSock.onclose=ctrlSock.onerror=null;try{ctrlSock.close();}catch(_){}ctrlSock=null;}
    try{ctrlSock=new WebSocket('ws://'+location.hostname+'/ctrl');}catch(e){schedCtrlReconnect();return;}
    ctrlSock.onopen=()=>{ctrlDelay=500;};
    ctrlSock.onclose=()=>schedCtrlReconnect();
    ctrlSock.onerror=()=>{try{ctrlSock.close();}catch(_){}};
  }
  function schedCtrlReconnect(){clearTimeout(ctrlRTimer);ctrlRTimer=setTimeout(()=>{connectCtrl();ctrlDelay=Math.min(ctrlDelay*1.5,4000);},ctrlDelay);}

  window.toggleStream=()=>{
    if(!vidSock||vidSock.readyState!==WebSocket.OPEN)return;
    if(!camOn){userStarted=true;vidSock.send('start');toast('▶ Başlatılıyor…','ok');}
    else{userStarted=false;vidSock.send('stop');toast('⏹ Durduruldu','ok');}
    streamBtn.disabled=true;
  };
  document.addEventListener('visibilitychange',()=>{
    if(document.visibilityState==='visible'){if(!wsConnected)connectVideo();if(!ctrlSock||ctrlSock.readyState!==WebSocket.OPEN)connectCtrl();}
  });

  // ── Dokunma → Servo ────────────────────────────────────────────────────────
  const touchCanvas=document.getElementById('touchCanvas');
  const touchCtx=touchCanvas.getContext('2d');
  const videoInner=document.getElementById('videoInner');
  const ripples=[];let rippleRaf=null;
  function drawRipples(){
    touchCtx.clearRect(0,0,800,600);
    const now=performance.now();let alive=false;
    for(const r of ripples){
      const age=now-r.born,prog=age/300;
      if(prog>=1)continue;alive=true;
      const alpha=1-prog,dotR=14*(1-prog*0.6);
      touchCtx.beginPath();touchCtx.arc(r.x,r.y,dotR,0,Math.PI*2);
      touchCtx.fillStyle=`rgba(255,255,255,${(alpha*0.85).toFixed(2)})`;touchCtx.fill();
      const ringR=10+prog*32,lw=2.5*(1-prog);
      touchCtx.beginPath();touchCtx.arc(r.x,r.y,ringR,0,Math.PI*2);
      touchCtx.strokeStyle=`rgba(76,201,240,${(alpha*0.9).toFixed(2)})`;touchCtx.lineWidth=lw;touchCtx.stroke();
    }
    if(alive){rippleRaf=requestAnimationFrame(drawRipples);}else{touchCtx.clearRect(0,0,800,600);rippleRaf=null;}
  }
  function spawnRipple(cx,cy){ripples.push({x:cx,y:cy,born:performance.now()});if(ripples.length>5)ripples.splice(0,ripples.length-5);if(!rippleRaf)rippleRaf=requestAnimationFrame(drawRipples);}
  function onVideoTap(e){
    if(ljRunning)return;if(e.target.closest('.overlay-top'))return;e.preventDefault();
    const rect=videoInner.getBoundingClientRect();
    const scaleX=800/rect.width,scaleY=600/rect.height;
    const clientX=e.clientX??e.touches?.[0]?.clientX,clientY=e.clientY??e.touches?.[0]?.clientY;
    if(clientX===undefined)return;
    const canvasX=(clientX-rect.left)*scaleX,canvasY=(clientY-rect.top)*scaleY;
    spawnRipple(canvasX,canvasY);
    const pxErrX=canvasX-CH_X,pxErrY=canvasY-CH_Y;
    const degErrPan=Math.atan(pxErrX/FOCAL_TOUCH_X)*(180/Math.PI);
    const degErrTilt=Math.atan(pxErrY/FOCAL_TOUCH_Y)*(180/Math.PI);
    sendServoTarget(clamp(currentPan-degErrPan,0,180),clamp(currentTilt+degErrTilt,0,180));
  }
  videoInner.addEventListener('pointerdown',onVideoTap,{passive:false});

  connectVideo();
  connectCtrl();
})();
</script>
</body>
</html>
)rawliteral";

// ── WS: Video ─────────────────────────────────────────────────────────────────
void onVideoWsEvent(AsyncWebSocket *srv, AsyncWebSocketClient *client,
                    AwsEventType type, void *arg, uint8_t *data, size_t len) {
  if (type == WS_EVT_CONNECT) {
    for (auto &c : ws.getClients())
      if (c.id()!=client->id() && c.status()==WS_CONNECTED) c.close();
    Serial.printf("[WS-VID] #%u bağlandı\n", client->id());
    camStateChanged=true; return;
  }
  if (type == WS_EVT_DISCONNECT) { Serial.printf("[WS-VID] #%u ayrıldı\n", client->id()); return; }
  if (type != WS_EVT_DATA) return;
  AwsFrameInfo *info=(AwsFrameInfo*)arg;
  if (info->opcode!=WS_TEXT||len==0) return;
  char cmd[16]={0}; memcpy(cmd,data,len<15?len:15);
  if     (!strcmp(cmd,"start")) pendingStart=true;
  else if(!strcmp(cmd,"stop"))  pendingStop=true;
}

// ── WS: Kontrol ───────────────────────────────────────────────────────────────
void onCtrlWsEvent(AsyncWebSocket *srv, AsyncWebSocketClient *client,
                   AwsEventType type, void *arg, uint8_t *data, size_t len) {
  if (type == WS_EVT_CONNECT) {
    for (auto &c : ctrl.getClients())
      if (c.id()!=client->id() && c.status()==WS_CONNECTED) c.close();
    Serial.printf("[WS-CTRL] #%u bağlandı\n", client->id()); return;
  }
  if (type == WS_EVT_DISCONNECT) { Serial.printf("[WS-CTRL] #%u ayrıldı\n", client->id()); return; }
  if (type != WS_EVT_DATA) return;
  AwsFrameInfo *info=(AwsFrameInfo*)arg;
  if (info->opcode!=WS_TEXT||len==0) return;
  char cmd[32]={0}; memcpy(cmd,data,len<31?len:31);
  if     (!strncmp(cmd,"pan:",   4)) setServoDirect(PAN_LEDC_CHANNEL,  strtof(cmd+4,nullptr));
  else if(!strncmp(cmd,"tilt:",  5)) setServoDirect(TILT_LEDC_CHANNEL, strtof(cmd+5,nullptr));
  else if(!strncmp(cmd,"laser:", 6)) digitalWrite(LASER_PIN, cmd[6]=='1' ? HIGH : LOW);
}

// ── Setup ─────────────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200); delay(300);
  Serial.println(F("\n=== XIAO CAM + PAN/TILT + DETECT v10.3 ==="));
  servoInit();
  setServoDirect(PAN_LEDC_CHANNEL,  90.0f);
  setServoDirect(TILT_LEDC_CHANNEL, 90.0f);
  pinMode(LASER_PIN, OUTPUT);
  digitalWrite(LASER_PIN, LOW);
  Serial.printf("[LASER] GPIO%d — kapalı\n", LASER_PIN);
  WiFi.mode(WIFI_AP);
  WiFi.softAP(AP_SSID, AP_PASSWORD, WIFI_CHANNEL, 0, MAX_CLIENTS);
  WiFi.setSleep(false);
  esp_wifi_set_ps(WIFI_PS_NONE);
  esp_wifi_set_bandwidth(WIFI_IF_AP, WIFI_BW_HT40);
  wifi_country_t wc = {};
  strncpy(wc.cc, "TR", sizeof(wc.cc));
  wc.schan=1; wc.nchan=13; wc.policy=WIFI_COUNTRY_POLICY_MANUAL;
  esp_wifi_set_country(&wc);
  wifi_config_t ap_conf = {};
  esp_wifi_get_config(WIFI_IF_AP, &ap_conf);
  ap_conf.ap.beacon_interval=50;
  esp_wifi_set_config(WIFI_IF_AP, &ap_conf);
  WiFi.setTxPower(WIFI_POWER_19_5dBm);
  esp_wifi_set_protocol(WIFI_IF_AP, WIFI_PROTOCOL_11B|WIFI_PROTOCOL_11G|WIFI_PROTOCOL_11N);
  Serial.printf("[WIFI] %s | %s\n", AP_SSID, WiFi.softAPIP().toString().c_str());
  ws.onEvent(onVideoWsEvent);
  ctrl.onEvent(onCtrlWsEvent);
  server.addHandler(&ws);
  server.addHandler(&ctrl);
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *r){ r->send(200,"text/html",HTML_PAGE); });
  server.onNotFound([](AsyncWebServerRequest *r){ r->send(404,"text/plain","Not found"); });
  server.begin();
  Serial.println(F("[HTTP] http://192.168.4.1"));
  xTaskCreatePinnedToCore(cameraTask,"CamStream",8192,NULL,1,&cameraTaskHandle,1);
}

void loop() {
  ws.cleanupClients(MAX_CLIENTS);
  ctrl.cleanupClients(MAX_CLIENTS);
  sendDiag();
  delay(5);
}
