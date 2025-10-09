#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <esp_camera.h>
#include <ESP32Servo.h>
#include <ArduinoJson.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_wifi.h>

// AP Ayarları
const char* AP_SSID = "ESP32CAM_AP";
const char* AP_PASSWORD = "12345678";

// Servo Ayarları
Servo servoPan, servoTilt;
const int SERVO_PAN_PIN = 14;
const int SERVO_TILT_PIN = 15;
volatile int panAngle = 90;
volatile int tiltAngle = 90;

// Flash Ayarları
#define FLASH_GPIO_NUM 4
volatile bool flashState = false;

// Kamera pinleri (AI-Thinker)
#define PWDN_GPIO_NUM 32
#define RESET_GPIO_NUM -1
#define XCLK_GPIO_NUM 0
#define SIOD_GPIO_NUM 26
#define SIOC_GPIO_NUM 27
#define Y9_GPIO_NUM 35
#define Y8_GPIO_NUM 34
#define Y7_GPIO_NUM 39
#define Y6_GPIO_NUM 36
#define Y5_GPIO_NUM 21
#define Y4_GPIO_NUM 19
#define Y3_GPIO_NUM 18
#define Y2_GPIO_NUM 5
#define VSYNC_GPIO_NUM 25
#define HREF_GPIO_NUM 23
#define PCLK_GPIO_NUM 22

// Async sunucular
AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

// Kamera ve FPS kontrol
const int STREAM_FPS = 20;
unsigned long lastFrameTime = 0;
bool cameraActive = true;

// RTOS Task Handles
TaskHandle_t cameraTaskHandle = NULL;

// WiFi optimizasyon parametreleri
#define WIFI_CHANNEL 1
#define MAX_CLIENTS 1
#define WIFI_PROTOCOL WIFI_PROTOCOL_11N

// Kamera başlatma
bool startCamera() {
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sccb_sda = SIOD_GPIO_NUM;
  config.pin_sccb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 30000000;
  config.pixel_format = PIXFORMAT_JPEG;
  config.fb_location = CAMERA_FB_IN_PSRAM;
  config.frame_size = FRAMESIZE_VGA;
  config.jpeg_quality = 10;
  config.fb_count = 3;
  config.grab_mode = CAMERA_GRAB_LATEST;

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Kamera hatası: 0x%x\n", err);
    return false;
  }

  sensor_t *s = esp_camera_sensor_get();
  s->set_vflip(s, 0);
  s->set_hmirror(s, 0);
  s->set_brightness(s, 0);
  s->set_saturation(s, 0);
  s->set_contrast(s, 0);
  s->set_denoise(s, 0);
  s->set_special_effect(s, 0);
  
  s->set_exposure_ctrl(s, 1);       // Disable auto-exposure
  s->set_ae_level(s, 2);            // Set exposure level to 0
  
  s->set_awb_gain(s, 1);
  s->set_wb_mode(s, 0);
  s->set_aec2(s, 0);
  s->set_gainceiling(s, GAINCEILING_16X);
  return true;
}

// Kamera görev fonksiyonu
void cameraTask(void *pv) {
  while (1) {
    if (cameraActive && ws.count() > 0) {
      unsigned long now = millis();
      if (now - lastFrameTime > (1000 / STREAM_FPS)) {
        camera_fb_t *fb = esp_camera_fb_get();
        if (fb) {
          lastFrameTime = now;
          if (fb->len < 60000) {
            ws.binaryAll(fb->buf, fb->len);
          }
          esp_camera_fb_return(fb);
        } else {
          Serial.println("Frame alınamadı! Kamera yeniden başlatılıyor...");
          esp_camera_deinit();
          delay(100);
          cameraActive = startCamera();
        }
      }
    }
    vTaskDelay(1);
  }
}

// HTML arayüzü
const char* HTML_PAGE = R"=====(
// html arayüzünü paylaşmıyorum 
)=====";

void onWsEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type, void *arg, uint8_t *data, size_t len) {
  switch (type) {
    case WS_EVT_CONNECT:
      Serial.printf("İstemci #%u bağlandı\n", client->id());
      break;
    case WS_EVT_DISCONNECT:
      Serial.printf("İstemci #%u ayrıldı\n", client->id());
      break;
    case WS_EVT_ERROR:
      Serial.printf("WS hatası #%u\n", client->id());
      break;
    case WS_EVT_DATA:
      AwsFrameInfo *info = (AwsFrameInfo*)arg;
      if (info->final && info->index == 0 && info->len == len) {
        if (info->opcode == WS_TEXT) {
          data[len] = 0;
          
          StaticJsonDocument<512> doc;
          DeserializationError error = deserializeJson(doc, (char*)data);
          if (error == DeserializationError::Ok) {
            if (doc.containsKey("joy")) {
              JsonObject joy = doc["joy"];
              int pan = joy["pan"];
              int tilt = joy["tilt"];
              panAngle = constrain(pan, 0, 180);
              tiltAngle = constrain(tilt, 0, 180);
            }
            
            if (doc.containsKey("flash")) {
              flashState = doc["flash"];
              digitalWrite(FLASH_GPIO_NUM, flashState ? HIGH : LOW);
            }
            
            if (doc.containsKey("settings")) {
              JsonObject settings = doc["settings"];
              sensor_t *s = esp_camera_sensor_get();
              
              if (settings.containsKey("brightness")) 
                s->set_brightness(s, settings["brightness"]);
              
              if (settings.containsKey("contrast")) 
                s->set_contrast(s, settings["contrast"]);
              
              if (settings.containsKey("denoise")) 
                s->set_denoise(s, settings["denoise"]);
              
              if (settings.containsKey("effect")) 
                s->set_special_effect(s, settings["effect"]);
              
              if (settings.containsKey("nightMode")) {
                int nightMode = settings["nightMode"];
                s->set_aec2(s, nightMode);
              }
              
              if (settings.containsKey("whiteBalance")) {
                int wb = settings["whiteBalance"];
                s->set_awb_gain(s, wb == 0 ? 1 : 0);
                if (wb != 0) {
                  s->set_wb_mode(s, wb);
                }
              }
            }
          } else {
            Serial.print("JSON ayrıştırma hatası: ");
            Serial.println(error.c_str());
          }
        }
      }
      break;
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(FLASH_GPIO_NUM, OUTPUT);
  digitalWrite(FLASH_GPIO_NUM, LOW);
  
  servoPan.attach(SERVO_PAN_PIN);
  servoTilt.attach(SERVO_TILT_PIN);
  servoPan.write(panAngle);
  servoTilt.write(tiltAngle);

  WiFi.softAP(AP_SSID, AP_PASSWORD, WIFI_CHANNEL, 0, MAX_CLIENTS);
  WiFi.setSleep(false);
  esp_wifi_set_protocol(WIFI_IF_AP, WIFI_PROTOCOL);
  esp_wifi_set_bandwidth(WIFI_IF_AP, WIFI_BW_HT20);
  WiFi.setTxPower(WIFI_POWER_19_5dBm);
  
  Serial.printf("AP: %s Kanal: %d IP: %s\n", AP_SSID, WIFI_CHANNEL, WiFi.softAPIP().toString().c_str());

  cameraActive = startCamera();

  ws.onEvent(onWsEvent);
  server.addHandler(&ws);

  server.on("/", HTTP_GET, [](AsyncWebServerRequest *req){
    req->send(200, "text/html", HTML_PAGE);
  });

  server.begin();

  xTaskCreatePinnedToCore(
    cameraTask,
    "CameraStream",
    8192,
    NULL,
    1,
    &cameraTaskHandle,
    1
  );
}

void loop() {
  ws.cleanupClients();
  servoPan.write(panAngle);
  servoTilt.write(tiltAngle);
  delay(1);
}

