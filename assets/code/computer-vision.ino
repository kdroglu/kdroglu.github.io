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
<!DOCTYPE html>
<html>
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1.0, maximum-scale=1.0, user-scalable=no">
  <title>ESP32-CAM Pan/Tilt</title>
  <script src="https://cdn.jsdelivr.net/npm/@tensorflow/tfjs@3.11.0/dist/tf.min.js"></script>
  <script src="https://cdn.jsdelivr.net/npm/@tensorflow-models/coco-ssd@2.2.2"></script>
  <script src="https://cdn.jsdelivr.net/npm/@tensorflow-models/blazeface@0.0.7/dist/blazeface.min.js"></script>
  
  <style>
    * { box-sizing: border-box; }
    body { 
      font-family: 'Segoe UI', Tahoma, Geneva, Verdana, sans-serif;
      text-align: center; 
      background: linear-gradient(135deg, #1a2a6c, #b21f1f, #1a2a6c);
      margin: 0; 
      padding: 10px; 
      color: white;
      min-height: 100vh;
      overflow-x: hidden;
      touch-action: manipulation;
    }
    .container { 
      width: 100%;
      max-width: 100%;
      margin: 0 auto; 
      background: rgba(0, 0, 20, 0.85); 
      padding: 15px; 
      border-radius: 15px; 
      box-shadow: 0 5px 15px rgba(0,0,0,0.6);
      backdrop-filter: blur(10px);
      border: 1px solid rgba(255,255,255,0.1);
    }
    .video-container { 
      position: relative; 
      width: 100%;
      max-width: 100%;
      margin: 0 auto 15px; 
      border: 2px solid #4cc9f0;
      border-radius: 10px;
      overflow: hidden;
      background: #000;
      aspect-ratio: 4/3;
    }
    #frame { 
      display: block; 
      width: 100%;
      height: 100%;
      object-fit: cover;
    }
    #objectCanvas, #faceCanvas {
      position: absolute;
      top: 0;
      left: 0;
      width: 100%;
      height: 100%;
      pointer-events: none;
    }
    #objectCanvas { z-index: 5; }
    #faceCanvas { z-index: 6; }
    .controls { 
      background: rgba(30, 40, 80, 0.7); 
      padding: 15px; 
      border-radius: 12px; 
      margin-bottom: 15px;
      border: 1px solid rgba(100, 200, 255, 0.2);
    }
    .joystick-container {
      position: relative;
      width: 200px;
      height: 200px;
      margin: 20px auto;
      background: rgba(100, 100, 100, 0.2);
      border-radius: 50%;
    }
    .joystick {
      position: absolute;
      width: 60px;
      height: 60px;
      background: rgba(76, 201, 240, 0.8);
      border-radius: 50%;
      top: 70px;
      left: 70px;
      cursor: move;
      box-shadow: 0 0 10px rgba(76, 201, 240, 0.5);
    }
    .btn-container {
      display: flex;
      justify-content: center;
      gap: 8px;
      margin-top: 15px;
      flex-wrap: wrap;
    }
    .btn {
      padding: 8px 12px;
      font-size: 0.85rem;
      border: none;
      border-radius: 30px;
      cursor: pointer;
      transition: all 0.3s;
      font-weight: bold;
      min-width: 90px;
      box-shadow: 0 2px 6px rgba(0,0,0,0.3);
    }
    #flashBtn { background: linear-gradient(135deg, #ffd166, #ff9a00); color: #1a2a6c; }
    #flashBtn.active { background: linear-gradient(135deg, #ff9a00, #ff3c00); color: white; box-shadow: 0 0 12px #ff9a00; }
    #objectBtn { background: linear-gradient(135deg, #06d6a0, #118ab2); color: white; }
    #objectBtn.active { background: linear-gradient(135deg, #118ab2, #073b4c); box-shadow: 0 0 12px #06d6a0; }
    #faceBtn { background: linear-gradient(135deg, #ef476f, #ff0054); color: white; }
    #faceBtn.active { background: linear-gradient(135deg, #ff0054, #9d0208); box-shadow: 0 0 12px #ef476f; }
    #trackBtn { background: linear-gradient(135deg, #9b5de5, #7209b7); color: white; }
    #trackBtn.active { background: linear-gradient(135deg, #7209b7, #560bad); box-shadow: 0 0 12px #9b5de5; }
    .connection { 
      color: #a0d2ff; 
      font-size: 0.85rem; 
      margin-top: 15px;
      padding: 6px;
      background: rgba(0, 30, 60, 0.6);
      border-radius: 8px;
      line-height: 1.4;
    }
    .fps { 
      position: absolute; 
      top: 8px; 
      right: 8px; 
      background: rgba(0,0,0,0.7); 
      color: #4cc9f0; 
      padding: 3px 6px; 
      font-size: 0.85rem; 
      border-radius: 4px;
      font-weight: bold;
      z-index: 10;
    }
    .settings-btn {
      position: absolute;
      top: 8px;
      left: 8px;
      background: rgba(0,0,0,0.7);
      color: #4cc9f0;
      padding: 5px 8px;
      border-radius: 4px;
      cursor: pointer;
      font-size: 0.85rem;
      z-index: 10;
    }
    .settings-panel {
      position: fixed;
      top: 0;
      right: -300px;
      width: 280px;
      height: 100%;
      background: rgba(0, 10, 30, 0.95);
      padding: 15px;
      z-index: 100;
      transition: right 0.4s ease;
      overflow-y: auto;
      box-shadow: -5px 0 15px rgba(0,0,0,0.5);
      backdrop-filter: blur(10px);
    }
    .settings-panel.open { right: 0; }
    .settings-panel h3 {
      color: #4cc9f0;
      margin-top: 0;
      border-bottom: 1px solid #4cc9f0;
      padding-bottom: 10px;
      font-size: 1.2rem;
    }
    .close-btn {
      position: absolute;
      top: 10px;
      right: 10px;
      background: none;
      border: none;
      color: #fff;
      font-size: 1.5rem;
      cursor: pointer;
    }
    .settings-overlay {
      display: none;
      position: fixed;
      top: 0;
      left: 0;
      width: 100%;
      height: 100%;
      background: rgba(0,0,0,0.5);
      z-index: 99;
    }
    .settings-overlay.open { display: block; }
    .slider-container { margin: 12px auto; width: 100%; }
    .slider-label { 
      display: flex; 
      justify-content: space-between; 
      margin-bottom: 6px; 
      font-size: 0.9rem;
    }
    input[type=range] { 
      width: 100%; 
      height: 25px; 
      -webkit-appearance: none;
      background: linear-gradient(90deg, #1a2a6c, #4cc9f0, #1a2a6c);
      border-radius: 12px;
      outline: none;
      border: 1px solid rgba(100, 200, 255, 0.3);
    }
    input[type=range]::-webkit-slider-thumb {
      -webkit-appearance: none;
      width: 25px;
      height: 25px;
      background: #fff;
      border-radius: 50%;
      cursor: pointer;
      box-shadow: 0 2px 6px rgba(0,0,0,0.4);
      border: 2px solid #4cc9f0;
    }
    .effect-btn {
      display: inline-block;
      padding: 6px 12px;
      margin: 4px;
      background: rgba(76, 201, 240, 0.2);
      border: 1px solid #4cc9f0;
      border-radius: 18px;
      cursor: pointer;
      transition: all 0.2s;
      font-size: 0.85rem;
    }
    .effect-btn.active {
      background: #4cc9f0;
      color: #1a2a6c;
      font-weight: bold;
    }
    .object-info, .face-info {
      position: absolute;
      bottom: 8px;
      background: rgba(0,0,0,0.7);
      padding: 4px 8px;
      border-radius: 4px;
      font-size: 0.85rem;
      max-width: 70%;
      z-index: 10;
    }
    .object-info { left: 8px; color: #06d6a0; }
    .face-info { right: 8px; color: #ef476f; }
    .model-status {
      position: absolute;
      top: 40px;
      left: 8px;
      background: rgba(0,0,0,0.7);
      color: #ffd166;
      padding: 4px 8px;
      border-radius: 4px;
      font-size: 0.75rem;
      z-index: 10;
    }
    .track-box {
      position: absolute;
      border: 3px solid #9b5de5;
      box-shadow: 0 0 15px #9b5de5;
      pointer-events: none;
      z-index: 15;
      display: none;
    }
    .track-status {
      position: absolute;
      top: 8px;
      right: 100px;
      background: rgba(0,0,0,0.7);
      color: #9b5de5;
      padding: 4px 8px;
      border-radius: 4px;
      font-size: 0.85rem;
      z-index: 10;
    }
    .debug-info {
      position: absolute;
      top: 60px;
      left: 8px;
      background: rgba(0,0,0,0.7);
      color: #ff9a00;
      padding: 4px 8px;
      border-radius: 4px;
      font-size: 0.75rem;
      z-index: 10;
    }
    .reticle {
      position: absolute;
      top: 50%;
      left: 50%;
      transform: translate(-50%, -50%);
      width: 30px;
      height: 30px;
      border: 2px solid rgba(255, 0, 0, 0.7);
      border-radius: 50%;
      z-index: 20;
      pointer-events: none;
    }
    .reticle::before, .reticle::after {
      content: '';
      position: absolute;
      background: rgba(255, 255, 255, 0.5);
    }
    .reticle::before {
      top: 50%;
      left: 2px;
      right: 2px;
      height: 1px;
      transform: translateY(-50%);
    }
    .reticle::after {
      left: 50%;
      top: 2px;
      bottom: 2px;
      width: 1px;
      transform: translateX(-50%);
    }
    .reticle.active { border-color: #00ff00; }
  </style>
</head>
<body>
  <div class="container">
    <div class="video-container">
      <img id="frame" src="">
      <canvas id="objectCanvas"></canvas>
      <canvas id="faceCanvas"></canvas>
      <div class="fps">FPS: <span id="fps">0</span></div>
      <div class="model-status" id="modelStatus">Modeller yükleniyor...</div>
      <div class="track-status" id="trackStatus">Takip Kapalı</div>
      <div class="debug-info" id="debugInfo"></div>
      <div class="object-info" id="objectInfo"></div>
      <div class="face-info" id="faceInfo"></div>
      <div class="settings-btn" id="settingsBtn"><i class="fas fa-cog"></i> AYARLAR</div>
      <div class="track-box" id="trackBox"></div>
      <div class="reticle" id="reticle"></div>
    </div>
    
    <div class="controls">
      <div class="joystick-container" id="joystickContainer">
        <div class="joystick" id="joystick"></div>
      </div>
      
      <div class="btn-container">
        <button id="flashBtn" class="btn">FLASH</button>
        <button id="objectBtn" class="btn">OBJE</button>
        <button id="faceBtn" class="btn">YÜZ</button>
        <button id="trackBtn" class="btn">TAKİP</button>
      </div>
    </div>
    
    <div class="connection">
      IP: <span id="ip">Yükleniyor...</span> | Bağlantı: <span id="connStatus">Bağlanıyor...</span>
    </div>
  </div>

  <div class="settings-overlay" id="settingsOverlay"></div>
  <div class="settings-panel" id="settingsPanel">
    <button class="close-btn" id="closeSettings">&times;</button>
    <h3><i class="fas fa-sliders-h"></i> Kamera Ayarları</h3>
    
    <div class="slider-container">
      <div class="slider-label">
        <span><i class="fas fa-adjust"></i> Parlaklık</span>
        <span id="brightnessVal">0</span>
      </div>
      <input type="range" id="brightness" min="-2" max="2" value="0">
    </div>
    
    <div class="slider-container">
      <div class="slider-label">
        <span><i class="fas fa-contrast"></i> Kontrast</span>
        <span id="contrastVal">0</span>
      </div>
      <input type="range" id="contrast" min="-2" max="2" value="0">
    </div>
    
    <div class="slider-container">
      <div class="slider-label">
        <span><i class="fas fa-snowflake"></i> Gürültü Azaltma</span>
        <span id="denoiseVal">KAPALI</span>
      </div>
      <input type="range" id="denoise" min="0" max="1" value="0">
    </div>
    
    <h4><i class="fas fa-magic"></i> Özel Efektler</h4>
    <div class="btn-group" data-group="effect">
      <div class="effect-btn active" data-value="0">Normal</div>
      <div class="effect-btn" data-value="1">Negatif</div>
      <div class="effect-btn" data-value="2">Gri Ton</div>
      <div class="effect-btn" data-value="3">Kırmızı</div>
      <div class="effect-btn" data-value="4">Yeşil</div>
      <div class="effect-btn" data-value="5">Mavi</div>
      <div class="effect-btn" data-value="6">Sepya</div>
    </div>
    
    <h4><i class="fas fa-moon"></i> Gece Modu</h4>
    <div class="btn-group" data-group="nightMode">
      <div class="effect-btn active" data-value="0">Kapalı</div>
      <div class="effect-btn" data-value="1">Açık</div>
    </div>
    
    <h4><i class="fas fa-sun"></i> Beyaz Dengesi</h4>
    <div class="btn-group" data-group="whiteBalance">
      <div class="effect-btn active" data-value="0">Otomatik</div>
      <div class="effect-btn" data-value="1">Güneşli</div>
      <div class="effect-btn" data-value="2">Bulutlu</div>
      <div class="effect-btn" data-value="3">Ofis</div>
      <div class="effect-btn" data-value="4">Ev</div>
    </div>
  </div>

  <script>
    const frame = document.getElementById('frame');
    const objectCanvas = document.getElementById('objectCanvas');
    const objectCtx = objectCanvas.getContext('2d');
    const faceCanvas = document.getElementById('faceCanvas');
    const faceCtx = faceCanvas.getContext('2d');
    const connStatus = document.getElementById('connStatus');
    const ipSpan = document.getElementById('ip');
    const fpsSpan = document.getElementById('fps');
    const flashBtn = document.getElementById('flashBtn');
    const objectBtn = document.getElementById('objectBtn');
    const faceBtn = document.getElementById('faceBtn');
    const trackBtn = document.getElementById('trackBtn');
    const objectInfo = document.getElementById('objectInfo');
    const faceInfo = document.getElementById('faceInfo');
    const modelStatus = document.getElementById('modelStatus');
    const trackStatus = document.getElementById('trackStatus');
    const debugInfo = document.getElementById('debugInfo');
    const trackBox = document.getElementById('trackBox');
    const settingsBtn = document.getElementById('settingsBtn');
    const settingsPanel = document.getElementById('settingsPanel');
    const settingsOverlay = document.getElementById('settingsOverlay');
    const closeSettings = document.getElementById('closeSettings');
    const brightnessSlider = document.getElementById('brightness');
    const brightnessVal = document.getElementById('brightnessVal');
    const contrastSlider = document.getElementById('contrast');
    const contrastVal = document.getElementById('contrastVal');
    const denoiseSlider = document.getElementById('denoise');
    const denoiseVal = document.getElementById('denoiseVal');
    const effectBtns = document.querySelectorAll('.effect-btn');
    const joystick = document.getElementById('joystick');
    const joystickContainer = document.getElementById('joystickContainer');
    const reticle = document.getElementById('reticle');

    let frameCount = 0;
    let startTime = Date.now();
    let lastFpsUpdate = 0;
    let socket;
    let lastSent = 0;
    const THROTTLE_MS = 0; // 5'ten 2'ye değiştirildi
    let reconnectTimeout;
    let objectModel = null;
    let faceModel = null;
    let objectDetectionActive = false;
    let faceDetectionActive = false;
    let trackingActive = false;
    const TRACK_INTERVAL = 0;
    let lastTrackTime = 0;
    const OBJECT_DETECTION_FPS = 30; // Obje tanıma FPS'i 60
    const FACE_DETECTION_FPS = 30; // Yüz tanıma FPS'i 30
    let lastObjectDetectionTime = 0;
    let lastFaceDetectionTime = 0;
    let facePredictions = [];
    let faceDetected = false;
    let objectPredictions = [];
    let objectDetected = false;
    let lastObjectCenterX = 0;
    let lastObjectCenterY = 0;
    let trackingMode = null;
    let trackingObjectClass = 'person';
    let isDragging = false;
    let panAngle = 90;
    let tiltAngle = 90;

    ipSpan.textContent = window.location.hostname || '192.168.4.1';

    async function loadModels() {
      try {
        modelStatus.textContent = "Modeller yükleniyor...";
        await tf.setBackend('webgl');
        await tf.ready();
        objectModel = await cocoSsd.load({ base: 'lite_mobilenet_v2' });
        faceModel = await blazeface.load();
        modelStatus.textContent = "Modeller yüklendi!";
      } catch (error) {
        modelStatus.textContent = "Model hatası: " + error.message;
      }
    }

    async function detectObjects() {
      if (!objectDetectionActive || !objectModel || frame.src === '') return;
      
      const now = Date.now();
      if (now - lastObjectDetectionTime < (1000 / OBJECT_DETECTION_FPS)) {
        requestAnimationFrame(detectObjects);
        return;
      }
      lastObjectDetectionTime = now;
      
      try {
        objectCanvas.width = frame.clientWidth;
        objectCanvas.height = frame.clientHeight;
        objectCtx.clearRect(0, 0, objectCanvas.width, objectCanvas.height);
        
        const predictions = await objectModel.detect(frame);
        objectPredictions = predictions;
        
        let infoText = '';
        let highestConfidence = 0;
        let bestObject = null;
        
        predictions.forEach(prediction => {
          const [x, y, width, height] = prediction.bbox;
          
          if (trackingActive && trackingMode === 'object' && 
              prediction.class.toLowerCase() === trackingObjectClass && 
              prediction.score > highestConfidence) {
            highestConfidence = prediction.score;
            bestObject = prediction;
            
            objectCtx.strokeStyle = '#9b5de5';
            objectCtx.lineWidth = 3;
            objectCtx.beginPath();
            objectCtx.rect(x-2, y-2, width+4, height+4);
            objectCtx.stroke();
          }
          
          objectCtx.strokeStyle = '#06d6a0';
          objectCtx.lineWidth = 2;
          objectCtx.beginPath();
          objectCtx.rect(x, y, width, height);
          objectCtx.stroke();
          
          objectCtx.fillStyle = 'rgba(6, 214, 160, 0.5)';
          objectCtx.fillRect(x, y - 20, width, 20);
          
          objectCtx.fillStyle = 'white';
          objectCtx.font = '14px Arial';
          objectCtx.fillText(`${prediction.class} (${Math.round(prediction.score * 100)}%)`, x + 5, y - 5);
          
          infoText += `${prediction.class} (${Math.round(prediction.score * 100)}%) `;
        });
        
        if (bestObject) {
          const [x, y, w, h] = bestObject.bbox;
          lastObjectCenterX = x + w/2;
          lastObjectCenterY = y + h/2;
          objectDetected = true;
          updateTrackBox(x, y, w, h);
        } else {
          objectDetected = false;
        }
        
        objectInfo.textContent = infoText;
      } catch (error) {
        console.error('Obje tanıma hatası:', error);
      }
      
      if (objectDetectionActive) {
        requestAnimationFrame(detectObjects);
      }
    }

    async function detectFaces() {
      if (!faceDetectionActive || !faceModel || frame.src === '') return;
      
      const now = Date.now();
      if (now - lastFaceDetectionTime < (1000 / FACE_DETECTION_FPS)) {
        requestAnimationFrame(detectFaces);
        return;
      }
      lastFaceDetectionTime = now;
      
      try {
        faceCanvas.width = frame.clientWidth;
        faceCanvas.height = frame.clientHeight;
        facePredictions = await faceModel.estimateFaces(frame, false);
        faceDetected = facePredictions.length > 0;
      } catch (error) {
        console.error('Yüz tanıma hatası:', error);
        faceDetected = false;
      }
      
      if (faceDetectionActive) {
        requestAnimationFrame(detectFaces);
      }
    }

    function drawFaces() {
      if (!faceDetectionActive) return;
      
      faceCtx.clearRect(0, 0, faceCanvas.width, faceCanvas.height);
      let faceCount = 0;
      
      facePredictions.forEach(prediction => {
        const start = prediction.topLeft;
        const end = prediction.bottomRight;
        const size = [end[0] - start[0], end[1] - start[1]];
        
        if (faceCount === 0 && trackingMode === 'face') {
          trackX = start[0];
          trackY = start[1];
          trackW = size[0];
          trackH = size[1];
          updateTrackBox(trackX, trackY, trackW, trackH);
        }
        
        faceCtx.strokeStyle = '#ef476f';
        faceCtx.lineWidth = 2;
        faceCtx.beginPath();
        faceCtx.rect(start[0], start[1], size[0], size[1]);
        faceCtx.stroke();
        
        faceCtx.fillStyle = 'rgba(239, 71, 111, 0.5)';
        faceCtx.fillRect(start[0], start[1] - 20, size[0], 20);
        
        faceCtx.fillStyle = 'white';
        faceCtx.font = '14px Arial';
        faceCtx.fillText('Yüz', start[0] + 5, start[1] - 5);
        
        faceCount++;
      });
      
      faceInfo.textContent = faceCount > 0 ? `${faceCount} yüz bulundu` : '';
      
      if (faceCount === 0 && trackingMode === 'face') {
        trackBox.style.display = 'none';
      }
      
      requestAnimationFrame(drawFaces);
    }

    function updateTrackBox(x, y, w, h) {
      trackBox.style.display = 'block';
      trackBox.style.left = x + 'px';
      trackBox.style.top = y + 'px';
      trackBox.style.width = w + 'px';
      trackBox.style.height = h + 'px';
    }

    function trackTarget() {
      if (!trackingActive) return;
      
      const now = Date.now();
      if (now - lastTrackTime < TRACK_INTERVAL) {
        requestAnimationFrame(trackTarget);
        return;
      }
      lastTrackTime = now;
      
      let centerX, centerY;
      const frameWidth = frame.clientWidth;
      const frameHeight = frame.clientHeight;
      const screenCenterX = frameWidth / 2;
      const screenCenterY = frameHeight / 2;
      
      if (trackingMode === 'face' && faceDetected) {
        centerX = trackX + trackW / 2;
        centerY = trackY + trackH / 2;
        trackBox.style.display = 'block';
      } 
      else if (trackingMode === 'object' && objectDetected) {
        centerX = lastObjectCenterX;
        centerY = lastObjectCenterY;
        trackBox.style.display = 'block';
      } 
      else {
        trackBox.style.display = 'none';
        debugInfo.textContent = "Hedef bulunamadı";
        reticle.classList.remove('active');
        requestAnimationFrame(trackTarget);
        return;
      }
      
      let diffX = centerX - screenCenterX;
      let diffY = centerY - screenCenterY;
      const distance = Math.sqrt(diffX * diffX + diffY * diffY);
      
      debugInfo.textContent = `P:${Math.round(panAngle)} T:${Math.round(tiltAngle)} X:${Math.round(centerX)} Y:${Math.round(centerY)} D:${Math.round(distance)} | ${TRACK_INTERVAL}ms`;
      
      if (distance < 12) {
        reticle.classList.add('active');
        diffX = 0;
        diffY = 0;
      } else {
        reticle.classList.remove('active');
      }
      
      const sensitivity_x = 0.06;
      const sensitivity_y = 0.05;
      const moveX = diffX * sensitivity_x / frameWidth;
      const moveY = diffY * sensitivity_y / frameHeight;
      
      const newPan = panAngle + moveX * 90;
      const newTilt = tiltAngle - moveY * 90;
      
      panAngle = Math.max(0, Math.min(180, newPan));
      tiltAngle = Math.max(0, Math.min(180, newTilt));
      
      sendJoystickCmd(panAngle, tiltAngle);
      requestAnimationFrame(trackTarget);
    }

    function initWebSocket() {
      clearTimeout(reconnectTimeout);
      
      socket = new WebSocket('ws://' + window.location.hostname + '/ws');
      socket.binaryType = 'arraybuffer';
      
      socket.onopen = () => {
        connStatus.textContent = 'BAĞLANDI';
      };
      
      socket.onmessage = (evt) => {
        if (evt.data instanceof ArrayBuffer) {
          const blob = new Blob([evt.data], { type: 'image/jpeg' });
          const url = URL.createObjectURL(blob);
          
          frame.onload = () => {
            URL.revokeObjectURL(url);
            frameCount++;
            const now = Date.now();
            const elapsed = (now - startTime) / 1000;
            
            if (now - lastFpsUpdate >= 1000) {
              fpsSpan.textContent = Math.round(frameCount / elapsed * 10) / 10;
              frameCount = 0;
              startTime = now;
              lastFpsUpdate = now;
            }
          };
          
          frame.src = url;
        }
      };
      
      socket.onclose = () => {
        connStatus.textContent = 'BAĞLANTI KAPANDI - Yeniden bağlanılıyor...';
        reconnectTimeout = setTimeout(initWebSocket, 2000);
      };
      
      socket.onerror = (error) => {
        console.error('WebSocket Hatası:', error);
        socket.close();
      };
    }

    flashBtn.addEventListener('click', () => {
      const newState = !flashBtn.classList.contains('active');
      flashBtn.classList.toggle('active', newState);
      flashBtn.textContent = newState ? 'FLASH AÇIK' : 'FLASH KAPALI';
      if (socket && socket.readyState === WebSocket.OPEN) {
        socket.send(JSON.stringify({ flash: newState }));
      }
    });

    objectBtn.addEventListener('click', () => {
      const newState = !objectBtn.classList.contains('active');
      
      if (newState && trackingActive) {
        faceDetectionActive = false;
        faceBtn.classList.remove('active');
      }
      
      objectBtn.classList.toggle('active', newState);
      objectBtn.textContent = newState ? 'OBJE AÇIK' : 'OBJE KAPALI';
      objectDetectionActive = newState;
      
      if (newState) {
        detectObjects();
      } else {
        objectInfo.textContent = '';
        objectCtx.clearRect(0, 0, objectCanvas.width, objectCanvas.height);
      }
    });

    faceBtn.addEventListener('click', () => {
      const newState = !faceBtn.classList.contains('active');
      
      if (newState && trackingActive) {
        objectDetectionActive = false;
        objectBtn.classList.remove('active');
      }
      
      faceBtn.classList.toggle('active', newState);
      faceBtn.textContent = newState ? 'YÜZ AÇIK' : 'YÜZ KAPALI';
      faceDetectionActive = newState;
      
      if (newState) {
        detectFaces();
        drawFaces();
      } else {
        faceInfo.textContent = '';
        faceCtx.clearRect(0, 0, faceCanvas.width, faceCanvas.height);
      }
    });

    trackBtn.addEventListener('click', () => {
      const newState = !trackBtn.classList.contains('active');
      trackBtn.classList.toggle('active', newState);
      trackingActive = newState;
      
      if (newState) {
        if (faceDetectionActive) {
          trackingMode = 'face';
          objectDetectionActive = false;
          objectBtn.classList.remove('active');
          trackStatus.textContent = "Yüz Takibi Aktif";
        } 
        else if (objectDetectionActive) {
          trackingMode = 'object';
          faceDetectionActive = false;
          faceBtn.classList.remove('active');
          
          const className = prompt("Takip edilecek objeyi girin (Örnek: person, car, etc.):", "person");
          trackingObjectClass = className ? className.toLowerCase() : 'person';
          trackStatus.textContent = `Obje Takibi: ${trackingObjectClass}`;
        } 
        else {
          alert("Takip için önce obje veya yüz tanıma açmalısınız!");
          trackBtn.classList.remove('active');
          trackingActive = false;
          return;
        }
        
        trackStatus.style.color = "#9b5de5";
        lastTrackTime = Date.now();
        trackTarget();
      } else {
        trackingMode = null;
        trackStatus.textContent = "Takip Kapalı";
        trackStatus.style.color = "";
        trackBox.style.display = 'none';
        debugInfo.textContent = "";
        reticle.classList.remove('active');
      }
    });

    function setupJoystick() {
      joystick.addEventListener('mousedown', startDrag);
      joystick.addEventListener('touchstart', startDrag);
      
      function startDrag(e) {
        e.preventDefault();
        isDragging = true;
        document.addEventListener('mousemove', moveJoystick);
        document.addEventListener('touchmove', moveJoystick);
        document.addEventListener('mouseup', stopDrag);
        document.addEventListener('touchend', stopDrag);
      }
      
      function moveJoystick(e) {
        if (!isDragging) return;
        
        const rect = joystickContainer.getBoundingClientRect();
        const radius = rect.width / 2;
        const centerX = rect.left + radius;
        const centerY = rect.top + radius;
        
        let clientX, clientY;
        if (e.type === 'touchmove') {
          clientX = e.touches[0].clientX;
          clientY = e.touches[0].clientY;
        } else {
          clientX = e.clientX;
          clientY = e.clientY;
        }
        
        const deltaX = clientX - centerX;
        const deltaY = clientY - centerY;
        const distance = Math.min(radius, Math.sqrt(deltaX * deltaX + deltaY * deltaY));
        const angle = Math.atan2(deltaY, deltaX);
        
        const newX = Math.cos(angle) * distance;
        const newY = Math.sin(angle) * distance;
        
        joystick.style.left = (radius + newX - 30) + 'px';
        joystick.style.top = (radius + newY - 30) + 'px';
        
        const pan = Math.round(90 + (newX / radius) * 90);
        const tilt = Math.round(90 - (newY / radius) * 90);
        
        panAngle = pan;
        tiltAngle = tilt;
        
        sendJoystickCmd(pan, tilt);
      }
      
      function stopDrag() {
        if (!isDragging) return;
        isDragging = false;
        document.removeEventListener('mousemove', moveJoystick);
        document.removeEventListener('touchmove', moveJoystick);
      }
    }
    
    function sendJoystickCmd(pan, tilt) {
      if (!socket || socket.readyState !== WebSocket.OPEN) return;
      
      const now = Date.now();
      if (now - lastSent < THROTTLE_MS) return;
      
      lastSent = now;
      
      const adjustedPan = 180 - pan;
      
      socket.send(JSON.stringify({ 
        joy: { 
          pan: adjustedPan,
          tilt: tilt 
        } 
      }));
    }
    
    function sendCameraSettings() {
      if (!socket || socket.readyState !== WebSocket.OPEN) return;
      
      const settings = {
        brightness: parseInt(brightnessSlider.value),
        contrast: parseInt(contrastSlider.value),
        denoise: parseInt(denoiseSlider.value),
        effect: document.querySelector('.btn-group[data-group="effect"] .effect-btn.active')?.dataset.value || 0,
        nightMode: parseInt(document.querySelector('.btn-group[data-group="nightMode"] .effect-btn.active')?.dataset.value || 0),
        whiteBalance: parseInt(document.querySelector('.btn-group[data-group="whiteBalance"] .effect-btn.active')?.dataset.value || 0)
      };
      
      socket.send(JSON.stringify({ settings: settings }));
    }
    
    settingsBtn.addEventListener('click', () => {
      settingsPanel.classList.add('open');
      settingsOverlay.classList.add('open');
    });
    
    closeSettings.addEventListener('click', () => {
      settingsPanel.classList.remove('open');
      settingsOverlay.classList.remove('open');
    });
    
    settingsOverlay.addEventListener('click', closeSettings);
    
    brightnessSlider.addEventListener('input', () => {
      brightnessVal.textContent = brightnessSlider.value;
      sendCameraSettings();
    });
    
    contrastSlider.addEventListener('input', () => {
      contrastVal.textContent = contrastSlider.value;
      sendCameraSettings();
    });
    
    denoiseSlider.addEventListener('input', () => {
      denoiseVal.textContent = denoiseSlider.value === '1' ? 'AÇIK' : 'KAPALI';
      sendCameraSettings();
    });
    
    document.querySelectorAll('.effect-btn').forEach(btn => {
      btn.addEventListener('click', function() {
        const group = this.closest('.btn-group');
        const buttons = group.querySelectorAll('.effect-btn');
        
        buttons.forEach(b => b.classList.remove('active'));
        this.classList.add('active');
        
        sendCameraSettings();
      });
    });
    
    initWebSocket();
    setupJoystick();
    loadModels();
    
    const style = document.createElement('style');
    style.innerHTML = `@import url('https://cdnjs.cloudflare.com/ajax/libs/font-awesome/6.4.0/css/all.min.css');`;
    document.head.appendChild(style);
  </script>
</body>
</html>
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
