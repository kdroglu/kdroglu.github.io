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
const char* AP_SSID     = "ESP32CAM_AP";
const char* AP_PASSWORD = "12345678";

// Servo Ayarları
Servo servoPan, servoTilt;
const int SERVO_PAN_PIN  = 14;
const int SERVO_TILT_PIN = 15;
volatile int panAngle  = 90;
volatile int tiltAngle = 90;

// Flash Ayarları
#define FLASH_GPIO_NUM 4
volatile bool flashState = false;

// Kamera pinleri (AI-Thinker)
#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27

#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22

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
  config.ledc_timer   = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM; 
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk    = XCLK_GPIO_NUM;
  config.pin_pclk    = PCLK_GPIO_NUM;
  config.pin_vsync   = VSYNC_GPIO_NUM;
  config.pin_href    = HREF_GPIO_NUM;
  config.pin_sccb_sda= SIOD_GPIO_NUM;
  config.pin_sccb_scl= SIOC_GPIO_NUM;
  config.pin_pwdn    = PWDN_GPIO_NUM;
  config.pin_reset   = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;
  config.fb_location = CAMERA_FB_IN_PSRAM;
  
  // Dengeli Kalite Ayarları
  config.frame_size   = FRAMESIZE_VGA;   // 640x480
  config.jpeg_quality = 10;              // Daha düşük kalite (bağlantı için)
  config.fb_count     = 2;               // Double buffer
  config.grab_mode    = CAMERA_GRAB_LATEST;

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Kamera hatası: 0x%x\n", err);
    return false;
  }
  
  // Sensör ayarları
  sensor_t *s = esp_camera_sensor_get();
  s->set_vflip(s, 0);       // Dikey çevirme
  s->set_hmirror(s, 0);     // Yatay ayna
  s->set_brightness(s, 0);  // Parlaklık
  s->set_saturation(s, 0);  // Doygunluk
  s->set_contrast(s, 0);    // Kontrast
  s->set_denoise(s, 0);     // Gürültü azaltma
  s->set_special_effect(s, 0); // Özel efekt
  s->set_awb_gain(s, 1);    // Otomatik beyaz dengesi (1: açık, 0: kapalı)
  s->set_wb_mode(s, 0);     // Beyaz dengesi modu (0: otomatik)
  s->set_aec2(s, 0);        // Gece modu (0: kapalı, 1: açık)
  
  return true;
}

// Kamera görev fonksiyonu
void cameraTask(void *pv) {
  while(1) {
    if (cameraActive && ws.count() > 0) {
      unsigned long now = millis();
      if (now - lastFrameTime > (1000 / STREAM_FPS)) {
        camera_fb_t *fb = esp_camera_fb_get();
        if (fb) {
          lastFrameTime = now;
          
          // Tüm istemcilere gönder
          if (fb->len < 50000) { // Sadece küçük frameleri gönder
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

// HTML arayüzü (Hata düzeltildi)
const char* HTML_PAGE = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1.0, maximum-scale=1.0, user-scalable=no">
  <title>ESP32-CAM Pan/Tilt</title>
  <style>
    * {
      box-sizing: border-box;
    }
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
      gap: 15px;
      margin-top: 15px;
      flex-wrap: wrap;
    }
    .btn {
      padding: 10px 20px;
      font-size: 1rem;
      border: none;
      border-radius: 40px;
      cursor: pointer;
      transition: all 0.3s;
      font-weight: bold;
      min-width: 120px;
      box-shadow: 0 3px 8px rgba(0,0,0,0.3);
    }
    #flashBtn {
      background: linear-gradient(135deg, #ffd166, #ff9a00);
      color: #1a2a6c;
    }
    #flashBtn.active {
      background: linear-gradient(135deg, #ff9a00, #ff3c00);
      color: white;
      box-shadow: 0 0 15px #ff9a00;
    }
    .connection { 
      color: #a0d2ff; 
      font-size: 0.9rem; 
      margin-top: 15px;
      padding: 8px;
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
      padding: 4px 8px; 
      font-size: 0.9rem; 
      border-radius: 4px;
      font-weight: bold;
    }
    .settings-btn {
      position: absolute;
      top: 8px;
      left: 8px;
      background: rgba(0,0,0,0.7);
      color: #4cc9f0;
      padding: 6px 10px;
      border-radius: 4px;
      cursor: pointer;
      z-index: 10;
    }
    .settings-panel {
      position: fixed;
      top: 0;
      right: -300px;
      width: 280px;
      height: 100%;
      background: rgba(0, 10, 30, 0.95);
      padding: 20px;
      z-index: 100;
      transition: right 0.4s ease;
      overflow-y: auto;
      box-shadow: -5px 0 15px rgba(0,0,0,0.5);
      backdrop-filter: blur(10px);
    }
    .settings-panel.open {
      right: 0;
    }
    .settings-panel h3 {
      color: #4cc9f0;
      margin-top: 0;
      border-bottom: 1px solid #4cc9f0;
      padding-bottom: 10px;
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
    .settings-overlay.open {
      display: block;
    }
    .slider-container { 
      margin: 15px auto; 
      width: 100%; 
    }
    .slider-label { 
      display: flex; 
      justify-content: space-between; 
      margin-bottom: 8px; 
    }
    input[type=range] { 
      width: 100%; 
      height: 30px; 
      -webkit-appearance: none;
      background: linear-gradient(90deg, #1a2a6c, #4cc9f0, #1a2a6c);
      border-radius: 15px;
      outline: none;
      border: 1px solid rgba(100, 200, 255, 0.3);
    }
    input[type=range]::-webkit-slider-thumb {
      -webkit-appearance: none;
      width: 30px;
      height: 30px;
      background: #fff;
      border-radius: 50%;
      cursor: pointer;
      box-shadow: 0 2px 8px rgba(0,0,0,0.5);
      border: 2px solid #4cc9f0;
    }
    .effect-btn {
      display: inline-block;
      padding: 8px 15px;
      margin: 5px;
      background: rgba(76, 201, 240, 0.2);
      border: 1px solid #4cc9f0;
      border-radius: 20px;
      cursor: pointer;
      transition: all 0.2s;
    }
    .effect-btn.active {
      background: #4cc9f0;
      color: #1a2a6c;
      font-weight: bold;
    }
  </style>
</head>
<body>
  <div class="container">
    <div class="video-container">
      <img id="frame" src="">
      <div class="fps">FPS: <span id="fps">0</span></div>
      <div class="settings-btn" id="settingsBtn">
        <i class="fas fa-cog"></i> AYARLAR
      </div>
    </div>
    
    <div class="controls">
      <div class="joystick-container" id="joystickContainer">
        <div class="joystick" id="joystick"></div>
      </div>
      
      <div class="btn-container">
        <button id="flashBtn" class="btn">FLASH KAPALI</button>
      </div>
    </div>
    
    <div class="connection">
      IP: <span id="ip">Yükleniyor...</span> | Bağlantı: <span id="connStatus">Bağlanıyor...</span>
    </div>
  </div>

  <!-- Ayarlar Paneli -->
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
    // DOM Elements
    const frame = document.getElementById('frame');
    const connStatus = document.getElementById('connStatus');
    const ipSpan = document.getElementById('ip');
    const fpsSpan = document.getElementById('fps');
    const flashBtn = document.getElementById('flashBtn');
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

    // FPS Değişkenleri
    let frameCount = 0;
    let startTime = Date.now();
    
    // WebSocket ve durum değişkenleri
    let socket;
    let lastSent = 0;
    const THROTTLE_MS = 15;
    let reconnectTimeout;
    
    // IP adresini al
    ipSpan.textContent = window.location.hostname || '192.168.4.1';
    
    // WebSocket bağlantısını başlat
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
            
            // FPS hesapla
            frameCount++;
            const now = Date.now();
            const elapsed = (now - startTime) / 1000;
            
            if (elapsed >= 1) {
              fpsSpan.textContent = Math.round(frameCount / elapsed);
              frameCount = 0;
              startTime = now;
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
    
    // Flash kontrolü
    flashBtn.addEventListener('click', () => {
      const newState = !flashBtn.classList.contains('active');
      flashBtn.classList.toggle('active', newState);
      flashBtn.textContent = newState ? 'FLASH AÇIK' : 'FLASH KAPALI';
      if (socket && socket.readyState === WebSocket.OPEN) {
        socket.send(JSON.stringify({ flash: newState }));
      }
    });
    
    // Joystick kontrolü (Tilt yönü düzeltildi)
    function setupJoystick() {
      let isDragging = false;
      
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
        
        // Tilt yönü düzeltildi
        const pan = Math.round(90 + (newX / radius) * 90);
        const tilt = Math.round(90 - (newY / radius) * 90);
        
        sendJoystickCmd(pan, tilt);
      }
      
      function stopDrag() {
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
      
      // Pan için açı ters çevriliyor
      const adjustedPan = 180 - pan;
      
      socket.send(JSON.stringify({ 
        joy: { 
          pan: adjustedPan,
          tilt: tilt 
        } 
      }));
    }
    
    // Kamera ayarlarını gönder
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
    
    // Ayarlar paneli kontrolü
    settingsBtn.addEventListener('click', () => {
      settingsPanel.classList.add('open');
      settingsOverlay.classList.add('open');
    });
    
    closeSettings.addEventListener('click', () => {
      settingsPanel.classList.remove('open');
      settingsOverlay.classList.remove('open');
    });
    
    settingsOverlay.addEventListener('click', closeSettings);
    
    // Kamera ayarları kontrolleri
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
    
    // Efekt butonları (HATA DÜZELTİLDİ)
    document.querySelectorAll('.effect-btn').forEach(btn => {
      btn.addEventListener('click', function() {
        const group = this.closest('.btn-group');
        const buttons = group.querySelectorAll('.effect-btn');
        
        // Sadece tıklanan butonu aktif yap, diğerlerini pasif yap
        buttons.forEach(b => b.classList.remove('active'));
        this.classList.add('active');
        
        sendCameraSettings();
      });
    });
    
    // Başlangıçta WebSocket ve Joystick'i başlat
    initWebSocket();
    setupJoystick();
    
    // Font Awesome ikonları
    const style = document.createElement('style');
    style.innerHTML = `
      @import url('https://cdnjs.cloudflare.com/ajax/libs/font-awesome/6.4.0/css/all.min.css');
    `;
    document.head.appendChild(style);
  </script>
</body>
</html>
)rawliteral";

// WebSocket event işleyici (HATA DÜZELTİLDİ)
void onWsEvent(AsyncWebSocket *server, AsyncWebSocketClient *client,
               AwsEventType type, void *arg, uint8_t *data, size_t len) {
  switch(type) {
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
        if(info->opcode == WS_TEXT) {
          data[len] = 0;
          
          StaticJsonDocument<512> doc;
          DeserializationError error = deserializeJson(doc, (char*)data);
          if (error == DeserializationError::Ok) {
            // Joystick Kontrolü
            if (doc.containsKey("joy")) {
              JsonObject joy = doc["joy"];
              int pan = joy["pan"];
              int tilt = joy["tilt"];
              
              panAngle = constrain(pan, 0, 180);
              tiltAngle = constrain(tilt, 0, 180);
            }
            
            // Flash Kontrolü
            if (doc.containsKey("flash")) {
              flashState = doc["flash"];
              digitalWrite(FLASH_GPIO_NUM, flashState ? HIGH : LOW);
            }
            
            // Kamera Ayarları
            if (doc.containsKey("settings")) {
              JsonObject settings = doc["settings"];
              sensor_t *s = esp_camera_sensor_get();
              
              if(settings.containsKey("brightness")) 
                s->set_brightness(s, settings["brightness"]);
              
              if(settings.containsKey("contrast")) 
                s->set_contrast(s, settings["contrast"]);
              
              if(settings.containsKey("denoise")) 
                s->set_denoise(s, settings["denoise"]);
              
              if(settings.containsKey("effect")) 
                s->set_special_effect(s, settings["effect"]);
              
              // Gece Modu
              if(settings.containsKey("nightMode")) {
                int nightMode = settings["nightMode"];
                Serial.printf("Gece modu ayarı: %d\n", nightMode);
                s->set_aec2(s, nightMode);
              }
              
              // Beyaz Dengesi
              if(settings.containsKey("whiteBalance")) {
                int wb = settings["whiteBalance"];
                Serial.printf("Beyaz dengesi ayarı: %d\n", wb);
                s->set_awb_gain(s, wb == 0 ? 1 : 0); // Otomatik için 1, diğerleri için 0
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
  
  // Flash LED setup
  pinMode(FLASH_GPIO_NUM, OUTPUT);
  digitalWrite(FLASH_GPIO_NUM, LOW);
  
  // Servo başlat
  servoPan.attach(SERVO_PAN_PIN);
  servoTilt.attach(SERVO_TILT_PIN);
  servoPan.write(panAngle);
  servoTilt.write(tiltAngle);

  // WiFi optimizasyonları
  WiFi.softAP(AP_SSID, AP_PASSWORD, WIFI_CHANNEL, 0, MAX_CLIENTS);
  WiFi.setSleep(false);
  
  // WiFi protokol ve güç ayarları
  esp_wifi_set_protocol(WIFI_IF_AP, WIFI_PROTOCOL);
  esp_wifi_set_bandwidth(WIFI_IF_AP, WIFI_BW_HT20);
  WiFi.setTxPower(WIFI_POWER_19_5dBm);
  
  Serial.printf("AP: %s Kanal: %d IP: %s\n", 
               AP_SSID, WIFI_CHANNEL, 
               WiFi.softAPIP().toString().c_str());

  // Kamera
  cameraActive = startCamera();

  // WebSocket
  ws.onEvent(onWsEvent);
  server.addHandler(&ws);

  // HTML sayfa
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *req){
    req->send(200, "text/html", HTML_PAGE);
  });

  server.begin();

  // Kamera görevini başlat
  xTaskCreatePinnedToCore(
    cameraTask,
    "CameraStream",
    8192,  // Stack boyutu artırıldı
    NULL,
    1,
    &cameraTaskHandle,
    1
  );
}

void loop() {
  // WebSocket temizliği
  ws.cleanupClients();
  
  // Servo pozisyonlarını güncelle
  servoPan.write(panAngle);
  delay(1);
  servoTilt.write(tiltAngle);
  delay(1);
}
