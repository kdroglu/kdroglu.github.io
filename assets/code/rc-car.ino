// ESP32 - Kumanda (Controller) - NRF24 gönderici
#include <Arduino.h>
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <stdint.h>

// NRF24 ayarları (CE, CSN pinlerini kendi bağlantına göre değiştir)
RF24 radio(5, 17); // CE, CSN
const byte address[6] = "00001";

// Pot pinleri
const int potSteer    = 34;  // Direksiyon
const int potThrottle = 35;  // Gaz (ham ADC değeri okunacak)
const int potBrake    = 32;  // Fren (ham ADC değeri okunacak)
const int switchGear  = 25;  // Vites switch (INPUT_PULLUP kullan)

// Direksiyon parametreleri
float steer_sensitivity = 0.333; // default (45° / 135°)
const int steer_center = 2047;   // ADC ortası (12-bit: 0..4095 -> orta ~2047)

// Gaz / Fren kalibrasyon değerleri (SEN ölçüp buraya ADC değerlerini koyacaksın)
int throttleMin = 1835;   // örn: pot full bıraktığında gördüğün ham ADC değeri
int throttleMax = 2570;  // örn: pot tam bastığında gördüğün ham ADC değeri
int brakeMin    = 1650;
int brakeMax    = 2410;

// Motor gücü
float motorPower = 0.0;   // 0 .. +maxMotor (pozitif hız büyüklüğü)
const float maxMotor = 100.0;

// Fiziksel model parametreleri (ayarlanabilir)
float air_drag_factor = 0.01; // gaz kapalıyken düşüş oranı (yaklaşık 1/15s)
float accel_step      = 1.2;   // ivmelenme birimi (ne kadar hızlı hedefe ulaşsın)
float brake_factor    = 0.7;   // fren etkisi katsayısı

// Paket: int16_t + float  (packed to avoid padding)
struct __attribute__((packed)) DataPacket {
  int16_t steer;   // 0..180
  float   motor;   // -100 .. +100 (işaret burada konulacak)
} data;

// float haritalama fonksiyonu (map ile karışmasın diye isim farklı)
float mapFloat(float x, float in_min, float in_max, float out_min, float out_max) {
  if (in_max == in_min) return out_min; // bölme hatasını engelle
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println("ESP32 Kumanda (raw ADC okumali) basladi");
  Serial.print("DataPacket sizeof = "); Serial.println(sizeof(data)); // beklenen 6

  // NRF24 başlat
  radio.begin();
  radio.openWritingPipe(address);
  radio.setPALevel(RF24_PA_HIGH);
  radio.stopListening();

  pinMode(switchGear, INPUT_PULLUP);
}

void loop() {
  // --- DIREKSIYON ---
  int rawSteer = analogRead(potSteer); // 0..~4095
  int delta = rawSteer - steer_center;
  float potDeg = (delta * (270.0f / 4095.0f)); // +/- up to ~135
  float angle  = potDeg * steer_sensitivity;
  if (angle > 45.0f) angle = 45.0f;
  if (angle < -45.0f) angle = -45.0f;
  int servoAngle = (int)round((angle + 45.0f) * 2.0f); // -45..+45 -> 0..180
  data.steer = (int16_t)constrain(servoAngle, 0, 180);

  // --- GAZ ve FREN (ham ADC değerlerini oku) ---
  int rawThrottle = analogRead(potThrottle); // HAM ADC
  int rawBrake    = analogRead(potBrake);    // HAM ADC

  // İç hesaplama: raw -> hedef throttle (0..maxMotor)
  float targetThrottle = mapFloat((float)rawThrottle, (float)throttleMin, (float)throttleMax, 0.0f, maxMotor);
  targetThrottle = constrain(targetThrottle, 0.0f, maxMotor);

  // fren için normalize edilmiş değer 0..1 (hesaplarda kullanmak için)
  float brakeNorm = mapFloat((float)rawBrake, (float)brakeMin, (float)brakeMax, 0.0f, 1.0f);
  brakeNorm = constrain(brakeNorm, 0.0f, 1.0f);

  // --- motorPower hesaplama ---
  if (targetThrottle > motorPower) {
    motorPower += accel_step;
    if (motorPower > targetThrottle) motorPower = targetThrottle;
  } else {
    // gaz azaldığında hava sürtünmesi etkisi
    motorPower -= air_drag_factor * motorPower;
    if (motorPower < 0.0f) motorPower = 0.0f;
  }

  // fren uygulama (brakeNorm 0..1)
  if (brakeNorm > 0.01f) {
    float brakeStrength = brakeNorm * brake_factor; // 0..brake_factor
    motorPower -= brakeStrength * maxMotor * 0.05f;
    if (motorPower < 0.0f) motorPower = 0.0f;
  }

  // vites: pull-up kullanıyoruz; LOW -> geri modu
  bool gearIsLow = (digitalRead(switchGear) == LOW);
  data.motor = gearIsLow ? -motorPower : motorPower;

  // --- GÖNDER ---
  bool ok = radio.write(&data, sizeof(data));

  // --- Seri çıktı: HAM ADC değerleri gösteriliyor (yüzdelik yok) ---
  Serial.print("rawSteer:"); Serial.print(rawSteer);
  Serial.print("\trawThrottle:"); Serial.print(rawThrottle);
  Serial.print("\trawBrake:"); Serial.print(rawBrake);
  Serial.print("\tmotorPower:"); Serial.print(motorPower);
  Serial.print("\tsentMotor:"); Serial.print(data.motor);
  Serial.print("\tTX:"); Serial.println(ok ? "OK" : "ERR");

  delay(20); // ~50 Hz
}
