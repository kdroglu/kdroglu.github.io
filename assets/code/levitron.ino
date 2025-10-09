// --- Pin Tanımları (ESP32-C3 Mini) ---
#include <Arduino.h>

const int hallPin       = 3;    // ADC1_CH3 (GPIO3)
const int pwmPushPin    = 20;   // H-Bridge IN1 (GPIO20)
const int pwmPullPin    = 21;   // H-Bridge IN2 (GPIO21)
const int potPin        = 4;    // Setpoint ayarı için pot
const int ledBalancePin = 6;    // Denge LED'i (IO6)

// --- Ölçüm Limitleri ---
const int lowerLimit    = 710;
const int upperLimit    = 875;

// --- PID Parametreleri ---
float Kp = 2.5;
float Ki = 0.002;
float Kd = 1.25;
float setpoint = 780;  // Başlangıç setpoint değeri

// --- Anti-windup sınırları ---
const float Imin = -20;
const float Imax =  20;

// --- Dirty-derivative filtresi sabiti (saniye) ---
const float tau = 0.1;

// --- EMA filtresi ve histerezis / debounce ayarları ---
const float alphaMeas   = 0.1;   // Ölçüm için EMA katsayısı
const float alphaPot    = 0.005;   // Pot için EMA katsayısı
const int   hysteresis  = 2;     // ±2 birim
const int   debounceMax = 4;     // ardışık 5 ihlalde “limit aktif”

// --- Denge LED eşiği ---
const float errorThreshold = 4.0; // |error| altındaki denge noktası

// --- Durum değişkenleri ---
float integral    = 0;
float derivFilt   = 0;
float prevError   = 0;
float measFilt;
float potFilt;
int   outCount    = 0;           // debounce sayacı
bool  limitState  = false;       // limit kapalı mı?
unsigned long lastTime;

void setup() {
  Serial.begin(115200);
  analogReadResolution(10);
  // Her PWM kanalı için ayrı 10-bit çözünürlük
  analogWriteResolution(pwmPushPin, 10);
  analogWriteResolution(pwmPullPin, 10);
  analogWriteFrequency(pwmPushPin, 30000);
  analogWriteFrequency(pwmPullPin, 30000);

  pinMode(pwmPushPin, OUTPUT);
  pinMode(pwmPullPin, OUTPUT);
  pinMode(potPin, INPUT);
  pinMode(ledBalancePin, OUTPUT);

  // Başlangıç filtre değerleri
  measFilt = analogRead(hallPin);
  potFilt  = analogRead(potPin);

  lastTime = millis();
}

void loop() {
  // --- Zaman farkı ---
  unsigned long now = millis();
  float dt = (now - lastTime) / 1000.0;
  lastTime = now;
  if (dt <= 0) dt = 0.001;

  // --- Ham ölçüm ve EMA filtresi ---
  int rawMeas = analogRead(hallPin);
  measFilt = alphaMeas * rawMeas + (1.0 - alphaMeas) * measFilt;

  // --- Pot okuma + EMA filtresi ---
  int rawPot = analogRead(potPin);
  potFilt = alphaPot * rawPot + (1.0 - alphaPot) * potFilt;
  // Filtrelenmiş pot değerinden setpoint hesapla
  setpoint = lowerLimit + (upperLimit - lowerLimit-20) * (potFilt / 1023.0);

  // --- Histerezisli sınırlar ---
  int lowTrig   = lowerLimit  - hysteresis;
  int lowReset  = lowerLimit  + hysteresis;
  int highTrig  = upperLimit  + hysteresis;
  int highReset = upperLimit  - hysteresis;

  // --- Debounce sayacı ---
  if (measFilt < lowTrig || measFilt > highTrig) {
    outCount++;
  } else {
    outCount = 0;
  }

  // --- Limit durum güncelle ---
  if (!limitState && outCount >= debounceMax) {
    limitState = true;
    integral  = 0;
    prevError = 0;
    derivFilt = 0;
  } else if (limitState && measFilt > lowReset && measFilt < highReset) {
    limitState = false;
  }

  // --- Limit aktifse PWM kes ve LED kapat ---
  if (limitState) {
    analogWrite(pwmPushPin, 0);
    analogWrite(pwmPullPin, 0);
    digitalWrite(ledBalancePin, LOW);
    delay(1);
    return;
  }

  // --- PID Hesaplama ---
  float error = setpoint - measFilt;

  // --- Seri Çizici Verileri ---
  Serial.print(rawMeas);
  Serial.print(",");
  Serial.print(10 * error);
  //Serial.print(",");
  //Serial.println(setpoint);

  // --- LED: Denge kontrolü ---
  if (abs(error) < errorThreshold) {
    digitalWrite(ledBalancePin, LOW);
  } else {
    digitalWrite(ledBalancePin, HIGH);
  }

  // --- Trapezoidal integral + anti-windup ---
  integral += 0.5 * (error + prevError) * dt;
  integral = constrain(integral, Imin, Imax);

  // --- Dirty-derivative filtresi ---
  float rawDeriv = (error - prevError) / dt;
  derivFilt = (tau / (tau + dt)) * derivFilt
              + (Kd * dt / (tau + dt)) * rawDeriv;

  // --- PID çıkış ---
  float output = Kp * error + Ki * integral + derivFilt;
  prevError = error;

  // --- PWM’e dönüştür ve uygula ---
  int pwmVal = constrain(int(abs(output)), 0, 1023);
  if (output > 0) {
    analogWrite(pwmPullPin, pwmVal);
    analogWrite(pwmPushPin, 0);
  } else {
    analogWrite(pwmPushPin, pwmVal);
    analogWrite(pwmPullPin, 0);
  }

  // --- Döngü gecikmesi (orijinal) ---
  delayMicroseconds(130);
}
