#include <stdio.h>
#include <Wire.h>
#include <Adafruit_TCS34725.h>
#include <HX711.h>

// ===== イベント定義 =====
#define EVT_BLACK   1
#define EVT_BLUE    2
#define EVT_LEFT    3
#define EVT_RIGHT   4
#define EVT_FRONT   5
#define EVT_CHECK   6
#define EVT_READY   7

unsigned long lastBumperSend = 0;
unsigned long lastColorSend  = 0;

const unsigned long BUMPER_COOLDOWN = 1500;//@@@tyousei@@@
const unsigned long COLOR_COOLDOWN  = 2000;//@@@tyousei@@@

// ===== READY管理フラグ =====
bool load_ready  = false;
bool color_ready = false;
bool ready_sent  = false;

// ===== センサ設定 =====
Adafruit_TCS34725 tcs =
  Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);

int blue_send_limit = 5;//@@@tyousei@@@

const int DT_PIN  = 27;
const int SCK_PIN = 26;
const int DT_PIN2  = 4;
const int SCK_PIN2 = 3;

HX711 scale, scale2;

// ===== UARTイベント送信 =====
void sendEvent(uint8_t code) {
  Serial1.write(0xAA);
  Serial1.write(code);
  Serial1.write(code ^ 0xFF);
}

/* =========================
   ロードセル初期化
   ========================= */
void load1_setup() {
  scale.begin(DT_PIN, SCK_PIN);
  scale.set_scale();
  scale.tare();
  delay(2000);
  scale.set_scale(-375.00);
  scale.tare();
}

void load2_setup() {
  scale2.begin(DT_PIN2, SCK_PIN2);
  scale2.set_scale();
  scale2.tare();
  delay(2000);
  scale2.set_scale(-375.00);
  scale2.tare();
}

/* =========================
   CPU0 setup（ロードセル）
   ========================= */
void setup() {
  Serial.begin(115200);
  Serial1.begin(19200);
  delay(2000);

  load1_setup();
  load2_setup();

  load_ready = true;   // ★ ロードセル準備完了
}

/* =========================
   CPU0 loop（バンパー）
   ========================= */
void loop() {

  // ===== READY送信（1回だけ）=====
  if (load_ready && color_ready && !ready_sent) {
    sendEvent(EVT_READY);
    Serial.println("SEND: READY");
    ready_sent = true;
  }

  float load   = scale.get_units(1);
  float load_2 = scale2.get_units(1);
  unsigned long now = millis();

  // ===== FRONT =====
  if (load <= -500 && load_2 >= 500) {//@@@tyousei@@@
    if (now - lastBumperSend > BUMPER_COOLDOWN) {
      sendEvent(EVT_FRONT);
      lastBumperSend = now;
    }
  }
  // ===== LEFT =====
  else if (load <= -750 || load_2 <= -750) {//@@@tyousei@@@
    if (now - lastBumperSend > BUMPER_COOLDOWN) {
      sendEvent(EVT_LEFT);
      lastBumperSend = now;
    }
  }
  // ===== RIGHT =====
  else if (load >= 750 || load_2 >= 750) {//@@@tyousei@@@
    if (now - lastBumperSend > BUMPER_COOLDOWN) {
      sendEvent(EVT_RIGHT);
      lastBumperSend = now;
    }
  }

  Serial.println(load);Serial.println(load_2);

  Serial.println("===================");

  delay(5);
}

/* =========================
   CPU1 setup（カラー）
   ========================= */
void setup1() {
  Serial.begin(115200);
  Serial1.begin(19200);
  Wire.begin();

  while (!tcs.begin()) {
    delay(2000);
  }

  tcs.setIntegrationTime(TCS34725_INTEGRATIONTIME_154MS);
  tcs.setGain(TCS34725_GAIN_4X);

  color_ready = true;   // ★ カラーセンサ準備完了
}

/* =========================
   CPU1 loop（カラー）
   ========================= */
void loop1() {

  uint16_t clear, red, green, blue;
  tcs.getRawData(&red, &green, &blue, &clear);

  if (clear == 0) return;

  unsigned long now = millis();
  int b = blue * 256 / clear;

  // ===== BLUE TILE =====
  if (b >= 90 && blue_send_limit > 0) {//@@@tyousei@@@
    if (now - lastColorSend > COLOR_COOLDOWN) {
      sendEvent(EVT_BLUE);
      blue_send_limit--;
      lastColorSend = now;
    }
  }
  // ===== BLACK TILE =====
  else if (red <= 300 && green <= 300 && blue <= 300 &&
           red != 0 && green != 0 && blue != 0) {//@@@tyousei@@@
    if (now - lastColorSend > COLOR_COOLDOWN) {
      sendEvent(EVT_BLACK);
      lastColorSend = now;
    }
  }

  Serial.print("red =");Serial.print(red);
  Serial.print("green =");Serial.print(green);
  Serial.print("blue =");Serial.print(blue);Serial.print(",");Serial.println(b);

  delay(10);
}
