#define DEBUG 1

#include <Wire.h>
#include <VL53L0X.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#include "MyUARTs_send.h"   // ★ 追加

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1

#define REQ_HEADER 0xA5
#define RES_HEADER 0x5A
#define CMD_TOF    0x01//@@@tyousei@@@
#define CMD_TEST   0x90//@@@tyousei@@@

#define TOF_MAX    250 //@@@tyousei@@@
#define MEDIAN_N   5   // 移動中央値の窓サイズ//@@@tyousei@@@

VL53L0X sensor1, sensor2, sensor3, sensor4, sensor5, sensor6;
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// ===== UART ライブラリ =====
MyUARTs_send uart;

// ===== 移動中央値用バッファ =====
uint16_t tof_buf[6][MEDIAN_N];
uint8_t  tof_idx = 0;
bool     buf_filled = false;

void setup() {
  Serial.begin(9600);
  Serial1.begin(9600);//@@@tyousei@@@
  Wire.begin();
  delay(1000);

  Serial.println("Start");

  // ===== UART 初期化 =====
  uart.begin(&Serial1, REQ_HEADER, RES_HEADER);

  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println("OLED init failed");
    while (true);
  }

  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);

  pinMode(27, OUTPUT);
  pinMode(28, OUTPUT);
  pinMode(29, OUTPUT);
  pinMode(3, OUTPUT);
  pinMode(4, OUTPUT);
  pinMode(2, OUTPUT);

  digitalWrite(27, LOW);
  digitalWrite(28, LOW);
  digitalWrite(29, LOW);
  digitalWrite(3, LOW);
  digitalWrite(4, LOW);
  digitalWrite(2, LOW);
  delay(10);

  initializeSensor(sensor1, 27, 0x30);
  initializeSensor(sensor2, 28, 0x31);
  initializeSensor(sensor3, 29, 0x32);
  initializeSensor(sensor4, 3,  0x33);
  initializeSensor(sensor5, 4,  0x34);
  initializeSensor(sensor6, 2,  0x35);

  // ===== 初期値を MAX で埋める =====
  for (int s = 0; s < 6; s++) {
    for (int i = 0; i < MEDIAN_N; i++) {
      tof_buf[s][i] = TOF_MAX;
    }
  }
}

void loop() {

  // ===== 常時ToF測定（1回分）=====
  tof_buf[0][tof_idx] = min(sensor1.readRangeSingleMillimeters(), TOF_MAX);
  tof_buf[1][tof_idx] = min(sensor2.readRangeSingleMillimeters(), TOF_MAX);
  tof_buf[2][tof_idx] = min(sensor3.readRangeSingleMillimeters(), TOF_MAX);
  tof_buf[3][tof_idx] = min(sensor4.readRangeSingleMillimeters(), TOF_MAX);
  tof_buf[4][tof_idx] = min(sensor5.readRangeSingleMillimeters(), TOF_MAX);
  tof_buf[5][tof_idx] = min(sensor6.readRangeSingleMillimeters(), TOF_MAX);

  tof_idx++;
  if (tof_idx >= MEDIAN_N) {
    tof_idx = 0;
    buf_filled = true;
  }

  // ===== コマンド受信 =====
  if (uart.available()) {
    uint8_t cmd;
    if (uart.readCommand(cmd)) {

      if (cmd == CMD_TOF) {

        uint8_t median_data[6];
        for (int i = 0; i < 6; i++) {
          median_data[i] = calcMedian(tof_buf[i], MEDIAN_N);
        }

        // ===== 応答送信 =====
        uart.sendResponse(0x01, median_data);

        // ===== デバッグ =====
        Serial.print("TOF median sent: ");
        for (int i = 0; i < 6; i++) {
          Serial.print(median_data[i]);
          if (i < 5) Serial.print(",");
        }
        Serial.println();
      }
      #if DEBUG
      else if (cmd == CMD_TEST) {

        byte sendData[6] = {1, 2, 3, 4, 5, 6};

        uart.sendResponse(cmd, sendData);

        Serial.println("[MyUARTs] CMD_TEST responded");
      }
      #endif
    }
  }

  delay(20); // ≒ VL53L0X 1回分//@@@tyousei@@@
}

// ===== 中央値計算 =====
uint8_t calcMedian(uint16_t *buf, int n) {
  uint16_t tmp[MEDIAN_N];
  for (int i = 0; i < n; i++) tmp[i] = buf[i];

  for (int i = 0; i < n - 1; i++) {
    for (int j = i + 1; j < n; j++) {
      if (tmp[j] < tmp[i]) {
        uint16_t t = tmp[i];
        tmp[i] = tmp[j];
        tmp[j] = t;
      }
    }
  }
  return tmp[n / 2];
}

void initializeSensor(VL53L0X &sensor, int pin, int address) {
  digitalWrite(pin, HIGH);
  delay(50);
  sensor.init();
  sensor.setTimeout(500);
  sensor.setMeasurementTimingBudget(20000);//@@@tyousei@@@
  sensor.setAddress(address);
}
