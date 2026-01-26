#ifndef MY_YPRS_H
#define MY_YPRS_H

#include <Arduino.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

struct YPR {
  float yaw;
  float pitch;
  float roll;
};

class MyYPRs {
public:
  MyYPRs();

  // 初期化
  bool begin(TwoWire* wire = &Wire);

  // センサ更新
  bool update();

  // ===== 角度取得 =====
  float getYaw360();       // 0〜360°（先輩コード互換・絶対角）
  float getYawRaw();       // 積分Yaw（内部用）
  float getYawRelative();  // ★ 相対Yaw（回転制御用）
  float getPitch();
  float getRoll();
  YPR   getYPR();

  // ===== 基準リセット =====
  void resetYaw();         // 相対Yawの基準を現在値に

private:
  Adafruit_MPU6050 mpu;

  // センサ値
  sensors_event_t accel;
  sensors_event_t gyro;
  sensors_event_t temp;

  // 内部角度
  float yawDeg;
  float pitchDeg;
  float rollDeg;

  float yawOffsetDeg;
  unsigned long lastUpdateMicros;

  float normalize360(float deg);
};

#endif
