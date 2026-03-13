#include "MyYPRs.h"

MyYPRs::MyYPRs() {
  yawDeg   = 0.0;
  pitchDeg = 0.0;
  rollDeg  = 0.0;

  yawOffsetDeg = 0.0;
  lastUpdateMicros = 0;
}

bool MyYPRs::begin(TwoWire* wire) {

  if (!mpu.begin(0x68, wire)) {
    Serial.println("MPU6050 not found");
    return false;
  }

  // 大会向け安定設定
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  delay(100);

  lastUpdateMicros = micros();
  resetYaw();

  return true;
}

bool MyYPRs::update() {

  unsigned long now = micros();
  float dt = (now - lastUpdateMicros) / 1e6;
  if (dt <= 0) {
    lastUpdateMicros = now;
    return false;
  }
  lastUpdateMicros = now;

  mpu.getEvent(&accel, &gyro, &temp);

  /* ===== yaw：ジャイロZ積分 ===== */
  yawDeg += gyro.gyro.z * RAD_TO_DEG * dt;

  /* ===== pitch / roll（坂検知用） ===== */
  pitchDeg = atan2(
    -accel.acceleration.x,
    sqrt(
      accel.acceleration.y * accel.acceleration.y +
      accel.acceleration.z * accel.acceleration.z
    )
  ) * RAD_TO_DEG;

  rollDeg = atan2(
    accel.acceleration.y,
    accel.acceleration.z
  ) * RAD_TO_DEG;

  return true;
}

// =======================
// 角度取得
// =======================

float MyYPRs::getYawRaw() {
  return yawDeg;
}

// ★ 回転制御専用（相対角）
float MyYPRs::getYawRelative() {
  return yawDeg - yawOffsetDeg;
}

// ★ 先輩互換・絶対角（方向管理用）
float MyYPRs::getYaw360() {

  float yaw = yawDeg - yawOffsetDeg;

  if (yaw < 0) yaw += 360;
  yaw = 360 - yaw;               // 先輩コード互換

  return normalize360(yaw);
}

float MyYPRs::getPitch() {
  return pitchDeg;
}

float MyYPRs::getRoll() {
  return rollDeg;
}

YPR MyYPRs::getYPR() {
  YPR d;
  d.yaw   = getYaw360();
  d.pitch = pitchDeg;
  d.roll  = rollDeg;
  return d;
}

// =======================
// reset
// =======================

void MyYPRs::resetYaw() {
  yawOffsetDeg = yawDeg;
}

// =======================
// util
// =======================

float MyYPRs::normalize360(float deg) {
  while (deg >= 360.0) deg -= 360.0;
  while (deg < 0.0)    deg += 360.0;
  return deg;
}
