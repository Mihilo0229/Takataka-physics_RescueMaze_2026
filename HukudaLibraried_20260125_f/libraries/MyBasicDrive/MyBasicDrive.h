#ifndef MY_BASIC_DRIVE_H
#define MY_BASIC_DRIVE_H

#include <Arduino.h>
#include <SCServo.h>
#include <stdint.h>

class MyBasicDrive {
public:
    MyBasicDrive();

    void begin(SMS_STS* servo,
               const uint8_t* motorID,
               const uint16_t* speed,
               const uint8_t* acc);

    // ---- 即時命令（元設計そのまま）----
    void forward(int16_t power);     // 前進
    void turnRight(int16_t power);   // 右回転
    void turnLeft(int16_t power);    // 左回転
    void stop();                     // 停止（安全用）

private:
    SMS_STS* _servo;

    uint8_t  _id[4];
    uint16_t _speed[4];
    uint8_t  _acc[4];

    int16_t  _pos[4];   // ★ int → int16_t に修正

    void send();        // 内部送信
};

#endif
