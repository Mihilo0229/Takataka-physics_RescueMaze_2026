#ifndef MYUARTS_READ_H
#define MYUARTS_READ_H

#include <Arduino.h>

class MyUARTs_read {
public:
    MyUARTs_read();

    // シリアル初期化
    void begin(HardwareSerial* port);

    // ===== コマンド要求 → 応答受信（既存） =====
    bool request(byte cmd, byte* buffer, unsigned long timeout_ms);

    // ===== イベント受信（追加） =====
    bool hasEvent();        // イベントが来ているか？
    byte readEvent();       // 1byteイベントを読む

    // ===== イベントフレーム受信（3byte） =====
    bool readEventFrame(uint8_t& code);


    private:
    HardwareSerial* _port;

    static const byte REQ_HEADER = 0xA5;
    static const byte RES_HEADER = 0x5A;
    static const int  DATA_LEN   = 6;

    // 受信バッファ掃除
    void flushInput();

    // チェックサム計算
    byte calcChecksum(byte cmd, const byte* data);
};

#endif
