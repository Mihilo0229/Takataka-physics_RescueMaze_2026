#include "MyUARTs_read.h"

MyUARTs_read::MyUARTs_read() {
    _port = nullptr;
}

void MyUARTs_read::begin(HardwareSerial* port) {
    _port = port;
}

void MyUARTs_read::flushInput() {
    while (_port->available() > 0) {
        _port->read();
    }
}

byte MyUARTs_read::calcChecksum(byte cmd, const byte* data) {
    byte cs = cmd;
    for (int i = 0; i < DATA_LEN; i++) {
        cs ^= data[i];
    }
    return cs;
}

/* =========================================================
   コマンド要求 → 応答受信（既存・変更なし）
   ========================================================= */
bool MyUARTs_read::request(byte cmd, byte* buffer, unsigned long timeout_ms) {
    if (_port == nullptr) return false;

    // ===== ① 古いデータ破棄 =====
    flushInput();

    // ===== ② コマンド送信 =====
    _port->write(REQ_HEADER);
    _port->write(cmd);

    unsigned long start = millis();
    int state = 0;      // 0: header待ち, 1: cmd待ち, 2: data, 3: cs
    int index = 0;
    byte recv_cmd = 0;
    byte recv_cs  = 0;

    // ===== ③ 応答待ち =====
    while (millis() - start < timeout_ms) {
        if (_port->available() <= 0) continue;

        byte b = _port->read();

        switch (state) {
        case 0: // RES_HEADER待ち
            if (b == RES_HEADER) {
                state = 1;
            }
            break;

        case 1: // cmd確認
            recv_cmd = b;
            if (recv_cmd != cmd) {
                state = 0; // 不一致 → 再同期
            } else {
                index = 0;
                state = 2;
            }
            break;

        case 2: // data受信
            buffer[index++] = b;
            if (index >= DATA_LEN) {
                state = 3;
            }
            break;

        case 3: // checksum
            recv_cs = b;
            if (recv_cs == calcChecksum(cmd, buffer)) {
                return true; // 正常受信
            } else {
                return false; // チェックサム不一致
            }
        }
    }

    // ===== ④ タイムアウト =====
    return false;
}

/* =========================================================
   イベント受信（追加）
   ========================================================= */

// イベントが来ているか？
bool MyUARTs_read::hasEvent() {
    if (_port == nullptr) return false;
    return (_port->available() > 0);
}

// 1byteイベントを読む
byte MyUARTs_read::readEvent() {
    if (_port == nullptr) return 0;
    if (_port->available() == 0) return 0;

    return _port->read();
}

/* =========================================================
   イベントフレーム受信（0xAA + code + checksum）
   ========================================================= */
bool MyUARTs_read::readEventFrame(uint8_t& code) {
    if (_port == nullptr) return false;

    // 3バイト以上なければ何もしない
    if (_port->available() < 3) return false;

    // ヘッダ探索
    uint8_t h = _port->read();
    if (h != 0xAA) {
        return false; // 再同期（次回に任せる）
    }

    // 残り2バイト確認
    if (_port->available() < 2) {
        return false;
    }

    uint8_t recv_code = _port->read();
    uint8_t recv_cs   = _port->read();

    // チェックサム確認
    if ((recv_code ^ 0xFF) != recv_cs) {
        return false;
    }

    code = recv_code;
    return true;
}
