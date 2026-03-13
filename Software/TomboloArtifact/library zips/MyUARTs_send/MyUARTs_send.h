#ifndef MYUARTS_SEND_H
#define MYUARTS_SEND_H

#include <Arduino.h>

class MyUARTs_send {
public:
    MyUARTs_send();

    void begin(HardwareSerial* port,
               uint8_t reqHeader,
               uint8_t resHeader);

    // ===== コマンド受信 =====
    bool available();
    bool readCommand(uint8_t& cmd);

    // ===== 応答送信（6byte固定） =====
    void sendResponse(uint8_t cmd,
                      const uint8_t data[6]);

private:
    HardwareSerial* _port;
    uint8_t _reqHeader;
    uint8_t _resHeader;
};

#endif
