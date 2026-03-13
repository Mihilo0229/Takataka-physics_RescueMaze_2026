#include "MyUARTs_send.h"

MyUARTs_send::MyUARTs_send() {
    _port = nullptr;
}

void MyUARTs_send::begin(HardwareSerial* port,
                         uint8_t reqHeader,
                         uint8_t resHeader)
{
    _port = port;
    _reqHeader = reqHeader;
    _resHeader = resHeader;
}

// ===== コマンドが来ているか =====
bool MyUARTs_send::available() {
    return (_port && _port->available() >= 2);
}

// ===== コマンド受信 =====
bool MyUARTs_send::readCommand(uint8_t& cmd) {
    if (!_port) return false;
    if (_port->available() < 2) return false;

    uint8_t h = _port->read();
    if (h != _reqHeader) {
        return false; // 再同期は read 側に任せる
    }

    cmd = _port->read();
    return true;
}

// ===== 応答送信（6byte固定） =====
void MyUARTs_send::sendResponse(uint8_t cmd,
                                const uint8_t data[6])
{
    if (!_port) return;

    uint8_t cs = cmd;

    _port->write(_resHeader);
    _port->write(cmd);

    for (uint8_t i = 0; i < 6; i++) {
        _port->write(data[i]);
        cs ^= data[i];
    }

    _port->write(cs);
}
