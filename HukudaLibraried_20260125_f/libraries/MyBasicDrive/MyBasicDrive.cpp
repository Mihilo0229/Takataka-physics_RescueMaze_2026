#include "MyBasicDrive.h"

MyBasicDrive::MyBasicDrive() {}

void MyBasicDrive::begin(SMS_STS* servo,
                         const uint8_t* motorID,
                         const uint16_t* speed,
                         const uint8_t* acc)
{
    _servo = servo;
    for (int i = 0; i < 4; i++) {
        _id[i]    = motorID[i];
        _speed[i] = speed[i];
        _acc[i]   = acc[i];
        _pos[i]   = 0;
    }
}

void MyBasicDrive::send() {
    _servo->SyncWritePosEx(_id, 4, _pos, _speed, _acc);
}

void MyBasicDrive::forward(int16_t power) {
    _pos[0] =  power;
    _pos[1] = -power;
    _pos[2] = -power;
    _pos[3] =  power;
    send();
}

void MyBasicDrive::turnRight(int16_t power) {
    _pos[0] = power;
    _pos[1] = power;
    _pos[2] = power;
    _pos[3] = power;
    send();
}

void MyBasicDrive::turnLeft(int16_t power) {
    _pos[0] = -power;
    _pos[1] = -power;
    _pos[2] = -power;
    _pos[3] = -power;
    send();
}

void MyBasicDrive::stop() {
    _pos[0] = 0;
    _pos[1] = 0;
    _pos[2] = 0;
    _pos[3] = 0;
    send();
}
