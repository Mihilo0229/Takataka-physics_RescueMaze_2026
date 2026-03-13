#include <MyYPR.h>

MyYPR imu;

void setup(){
  Serial.begin(115200);
  imu.begin(0x68,&Wire);
}

void loop(){
  imu.update();

  YPR a = imu.getYPR();
  Serial.print("Y:"); Serial.print(a.yaw);
  Serial.print(" P:"); Serial.print(a.pitch);
  Serial.print(" R:"); Serial.println(a.roll);

  delay(10);
}
