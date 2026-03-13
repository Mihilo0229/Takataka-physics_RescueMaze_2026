/*******************************************************************************************/
/* Servo_rescue                                                                              
/*処理：サーボモーターによるレスキューキットの排出
/*
/*更新者：清田侑希 2025/3/26
/*
/*******************************************************************************************/
void motor_servo(int motor_speed, int seconds) {
  int pulse_width = 1500-motor_speed*8;//@@@tyousei@@@
  int starttime = millis();
  while(true) {
    if (millis()-starttime > seconds) break;
      digitalWrite(SERVO_PIN, HIGH);
      delayMicroseconds(pulse_width);
      digitalWrite(SERVO_PIN, LOW);
      delay(20);    
  } 
}

void motor_servo2(int motor_speed, int seconds) {
  int pulse_width = 1400-motor_speed*8;//@@@
  int starttime = millis();
  while(true) {
    if (millis()-starttime > seconds) break;
      digitalWrite(SERVO_PIN2, HIGH);
      delayMicroseconds(pulse_width);
      digitalWrite(SERVO_PIN2, LOW);
      delay(20);    
  } 
}

void servo_res(){
  motor_servo(130,220);//@@@tyousei@@@
  motor_servo(0,1000);//@@@
  motor_servo(-135,210);//@@@
  motor_servo(0,1000);//@@@
  delay(150);
  servo1_kit--;
}

void servo_res2(){//@@@
  motor_servo2(-120,200);
  motor_servo2(0,1000);
  motor_servo2(130,200);
  motor_servo2(0,1000);
  delay(150);
  servo2_kit--;
}