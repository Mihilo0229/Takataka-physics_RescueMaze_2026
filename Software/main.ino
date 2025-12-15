/*

関数の編集時には記録を残してください

このコードの見方
関数の最初にその関数の簡易的な説明が書いてあります
また編集者の名前と日付も書いてあります
検索でキーワードや編集者の名前を入れると見たい関数が探しやすいです
*/



#include <stdio.h>
#include <cstdint>
#include <Arduino.h>
#include <time.h>
#include <cstdint>
#include <SCServo.h> //SCServoを使うと宣言]
#include <SoftwareSerial.h> //SoftwareSerial(複数機器とシリアル通信)を使うと宣言
#include "MPU6050_6Axis_MotionApps20.h"
#include <Adafruit_NeoPixel.h>
#include <queue>
#include <stack>


std::stack<uint8_t> S;
std::queue<uint8_t> Q;
MPU6050 mpu;
SMS_STS sms_sts;
SoftwareSerial softSerial(36,37);

#define East 1
#define North 2
#define West 3
#define South 4

#define Right 1
#define Front 2
#define Left 3
#define Back 4

/*デバッグ用*/
uint8_t Homecount = 0;
bool BFScount = false;


uint8_t x = 50;
uint8_t y = 50;
uint8_t Direction = North;
uint8_t Status = 0;

uint8_t CheckX = 50;
uint8_t CheckY = 50;
uint8_t CheckD = North;

/*MPU変数*/
// MPU control/status vars
double pitch;
double roll;
double yaw;

uint8_t mpuIntStatus;
uint8_t devStatus;
uint16_t packetSize;
uint8_t fifoBuffer[64];

Quaternion q;
VectorFloat gravity;
VectorInt16 aa;
VectorInt16 aaReal;
float ypr[3];


/*******************************************************************************************/
/* Yaw・Pitch・Roll角の取得                                                                            
/*処理：MPU6050を用いて回転や坂検知のためのRoll・Yaw角の取得
/*
/*更新者：清田侑希　2025/1/25
/*
/*******************************************************************************************/
void getYawPitchRoll() {
  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);

    yaw = ypr[0] * 180 / M_PI;
    pitch = ypr[1] * 180 / M_PI;
    roll = ypr[2] * 180 / M_PI;
    //yaw角のデータを常に正にする
    if (yaw < 0) {
      yaw = 360 + yaw;
    }
    yaw = 360 - yaw;
    
    Serial.print(" X: ");
    Serial.print(gravity.x);
    Serial.print(" Y: ");
    Serial.print(gravity.y);
    Serial.print(" Z: ");
    Serial.print(gravity.z);
    Serial.print(" Yaw: ");
    Serial.print(yaw);
    Serial.print(" Pitch: ");
    Serial.print(pitch);
    Serial.print(" Roll: ");
    Serial.println(roll);
    
  }
}
/*.........角度補正用の変数の定義.........*/
float body_len = 115; //機体の前のセンサーと後ろのセンサーの距離
float rag = 0; //前と後ろのtofの値の差
float tan_value = 0; //rag/body_lenからくるtanθの値
float theta_rad = 0; //θ・弧度法
float theta_deg = 0; //θ・度数法
float ave_tof_data = 0; //右側のTofの平均値
float rag2 = 0; //中心線15㎝との差
float tan_value2 = 0; //中心補正をかけるときのtanθの値
float theta_rad2 = 0; //θ・弧度法
float theta_deg2 = 0; //θ・度数法
float run_len = 0; //走る長さ
float run_len_time = 1; //30㎝の何倍か
/*.....tof受信用の変数.......................................*/

const byte HEADER = 255;  // ヘッダー (送信側と一致すること)
bool headerDetected = false;
byte receivedData[6];  // 受信データ格納用
int receivedIndex = 0; // データ格納位置

/*.........NeoPixelの変数の定義........................*/
#define PIN        9    // データ信号用のピン
#define NUMPIXELS  8     // LEDの数
#define BRIGHTNESS 100   // LEDの明るさを設定 (0～255)

Adafruit_NeoPixel pixels(NUMPIXELS, PIN, NEO_GRB + NEO_KHZ800); //8個のNeoPixelがSeeedのデジタルピン27に接続され、GRB順の800kHzのデータ伝送速度で動作
/*..................レスキューキット排出用のサーボモーターの変数の定義............*/
const int SERVO_PIN = 10; //servo2 36
const int SERVO_PIN2 = 36;
int servo1_kit = 8; //レスキューキットの残数
int servo2_kit = 1; //上に同じ
/*..........................................................*/
/*センサー類変数*/
unsigned int SERVO_POS = 0; //変数SERVO_POS=0とする
byte ID[4]; //IDはそれぞれのモーターのID
s16 Position[4]; //Positionはモーターが回る角度(右一回転＝4095)
u16 Speed[4]; 
byte ACC[4]; 
int susumu_kaisuu= 40;
int i=1;
int n=1;
double kakudo; //tiziki()でのyawの代入
int kakudo_true; //tiziki()でyawを整数化したあとの変数
double kakudo2; //tiziki_kaitenn()でのyawの代入
int kakudo_true2; //tiziki_kaitenn()でyawを整数化したあとの変数
int mawasu; //どこまで機体を回すかの変数
bool chousei =0; //MPUが0度と360度になったときの分岐
double katamuki; //tiziki2()でのrollの代入
int katamuki_true; //tiziki2()でrollを整数化したあとの変数
int count2 = 0;//進んだ回数＿40回でわかれている
int bump_giveup_count = 0; //障害物に当たった時あきらめる変数

bool VictimisAlready[90][90];
bool right_wall = false;//ここもセンサーの値を取得するファイルとして分けたい
bool front_wall = false;
bool left_wall = false;
int8_t BlackTile =false;
int8_t blue_count =false;
bool Slope = false;//sloopがあるかどうか
bool Gap = false;



/*******************************************************************************************/
/* 地磁気回転                                                                            
/*処理：回転の際に用いるYaw角のデータを整数化
/*
/*更新者：清田侑希 2025/1/25
/*
/*******************************************************************************************/
void tiziki_kaitenn()
{
  getYawPitchRoll();


  kakudo2 = yaw;
  kakudo_true2 = int(kakudo2);
  Serial.println(kakudo_true2);
}



/*******************************************************************************************/
/* 地磁気                                                                              
/*処理：回転後に発生する機体の方向の誤差をMPU6050によって補正
/* 変更点：モーターにギアを導入したのでモーターの回転方向が逆になってます
/*更新者：清田侑希 2025/1/25
/*清田侑希 2025/3/27 変更点：基準となる角度を定数から変数に変更
/*******************************************************************************************/
void tiziki()
{
  getYawPitchRoll();
  kakudo =yaw;
  kakudo_true = int(kakudo);
  Serial.println(kakudo_true);
  if (kakudo_true >= 0 && kakudo_true < 45) {
    for (int i =0; i < kakudo_true -mawasu ;i++){
        Position[0] = 60; //右に回転
        Position[1] = 60;
        Position[2] = 60;
        Position[3] = 60;
        sms_sts.SyncWritePosEx(ID, 4, Position, Speed, ACC);
        delay(18);
      
    }
  }else if (kakudo_true >= 45 && kakudo_true < 90) {
    for (int i =0; i < mawasu-kakudo_true ;i++){
        Position[0] = -60; //左に回転
        Position[1] = -60;
        Position[2] = -60;
        Position[3] = -60;
        sms_sts.SyncWritePosEx(ID, 4, Position, Speed, ACC);
        delay(18);
      
    }
  }else if (kakudo_true >= 90 && kakudo_true < 135) {
    for (int i =0; i < kakudo_true-mawasu ;i++){
        Position[0] = 60; //右に回転
        Position[1] = 60;
        Position[2] = 60;
        Position[3] = 60;
        sms_sts.SyncWritePosEx(ID, 4, Position, Speed, ACC);
        delay(18);
      
    }
  }else if (kakudo_true >= 135 && kakudo_true < 180) {
    for (int i =0; i < mawasu-kakudo_true ;i++){
        Position[0] = -60; //左に回転
        Position[1] = -60;
        Position[2] = -60;
        Position[3] = -60;
        sms_sts.SyncWritePosEx(ID, 4, Position, Speed, ACC);
        delay(18);
      
    }
  }else if (kakudo_true >= 180 && kakudo_true < 225) {
    for (int i =0; i < kakudo_true-mawasu ;i++){
        Position[0] = 60; //右に回転
        Position[1] = 60;
        Position[2] = 60;
        Position[3] = 60;
        sms_sts.SyncWritePosEx(ID, 4, Position, Speed, ACC);
        delay(18);
      
    }
  }else if (kakudo_true >= 225 && kakudo_true < 270) {
    for (int i =0; i < mawasu-kakudo_true ;i++){
        Position[0] = -60; //左に回転
        Position[1] = -60;
        Position[2] = -60;
        Position[3] = -60;
        sms_sts.SyncWritePosEx(ID, 4, Position, Speed, ACC);
        delay(18);
      
    }
  }else if (kakudo_true >= 270 && kakudo_true < 315) {
    for (int i =0; i < kakudo_true-mawasu ;i++){
        Position[0] = 60; //右に回転
        Position[1] = 60;
        Position[2] = 60;
        Position[3] = 60;
        sms_sts.SyncWritePosEx(ID, 4, Position, Speed, ACC);
        delay(18);
      
    }
  }else if (kakudo_true >= 315 && kakudo_true <= 360) {
    for (int i =0; i < mawasu-kakudo_true ;i++){
        Position[0] = -60; //左に回転
        Position[1] = -60;
        Position[2] = -60;
        Position[3] = -60;
        sms_sts.SyncWritePosEx(ID, 4, Position, Speed, ACC);
        delay(18);
      
    }
  }
  Serial.println("kakudo_hosei");
  delay(100);
}



/*******************************************************************************************/
/* 地磁気２                                                                              
/*処理：坂の検知に用いるRoll角の整数化
/*変更点：MPU6050の設置方向の関係からPitch角からRoll角に仕様変更
/*更新者：清田侑希 2025/1/25
/*
/*******************************************************************************************/
void tiziki_2(){
  getYawPitchRoll();
  katamuki = roll;
  katamuki_true = int(katamuki);
  Serial.println(katamuki_true); 
  
}

/*******************************************************************************************/
/* NeoPixelのいろいろな動作関数                                                                            
/*処理：Neopixelを前後左右、赤黄緑に光らせる
/*更新者：清田侑希 2025/3/25
/*
/*******************************************************************************************/

void NeoPixel_Front_ON() {
  pixels.setPixelColor(0, pixels.Color(255, 0, 0)); // 0番目のLEDを(R,G,B)=(255,0,0)⇔赤に点灯
  pixels.setPixelColor(1, pixels.Color(255, 0, 0)); // 1番目のLEDを(R,G,B)=(255,0,0)⇔赤に点灯
  pixels.show(); // LEDに反映
}
void NeoPixel_Front_OFF() {
  pixels.setPixelColor(0, pixels.Color(0, 0, 0)); // 0番目のLEDを消灯
  pixels.setPixelColor(1, pixels.Color(0, 0, 0)); // 1番目のLEDを消灯
  pixels.show(); // LEDに反映
}
void NeoPixel_Right_ON() {
  pixels.setPixelColor(2, pixels.Color(255, 0, 0)); // 2番目のLEDを(R,G,B)=(255,0,0)⇔赤に点灯
  pixels.setPixelColor(3, pixels.Color(255, 0, 0)); // 3番目のLEDを(R,G,B)=(255,0,0)⇔赤に点灯
  pixels.show(); // LEDに反映
}
void NeoPixel_Right_OFF() {
  pixels.setPixelColor(2, pixels.Color(0, 0, 0)); // 2番目のLEDを消灯
  pixels.setPixelColor(3, pixels.Color(0, 0, 0)); // 3番目のLEDを消灯
  pixels.show(); // LEDに反映
}
void NeoPixel_Rear_ON() {
  pixels.setPixelColor(4, pixels.Color(255, 0, 0)); // 4番目のLEDを(R,G,B)=(255,0,0)⇔赤に点灯
  pixels.setPixelColor(5, pixels.Color(255, 0, 0)); // 5番目のLEDを(R,G,B)=(255,0,0)⇔赤に点灯
  pixels.show(); // LEDに反映
}
void NeoPixel_Rear_OFF() {
  pixels.setPixelColor(4, pixels.Color(0, 0, 0)); // 4番目のLEDを消灯
  pixels.setPixelColor(5, pixels.Color(0, 0, 0)); // 5番目のLEDを消灯
  pixels.show(); // LEDに反映
}
void NeoPixel_Left_ON() {
  pixels.setPixelColor(6, pixels.Color(255, 0, 0)); // 6番目のLEDを(R,G,B)=(255,0,0)⇔赤に点灯
  pixels.setPixelColor(7, pixels.Color(255, 0, 0)); // 7番目のLEDを(R,G,B)=(255,0,0)⇔赤に点灯
  pixels.show(); // LEDに反映
}
void NeoPixel_Left_OFF() {
  pixels.setPixelColor(6, pixels.Color(0, 0, 0)); // 6番目のLEDを消灯
  pixels.setPixelColor(7, pixels.Color(0, 0, 0)); // 7番目のLEDを消灯
  pixels.show(); // LEDに反映
}
void NeoPixel_Color(uint8_t r, uint8_t g, uint8_t b) {
  pixels.clear();
  pixels.fill(pixels.Color(r, g, b));
  pixels.show();
}


/*******************************************************************************************/
/* collect_tof_data                                                                              
/*処理： ヘッダーが来たら6バイト距離のデータを取得するこれによって各壁の検知を行う
/*
/*更新者：清田侑希　2025/1/26
/*清田侑希　2025/2/23 変更点：壁の有無を各方向の2つのセンサーがともに壁を検知したときのみ壁とみなす
/*清田侑希　2025/3/30 変更点：tofセンサーをの値を生のデータのまま送信する
/*******************************************************************************************/
void get_tof_data() {
    if (Serial3.available() > 0) {
      byte incomingByte = Serial3.read(); // Serial3から1バイト読み取り
        if (!headerDetected) {
          if (incomingByte == 255) {  // ヘッダー（255）を検出
            headerDetected = true;  // ヘッダー検出
            receivedIndex = 0;      // データ格納位置をリセット
            Serial.println("Header detected!"); // デバッグ用
          }
        } else {
          receivedData[receivedIndex++] = incomingByte; // データを格納
    
          // 6つのセンサー分データを受信した場合
          if (receivedIndex == 6) {
            // 受信したデータを表示
            Serial.println("Received data:");
            for (int i = 0; i < receivedIndex; i++) {
              Serial.print("Sensor ");
              Serial.print(i + 1); // センサー番号（1から6）
              Serial.print(": ");
              Serial.println(receivedData[i], DEC); // 受信データを10進数で表示
            }
              //壁の有無を変数に代入
            if (receivedData[0] <= 175 && receivedData[3] <= 175) {
                front_wall = true;
                Serial.println("front_wall");
            } else {
                front_wall = false;
            }
            
            if (receivedData[1] <= 175 && receivedData[2] <= 175) {
                left_wall = true;
                Serial.println("left_wall");
            } else {
                left_wall = false;
            }
            
            if (receivedData[4] <= 175 && receivedData[5] <= 175) {
                right_wall = true;
                Serial.println("right_wall");
            } else {
                right_wall = false;
            }
            Serial.println("All data received");
            Status = 1;
            headerDetected = false; // ヘッダーをリセット
          }
        }
   
        }
      }
/*******************************************************************************************/
/* Servo_rescue                                                                              
/*処理：サーボモーターによるレスキューキットの排出
/*
/*更新者：清田侑希 2025/3/26
/*
/*******************************************************************************************/
void motor_servo(int motor_speed, int seconds) {
  int pulse_width = 1500-motor_speed*8;
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
  int pulse_width = 1500-motor_speed*8;
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
  motor_servo(100,200);
  motor_servo(0,1000);
  motor_servo(-100,200);
  motor_servo(0,1000);
  delay(150);
  servo1_kit--;
}

void servo_res2(){
  motor_servo2(-100,200);
  motor_servo2(0,1000);
  motor_servo2(100,200);
  motor_servo2(0,1000);
  delay(150);
  servo2_kit--;
}

/*******************************************************************************************/
/* get_victim_data                                                                              
/*処理：OpenMVによって被災者の発見
/*
/*更新者：清田侑希 2025/2/23
/*清田侑希 2025/3/26 変更点：被災者を発見したときにLEDを点灯させる
/* 清田侑希 2025/3/28 変更点：被災者を発見したときにサーボモーターを動かす
/*******************************************************************************************/
//右側のカメラ
void serialEvent7(){
  if (Serial7.available()>0 && !VictimisAlready[x][y]){
    VictimisAlready[x][y] = true;
    int recivedData7 = Serial7.read();
    if (recivedData7 == 1){
      Serial.println("H_victim");
      for ( int i =0;i < 5 ;i++){
        NeoPixel_Color(255,0,0);
        delay(650);
        pixels.clear();
        pixels.show();
        delay(650);
      }
      if(servo1_kit >= 2){
        servo_res();
        delay(650);
        servo_res();
        delay(650);
      }else if(servo1_kit == 1 && servo2_kit >= 1 ){
        servo_res();
        delay(500);
        Position[0] = 7450; //右に回転
        Position[1] = 7450;
        Position[2] = 7450;
        Position[3] = 7450;
        sms_sts.SyncWritePosEx(ID, 4, Position, Speed, ACC);
        delay(1700);
        delay(300);
        Position[0] = 7450; //右に回転
        Position[1] = 7450;
        Position[2] = 7450;
        Position[3] = 7450;
        sms_sts.SyncWritePosEx(ID, 4, Position, Speed, ACC);
        delay(1700);
        delay(300);
        servo_res2();
        delay(500);
        Position[0] = 7450; //右に回転
        Position[1] = 7450;
        Position[2] = 7450;
        Position[3] = 7450;
        sms_sts.SyncWritePosEx(ID, 4, Position, Speed, ACC);
        delay(1700);
        delay(300);
        Position[0] = 7450; //右に回転
        Position[1] = 7450;
        Position[2] = 7450;
        Position[3] = 7450;
        sms_sts.SyncWritePosEx(ID, 4, Position, Speed, ACC);
        delay(1700);
        delay(300);
      }else if(servo2_kit >= 2){
        Position[0] = 7450; //右に回転
        Position[1] = 7450;
        Position[2] = 7450;
        Position[3] = 7450;
        sms_sts.SyncWritePosEx(ID, 4, Position, Speed, ACC);
        delay(1700);
        delay(300);
        Position[0] = 7450; //右に回転
        Position[1] = 7450;
        Position[2] = 7450;
        Position[3] = 7450;
        sms_sts.SyncWritePosEx(ID, 4, Position, Speed, ACC);
        delay(1700);
        delay(300);
        servo_res2();
        delay(500);
        servo_res2();
        delay(500);
        Position[0] = 7450; //右に回転
        Position[1] = 7450;
        Position[2] = 7450;
        Position[3] = 7450;
        sms_sts.SyncWritePosEx(ID, 4, Position, Speed, ACC);
        delay(1700);
        delay(300);
        Position[0] = 7450; //右に回転
        Position[1] = 7450;
        Position[2] = 7450;
        Position[3] = 7450;
        sms_sts.SyncWritePosEx(ID, 4, Position, Speed, ACC);
        delay(1700);
        delay(300);
        
      }else if(servo1_kit == 1){
          servo_res();
          delay(500);
      }else if(servo2_kit == 1){
        Position[0] = 7450; //右に回転
        Position[1] = 7450;
        Position[2] = 7450;
        Position[3] = 7450;
        sms_sts.SyncWritePosEx(ID, 4, Position, Speed, ACC);
        delay(1700);
        delay(300);
        Position[0] = 7450; //右に回転
        Position[1] = 7450;
        Position[2] = 7450;
        Position[3] = 7450;
        sms_sts.SyncWritePosEx(ID, 4, Position, Speed, ACC);
        delay(1700);
        delay(300);
        servo_res2();
        delay(500);
        Position[0] = 7450; //右に回転
        Position[1] = 7450;
        Position[2] = 7450;
        Position[3] = 7450;
        sms_sts.SyncWritePosEx(ID, 4, Position, Speed, ACC);
        delay(1700);
        delay(300);
        Position[0] = 7450; //右に回転
        Position[1] = 7450;
        Position[2] = 7450;
        Position[3] = 7450;
        sms_sts.SyncWritePosEx(ID, 4, Position, Speed, ACC);
        delay(1700);
        delay(300);
      }


    }else if (recivedData7 == 2){
        Serial.println("S_victim");
        for ( int i =0;i < 5 ;i++){
          NeoPixel_Color(255,200,0);
          delay(650);
          pixels.clear();
          pixels.show();
          delay(650);
        }
        if (servo1_kit >= 1){
          servo_res();
          delay(500);
        }else if (servo2_kit >= 1){
        Position[0] = 7450; //右に回転
        Position[1] = 7450;
        Position[2] = 7450;
        Position[3] = 7450;
        sms_sts.SyncWritePosEx(ID, 4, Position, Speed, ACC);
        delay(1700);
        delay(300);
        Position[0] = 7450; //右に回転
        Position[1] = 7450;
        Position[2] = 7450;
        Position[3] = 7450;
        sms_sts.SyncWritePosEx(ID, 4, Position, Speed, ACC);
        delay(1700);
        delay(300);
        servo_res2();
        delay(500);
        Position[0] = 7450; //右に回転
        Position[1] = 7450;
        Position[2] = 7450;
        Position[3] = 7450;
        sms_sts.SyncWritePosEx(ID, 4, Position, Speed, ACC);
        delay(1700);
          delay(300);
        Position[0] = 7450; //右に回転
        Position[1] = 7450;
        Position[2] = 7450;
        Position[3] = 7450;
        sms_sts.SyncWritePosEx(ID, 4, Position, Speed, ACC);
        delay(1700);
          delay(300);
        }
      }else if(recivedData7 == 3){
        Serial.println("U_victim");
        for ( int i =0;i < 5 ;i++){
          NeoPixel_Color(0,255,0);
          delay(650);
          pixels.clear();
          pixels.show();
          delay(650);
        }
      }else if (recivedData7 == 4){
        Serial.println("red_victim");
        for ( int i =0;i < 3 ;i++){
          NeoPixel_Color(255,0,0);
          delay(1000);
          pixels.clear();
          pixels.show();
          delay(1000);
        }
      if(servo1_kit >= 2){
        servo_res();
        delay(500);
        servo_res();
        delay(500);
      }else if(servo1_kit == 1 && servo2_kit >= 1 ){
        servo_res();
        delay(500);
        Position[0] = 7450; //右に回転
        Position[1] = 7450;
        Position[2] = 7450;
        Position[3] = 7450;
        sms_sts.SyncWritePosEx(ID, 4, Position, Speed, ACC);
        delay(1700);
        delay(300);
        Position[0] = 7450; //右に回転
        Position[1] = 7450;
        Position[2] = 7450;
        Position[3] = 7450;
        sms_sts.SyncWritePosEx(ID, 4, Position, Speed, ACC);
        delay(1700);
        delay(300);
        servo_res2();
        delay(500);
        Position[0] = 7450; //右に回転
        Position[1] = 7450;
        Position[2] = 7450;
        Position[3] = 7450;
        sms_sts.SyncWritePosEx(ID, 4, Position, Speed, ACC);
        delay(1700);
        delay(300);
        Position[0] = 7450; //右に回転
        Position[1] = 7450;
        Position[2] = 7450;
        Position[3] = 7450;
        sms_sts.SyncWritePosEx(ID, 4, Position, Speed, ACC);
        delay(1700);
        delay(300);
      }else if(servo2_kit >= 2){
        Position[0] = 7450; //右に回転
        Position[1] = 7450;
        Position[2] = 7450;
        Position[3] = 7450;
        sms_sts.SyncWritePosEx(ID, 4, Position, Speed, ACC);
        delay(1700);
        delay(300);
        Position[0] = 7450; //右に回転
        Position[1] = 7450;
        Position[2] = 7450;
        Position[3] = 7450;
        sms_sts.SyncWritePosEx(ID, 4, Position, Speed, ACC);
        delay(1700);
        delay(300);
        servo_res2();
        delay(500);
        servo_res2();
        delay(500);
        Position[0] = 7450; //右に回転
        Position[1] = 7450;
        Position[2] = 7450;
        Position[3] = 7450;
        sms_sts.SyncWritePosEx(ID, 4, Position, Speed, ACC);
        delay(1700);
        delay(300);
        Position[0] = 7450; //右に回転
        Position[1] = 7450;
        Position[2] = 7450;
        Position[3] = 7450;
        sms_sts.SyncWritePosEx(ID, 4, Position, Speed, ACC);
        delay(1700);
        delay(300);
        
      }else if(servo1_kit == 1){
          servo_res();
          delay(500);
      }else if(servo2_kit == 1){
        Position[0] = 7450; //右に回転
        Position[1] = 7450;
        Position[2] = 7450;
        Position[3] = 7450;
        sms_sts.SyncWritePosEx(ID, 4, Position, Speed, ACC);
        delay(1700);
        delay(300);
        Position[0] = 7450; //右に回転
        Position[1] = 7450;
        Position[2] = 7450;
        Position[3] = 7450;
        sms_sts.SyncWritePosEx(ID, 4, Position, Speed, ACC);
        delay(1700);
        delay(300);
        servo_res2();
        delay(500);
        Position[0] = 7450; //右に回転
        Position[1] = 7450;
        Position[2] = 7450;
        Position[3] = 7450;
        sms_sts.SyncWritePosEx(ID, 4, Position, Speed, ACC);
        delay(1700);
        delay(300);
        Position[0] = 7450; //右に回転
        Position[1] = 7450;
        Position[2] = 7450;
        Position[3] = 7450;
        sms_sts.SyncWritePosEx(ID, 4, Position, Speed, ACC);
        delay(1700);
        delay(300);
      }
      }else if(recivedData7 == 5){
        Serial.println("yellow_victim");
        for ( int i =0;i < 3 ;i++){
          NeoPixel_Color(255,200,0);
          delay(1000);
          pixels.clear();
          pixels.show();
          delay(1000);
        }
        if (servo1_kit >= 1){
          servo_res();
          delay(500);
        }else if (servo2_kit >= 1){
          
        Position[0] = 7450; //右に回転
        Position[1] = 7450;
        Position[2] = 7450;
        Position[3] = 7450;
        sms_sts.SyncWritePosEx(ID, 4, Position, Speed, ACC);
        delay(1700);
        delay(300);
        Position[0] = 7450; //右に回転
        Position[1] = 7450;
        Position[2] = 7450;
        Position[3] = 7450;
        sms_sts.SyncWritePosEx(ID, 4, Position, Speed, ACC);
        delay(1700);
        delay(300);
        servo_res2();
        delay(500);
        Position[0] = 7450; //右に回転
        Position[1] = 7450;
        Position[2] = 7450;
        Position[3] = 7450;
        sms_sts.SyncWritePosEx(ID, 4, Position, Speed, ACC);
        delay(1700);
        delay(300);
        Position[0] = 7450; //右に回転
        Position[1] = 7450;
        Position[2] = 7450;
        Position[3] = 7450;
        sms_sts.SyncWritePosEx(ID, 4, Position, Speed, ACC);
        delay(1700);
        delay(300);
        }
      }else if(recivedData7 == 6){
        Serial.println("green_victim");
        for ( int i =0;i < 3 ;i++){
          NeoPixel_Color(0,255,0);
          delay(1000);
          pixels.clear();
          pixels.show();
          delay(1000);

        }
      }
    }
}

/*******************************************************************************************/
/* get_victim_data2                                                                              
/*処理：OpenMVによって被災者を発見する
/*
/*更新者：清田侑希　2025/2/23
/*清田侑希　2025/3/26 変更点：被災者を発見したときにLEDを点灯させる
/* 清田侑希　2025/3/28 変更点：被災者を発見したときにサーボモーターを動かす
/*******************************************************************************************/
//左側のカメラ

void serialEvent8(){
  if (Serial8.available()>0 && !VictimisAlready[x][y]){
    VictimisAlready[x][y] = true;
    int recivedData = Serial8.read();
    if (recivedData == 1){
      Serial.println("H_victim");
      for ( int i =0;i < 5 ;i++){
        NeoPixel_Color(255,0,0);
        delay(650);
        pixels.clear();
        pixels.show();
        delay(650);
      }
      if(servo2_kit >= 2){
        servo_res2();
        delay(650);
        servo_res2();
        delay(650);
      }else if(servo2_kit == 1 && servo1_kit >= 1 ){
        servo_res2();
        delay(500);
        Position[0] = 7450; //右に回転
        Position[1] = 7450;
        Position[2] = 7450;
        Position[3] = 7450;
        sms_sts.SyncWritePosEx(ID, 4, Position, Speed, ACC);
        delay(1700);
        delay(300);
        Position[0] = 7450; //右に回転
        Position[1] = 7450;
        Position[2] = 7450;
        Position[3] = 7450;
        sms_sts.SyncWritePosEx(ID, 4, Position, Speed, ACC);
        delay(1700);
        delay(300);
        servo_res();
        delay(500);
        Position[0] = 7450; //右に回転
        Position[1] = 7450;
        Position[2] = 7450;
        Position[3] = 7450;
        sms_sts.SyncWritePosEx(ID, 4, Position, Speed, ACC);
        delay(1700);
        delay(300);
        Position[0] = 7450; //右に回転
        Position[1] = 7450;
        Position[2] = 7450;
        Position[3] = 7450;
        sms_sts.SyncWritePosEx(ID, 4, Position, Speed, ACC);
        delay(1700);
        delay(300);
      }else if(servo1_kit >= 2){
        Position[0] = 7450; //右に回転
        Position[1] = 7450;
        Position[2] = 7450;
        Position[3] = 7450;
        sms_sts.SyncWritePosEx(ID, 4, Position, Speed, ACC);
        delay(1700);
        delay(300);
        Position[0] = 7450; //右に回転
        Position[1] = 7450;
        Position[2] = 7450;
        Position[3] = 7450;
        sms_sts.SyncWritePosEx(ID, 4, Position, Speed, ACC);
        delay(1700);
        delay(300);
        servo_res();
        delay(500);
        servo_res();
        delay(500);
        Position[0] = 7450; //右に回転
        Position[1] = 7450;
        Position[2] = 7450;
        Position[3] = 7450;
        sms_sts.SyncWritePosEx(ID, 4, Position, Speed, ACC);
        delay(1700);
        delay(300);
        Position[0] = 7450; //右に回転
        Position[1] = 7450;
        Position[2] = 7450;
        Position[3] = 7450;
        sms_sts.SyncWritePosEx(ID, 4, Position, Speed, ACC);
        delay(1700);
        delay(300);
        
      }else if(servo2_kit == 1){
          servo_res();
          delay(500);
      }else if(servo1_kit == 1){
        Position[0] = 7450; //右に回転
        Position[1] = 7450;
        Position[2] = 7450;
        Position[3] = 7450;
        sms_sts.SyncWritePosEx(ID, 4, Position, Speed, ACC);
        delay(1700);
        delay(300);
        Position[0] = 7450; //右に回転
        Position[1] = 7450;
        Position[2] = 7450;
        Position[3] = 7450;
        sms_sts.SyncWritePosEx(ID, 4, Position, Speed, ACC);
        delay(1700);
        delay(300);
        servo_res();
        delay(500);
        Position[0] = 7450; //右に回転
        Position[1] = 7450;
        Position[2] = 7450;
        Position[3] = 7450;
        sms_sts.SyncWritePosEx(ID, 4, Position, Speed, ACC);
        delay(1700);
        delay(300);
        Position[0] = 7450; //右に回転
        Position[1] = 7450;
        Position[2] = 7450;
        Position[3] = 7450;
        sms_sts.SyncWritePosEx(ID, 4, Position, Speed, ACC);
        delay(1700);
        delay(300);
      }
    }else if (recivedData == 2){
        Serial.println("S_victim");
        for ( int i =0;i < 5 ;i++){
          NeoPixel_Color(255,200,0);
          delay(650);
          pixels.clear();
          pixels.show();
          delay(650);
        }
        if (servo2_kit >= 1){
          servo_res2();
          delay(500);
        }else if (servo1_kit >= 1){
        Position[0] = 7450; //右に回転
        Position[1] = 7450;
        Position[2] = 7450;
        Position[3] = 7450;
        sms_sts.SyncWritePosEx(ID, 4, Position, Speed, ACC);
        delay(1700);
        delay(300);
        Position[0] = 7450; //右に回転
        Position[1] = 7450;
        Position[2] = 7450;
        Position[3] = 7450;
        sms_sts.SyncWritePosEx(ID, 4, Position, Speed, ACC);
        delay(1700);
        delay(300);
        servo_res();
        delay(500);
        Position[0] = 7450; //右に回転
        Position[1] = 7450;
        Position[2] = 7450;
        Position[3] = 7450;
        sms_sts.SyncWritePosEx(ID, 4, Position, Speed, ACC);
        delay(1700);
          delay(300);
        Position[0] = 7450; //右に回転
        Position[1] = 7450;
        Position[2] = 7450;
        Position[3] = 7450;
        sms_sts.SyncWritePosEx(ID, 4, Position, Speed, ACC);
        delay(1700);
          delay(300);
        }
      }else if(recivedData == 3){
        Serial.println("U_victim");
        for ( int i =0;i < 5 ;i++){
          NeoPixel_Color(0,255,0);
          delay(650);
          pixels.clear();
          pixels.show();
          delay(500);
        }
      }else if (recivedData == 4){
        Serial.println("red_victim");
        for ( int i =0;i < 3 ;i++){
          NeoPixel_Color(255,0,0);
          delay(1000);
          pixels.clear();
          pixels.show();
          delay(1000);
        }
      if(servo2_kit >= 2){
        servo_res2();
        delay(500);
        servo_res2();
        delay(500);
      }else if(servo2_kit == 1 && servo1_kit >= 1 ){
        servo_res2();
        delay(500);
        Position[0] = 7450; //右に回転
        Position[1] = 7450;
        Position[2] = 7450;
        Position[3] = 7450;
        sms_sts.SyncWritePosEx(ID, 4, Position, Speed, ACC);
        delay(1700);
        delay(300);
        Position[0] = 7450; //右に回転
        Position[1] = 7450;
        Position[2] = 7450;
        Position[3] = 7450;
        sms_sts.SyncWritePosEx(ID, 4, Position, Speed, ACC);
        delay(1700);
        delay(300);
        servo_res();
        delay(500);
        Position[0] = 7450; //右に回転
        Position[1] = 7450;
        Position[2] = 7450;
        Position[3] = 7450;
        sms_sts.SyncWritePosEx(ID, 4, Position, Speed, ACC);
        delay(1700);
        delay(300);
        Position[0] = 7450; //右に回転
        Position[1] = 7450;
        Position[2] = 7450;
        Position[3] = 7450;
        sms_sts.SyncWritePosEx(ID, 4, Position, Speed, ACC);
        delay(1700);
        delay(300);
      }else if(servo1_kit >= 2){
        Position[0] = 7450; //右に回転
        Position[1] = 7450;
        Position[2] = 7450;
        Position[3] = 7450;
        sms_sts.SyncWritePosEx(ID, 4, Position, Speed, ACC);
        delay(1700);
        delay(300);
        Position[0] = 7450; //右に回転
        Position[1] = 7450;
        Position[2] = 7450;
        Position[3] = 7450;
        sms_sts.SyncWritePosEx(ID, 4, Position, Speed, ACC);
        delay(1700);
        delay(300);
        servo_res();
        delay(500);
        servo_res();
        delay(500);
        Position[0] = 7450; //右に回転
        Position[1] = 7450;
        Position[2] = 7450;
        Position[3] = 7450;
        sms_sts.SyncWritePosEx(ID, 4, Position, Speed, ACC);
        delay(1700);
        delay(300);
        Position[0] = 7450; //右に回転
        Position[1] = 7450;
        Position[2] = 7450;
        Position[3] = 7450;
        sms_sts.SyncWritePosEx(ID, 4, Position, Speed, ACC);
        delay(1700);
        delay(300);
        
      }else if(servo2_kit == 1){
          servo_res();
          delay(500);
      }else if(servo1_kit == 1){
        Position[0] = 7450; //右に回転
        Position[1] = 7450;
        Position[2] = 7450;
        Position[3] = 7450;
        sms_sts.SyncWritePosEx(ID, 4, Position, Speed, ACC);
        delay(1700);
        delay(300);
        Position[0] = 7450; //右に回転
        Position[1] = 7450;
        Position[2] = 7450;
        Position[3] = 7450;
        sms_sts.SyncWritePosEx(ID, 4, Position, Speed, ACC);
        delay(1700);
        delay(300);
        servo_res();
        delay(500);
        Position[0] = 7450; //右に回転
        Position[1] = 7450;
        Position[2] = 7450;
        Position[3] = 7450;
        sms_sts.SyncWritePosEx(ID, 4, Position, Speed, ACC);
        delay(1700);
        delay(300);
        Position[0] = 7450; //右に回転
        Position[1] = 7450;
        Position[2] = 7450;
        Position[3] = 7450;
        sms_sts.SyncWritePosEx(ID, 4, Position, Speed, ACC);
        delay(1700);
        delay(300);
      }
      }else if(recivedData == 5){
        Serial.println("yellow_victim");
        for ( int i =0;i < 3 ;i++){
          NeoPixel_Color(255,200,0);
          delay(1000);
          pixels.clear();
          pixels.show();
          delay(1000);
        }
        if (servo2_kit >= 1){
          servo_res2();
          delay(500);
        }else if (servo1_kit >= 1){
        Position[0] = 7450; //右に回転
        Position[1] = 7450;
        Position[2] = 7450;
        Position[3] = 7450;
        sms_sts.SyncWritePosEx(ID, 4, Position, Speed, ACC);
        delay(1700);
        delay(300);
        Position[0] = 7450; //右に回転
        Position[1] = 7450;
        Position[2] = 7450;
        Position[3] = 7450;
        sms_sts.SyncWritePosEx(ID, 4, Position, Speed, ACC);
        delay(1700);
        delay(300);
        servo_res();
        delay(500);
        Position[0] = 7450; //右に回転
        Position[1] = 7450;
        Position[2] = 7450;
        Position[3] = 7450;
        sms_sts.SyncWritePosEx(ID, 4, Position, Speed, ACC);
        delay(1700);
          delay(300);
        Position[0] = 7450; //右に回転
        Position[1] = 7450;
        Position[2] = 7450;
        Position[3] = 7450;
        sms_sts.SyncWritePosEx(ID, 4, Position, Speed, ACC);
        delay(1700);
          delay(300);
        }
      }else if(recivedData == 6){
        Serial.println("green_victim");
        for ( int i =0;i < 3 ;i++){
          NeoPixel_Color(0,255,0);
          delay(1000);
          pixels.clear();
          pixels.show();
          delay(1000);

        }
      }
    }
}

/*******************************************************************************************/
/* 右回転                                                                              
/*処理：MPU6050で取得したデータをもとに90度右回転するまでモーターを回し続ける
/*変更点：ギア搭載による回転方向の反転と適切なdelayの実装
/*更新者：清田侑希　2025/1/25
/*　清田侑希　2025/3/12 変更点：90度モーターのみで回転し、90度回らなかったら補正を行う
/*　清田侑希　2025/3/28 変更点：↑の機能を削除する
/*******************************************************************************************/
void migi(){
  tiziki_kaitenn();
  delay(50);
  if (kakudo_true2 <=90){
    mawasu = kakudo_true2 +280;
    kakudo_true2 =kakudo_true2 +360;
    chousei =1;
  }else if (kakudo_true2 <= 360){
    mawasu = kakudo_true2-80;
  }
  //仮にバンプなどで90度回らなかったら90度回るまで補正し続ける
  while (kakudo_true2 > mawasu){
  Position[0] = 440; //右に回転
  Position[1] = 440;
  Position[2] = 440;
  Position[3] = 440;
  sms_sts.SyncWritePosEx(ID, 4, Position, Speed, ACC);
  delay(100);
  tiziki_kaitenn();
  serialEvent7();
  serialEvent8();
  if (chousei ==1){
    if (kakudo_true2 <=90){
      kakudo_true2 =kakudo_true2 +360;
    }else {
      chousei =0;
      }
    }
    delay(50);
  }
  
  delay(200);
  //tiziki(); //角度補正            
}




/*******************************************************************************************/
/* 左回転                                                                              
/*処理：MPU6050で取得したデータをもとに90度左回転するまでモーターを回し続ける
/*変更点：ギア搭載による回転方向の反転と適切なdelayの実装
/*更新者：清田侑希　2025/1/25
/*　清田侑希　2025/3/12 変更点：90度モーターのみで回転し、90度回らなかったら補正を行う
/*　清田侑希　2025/3/28 変更点：↑の機能を削除する
/*******************************************************************************************/
void hidari(){
  //どこまで回すかの数値を処理
  tiziki_kaitenn();
  delay(50);
  if (kakudo_true2 >= 270){
    mawasu = kakudo_true2 - 280; 
    kakudo_true2 = kakudo_true2 - 360;
    chousei = 1; 
  }else if (kakudo_true2 >= 0){
    mawasu = kakudo_true2 + 80;
  }
    //90度回転しなかったときに角度の補正を行う
  while (kakudo_true2 < mawasu){
  Position[0] = -440; //左に回転
  Position[1] = -440;
  Position[2] = -440;
  Position[3] = -440;
  sms_sts.SyncWritePosEx(ID, 4, Position, Speed, ACC);
  delay(100);
  tiziki_kaitenn();
  serialEvent7();
  serialEvent8();
  if (chousei ==1){
    if (kakudo_true2 >= 270){
      kakudo_true2 =kakudo_true2 - 360;
      
    }else {
      chousei =0;
    }
  }
  delay(50);
  }
  delay(200);
  //tiziki(); //角度補正         
}

/*******************************************************************************************/
/*色タイルまたはバンプ                                                                              
/*処理：Serial1にデータが来たら1バイト読み込んで青、黒、ロードセルを検知する
/*Data 1:黒タイル　2：青タイル　3：左の障害物　4：右の障害物　5：前の壁　6：銀のタイル
/*更新者：清田侑希　2025/1/26
/*清田侑希　2025/2/23 変更点：障害物を検知して最初の動作の時に壁との距離を測るようにした
/*清田侑希　2025/3/28 変更点：障害物を検知したとき毎回壁との距離を測るようにした
/*******************************************************************************************/
void serialEvent1() {
  if (Serial1.available()>0){
  int receivedData2 = Serial1.read();
  Serial.println(receivedData2);
  delay(50);
  if(receivedData2 == 1){
    BlackTile = true;
    while (count2 >0){
      Position[0] = -303; //back
      Position[1] = 303;
      Position[2] = 303;
      Position[3] = -303;
      sms_sts.SyncWritePosEx(ID, 4, Position, Speed, ACC);
      Serial.println("Backing...");
      delay(100);
      count2--;
    }
    count2 = 40; //進むwhile文から出る
  }else if(receivedData2 == 2){
    Serial.println("blue_tile");
    delay(50);
    blue_count = true;
  }
  else if(receivedData2 == 5 || bump_giveup_count > 3){
    if(receivedData2== 5 || bump_giveup_count > 3){
        Position[0] = -2025; //後ろに下がる
        Position[1] = 2025;
        Position[2] = 2025;
        Position[3] = -2025;
        sms_sts.SyncWritePosEx(ID, 4, Position, Speed, ACC);
        Serial.println("Backing...");
        delay(1000);
        if (count2 < 20 && bump_giveup_count > 1){
            //座標の変更をもとに戻す
            Gap = true;
        }
        count2 = 40;
    }

  }else if(receivedData2 == 3 ){
    Serial.println("left");
    Position[0] = -1023; //左に回転
    Position[1] = -1023;
    Position[2] = -1023;
    Position[3] = -1023;
    sms_sts.SyncWritePosEx(ID, 4, Position, Speed, ACC);
    delay(314);
    delay(500);
    Position[0] = -1240; //後ろに下がる
    Position[1] = 1240;
    Position[2] = 1240;
    Position[3] = -1240;
    sms_sts.SyncWritePosEx(ID, 4, Position, Speed, ACC);
    delay(420);
    delay(500);
    Position[0] = 1534; //右に回転
    Position[1] = 1534;
    Position[2] = 1534;
    Position[3] = 1534;
    sms_sts.SyncWritePosEx(ID, 4, Position, Speed, ACC);
    delay(471);
    delay(500);
    //if (bump_giveup_count == 0){
      while (Serial3.available() > 0) {
        char receivedChar = Serial3.read(); // データを読み取る
        // 必要に応じて受信データを処理する
      }
      receivedIndex = 0; //tofデータを取得するためにバイト数を初期化する
      front_wall = false; //怖いので初期化
      while (receivedIndex != 6){
          get_tof_data();
      }
      if (receivedData[0] <= 90 && receivedData[3] <= 90&& receivedData[0] - receivedData[3] < 30 && receivedData[0] - receivedData[3] > -30){      
            if (count2 < 20 ){
                //座標の変更をもとに戻す
                Gap = true;
            }
            count2 = 46;  
      }
    //}
    count2= count2-6;
    bump_giveup_count++;
  }else if(receivedData2 == 4){
    Serial.println("right");
    Position[0] = 1023; //右に回転
    Position[1] = 1023;
    Position[2] = 1023;
    Position[3] = 1023;
    sms_sts.SyncWritePosEx(ID, 4, Position, Speed, ACC);
    delay(314);
    delay(500);
    Position[0] = -1240; //後ろに下がる
    Position[1] = 1240;
    Position[2] = 1240;
    Position[3] = -1240;
    sms_sts.SyncWritePosEx(ID, 4, Position, Speed, ACC);
    delay(420);
    delay(500);
    Position[0] = -1534; //左に回転
    Position[1] = -1534;
    Position[2] = -1534;
    Position[3] = -1534;
    sms_sts.SyncWritePosEx(ID, 4, Position, Speed, ACC);
    delay(471);
    delay(500);
    //if (bump_giveup_count == 0){
    while (Serial3.available() > 0) {
      char receivedChar = Serial3.read(); // データを読み取る
      // 必要に応じて受信データを処理する
    }
    receivedIndex = 0; //tofデータを取得するためにバイト数を初期化する
    front_wall = false; //怖いので初期化
    while (receivedIndex != 6){
        get_tof_data();
    }
    if (receivedData[0] <= 90 && receivedData[3] <= 90 && receivedData[0] - receivedData[3] < 30 && receivedData[0] - receivedData[3] > -30){
     
          if (count2 < 20 ){
              //座標の変更をもとに戻す
              Gap = true;
          }
          count2 = 46;  
    }
  //}
    count2= count2-6;
    bump_giveup_count++;
  }else if(receivedData2 == 6){
        Serial.println("Check_Point");
        //現在の座標をチェックポイントの座標として代入する
        CheckX = x;
        CheckY = y;
        CheckD = Direction;
        NeoPixel_Rear_ON(); //デバッグ用
        delay(200);
        NeoPixel_Rear_OFF();

  }else{
    Serial.println("No sensors");
    delay(100);
  }

  }

}

/*******************************************************************************************/
/* 直進                                                                              
/*処理：モーターを回しながら直進この時カラーセンサーとロードセル、カメラの割り込みと坂の検知が入る
/*
/*更新者：清田侑希　2025/1/26
/*清田侑希　2025/3/27 変更点：tofセンサーの値を元にして壁と機体の角度のズレを計算し壁と常に平行になるように補正する
/*清田侑希　2025/3/30　変更点：tofセンサーの値を元にしてタイルの中心とのズレを計算し、常に左右方向でタイルの中心にいるように補正するようにする
/*******************************************************************************************/
void susumu_heitan() {
  count2 = 0; //いったん初期化
  if (right_wall == true && bump_giveup_count == 0 &&(receivedData[4] - receivedData[5] >= 10 || receivedData[4] - receivedData[5] <=-10)){
    if (receivedData[4] - receivedData[5] < 0){
        rag = receivedData[5] - receivedData[4];
        tan_value = rag / body_len;
        theta_rad = atan(tan_value);
        theta_deg = theta_rad * 180.0 / PI;  // 度数法に変換
        Serial.print("rag_deg:");
        Serial.println(theta_deg);
        for (int i; i < theta_deg; i++){
          Position[0] = -82; //左に回転
          Position[1] = -82;
          Position[2] = -82;
          Position[3] = -82;
          sms_sts.SyncWritePosEx(ID, 4, Position, Speed, ACC);
          delay(20);
        }
    } else if (receivedData[4] - receivedData[5] > 0){
        rag = receivedData[4] - receivedData[5];
        tan_value = rag / body_len;
        theta_rad = atan(tan_value);
        theta_deg = theta_rad * 180.0 / PI;  // 度数法に変換
        Serial.print("rag_deg:");
        Serial.println(theta_deg);
        for (int i; i < theta_deg; i++){
          Position[0] = 82; //右に回転
          Position[1] = 82;
          Position[2] = 82;
          Position[3] = 82;
          sms_sts.SyncWritePosEx(ID, 4, Position, Speed, ACC);
          delay(20);
        }
    }
    delay(300);
  }else if (Status != 3) {
    delay(100);
    tiziki();
    delay(300);
  }
  ave_tof_data = (receivedData[4] + receivedData[5])/2;
  rag2 = 97.5 - ave_tof_data;
  if((rag2 >= 20 || rag2 <= -20)&&right_wall == true){
    if(rag2 < 0){
      rag2 = 0-rag2;
    }
    tan_value2 = rag2/300;
    theta_rad2 = atan(tan_value2);
    theta_deg2 = theta_rad2 * 180.0 / PI;  // 度数法に変換
    if( rag2 > 0){
        Serial.println(theta_deg2);
        for (int i; i < theta_deg2; i++){
          Position[0] = -70; //左に回転
          Position[1] = -70;
          Position[2] = -70;
          Position[3] = -70;
          sms_sts.SyncWritePosEx(ID, 4, Position, Speed, ACC);
          delay(20);
        }
    }else if(rag2 < 0){
        Serial.println(theta_deg2);
        for (int i; i < theta_deg2; i++){
          Position[0] = 70; //右に回転
          Position[1] = 70;
          Position[2] = 70;
          Position[3] = 70;
          sms_sts.SyncWritePosEx(ID, 4, Position, Speed, ACC);
          delay(20);
        }
    }
    run_len = sqrt(pow(rag2, 2) + pow(300, 2));
    run_len_time = run_len/300;
    delay(200);
  }
    while (Serial1.available() > 0) {
      char receivedChar = Serial1.read(); // データを読み取る
      // 必要に応じて受信データを処理する
    }
  while (count2  <susumu_kaisuu) {
    Serial.println("Going...");
    Position[0] = 303*run_len_time; //go
    Position[1] = -303*run_len_time;
    Position[2] = -303*run_len_time;
    Position[3] = 303*run_len_time;
    sms_sts.SyncWritePosEx(ID, 4, Position, Speed, ACC);
    delay(90*run_len_time);
    serialEvent1(); //color_load
    serialEvent7();//_victim_camera1
    serialEvent8();//_victim_camera2
    count2++;
  }
    if( (rag2 >= 15 || rag2 <= -15)&& right_wall == true){
      if( rag2 > 0){
        Serial.println(theta_deg2);
        for (int i; i < theta_deg2; i++){
          Position[0] = 70; //に右回転
          Position[1] = 70;
          Position[2] = 70;
          Position[3] = 70;
          sms_sts.SyncWritePosEx(ID, 4, Position, Speed, ACC);
          delay(20);
        }
    }else if(rag2 < 0){
        Serial.println(theta_deg2);
        for (int i; i < theta_deg2; i++){
          Position[0] = -70; //左に回転
          Position[1] = -70;
          Position[2] = -70;
          Position[3] = -70;
          sms_sts.SyncWritePosEx(ID, 4, Position, Speed, ACC);
          delay(20);
        }
    }
    }
    /*...........坂があるかどうか..............*/
    tiziki_2(); //get_roll_data
    delay(50);
    if (katamuki_true <-18){
        Serial.println("Slope_Ue");
        while(katamuki_true <-18){
            Serial.println("Climbing...");
            Position[0] = 400; //前に進む
            Position[1] = -400;
            Position[2] = -400;
            Position[3] = 400;
            sms_sts.SyncWritePosEx(ID, 4, Position, Speed, ACC);
            delay(100);
            tiziki_2(); //get_roll_data
            delay(100);
            Slope = true;
        }
        Position[0] = 8190; //前に進む
        Position[1] = -8190;
        Position[2] = -8190;
        Position[3] = 8190;
        sms_sts.SyncWritePosEx(ID, 4, Position, Speed, ACC);
        delay(2500);
        serialEvent1();
    }else if (katamuki_true >18){
        while(katamuki_true > 18){
            Serial.println("Down Hill...");
            Position[0] = 400; //前に進む
            Position[1] = -400;
            Position[2] = -400;
            Position[3] = 400;
            sms_sts.SyncWritePosEx(ID, 4, Position, Speed, ACC);
            delay(100);
            tiziki_2(); //get_roll_data
            delay(100);
            Slope = true;
        }
        Position[0] = 8190; //前に進む
        Position[1] = -8190;
        Position[2] = -8190;
        Position[3] = 8190;
        sms_sts.SyncWritePosEx(ID, 4, Position, Speed, ACC);
        delay(2500);
        serialEvent1();
    }

      /*............青タイルがある場合の待機............*/

  if (blue_count == true){
    Serial.println("Waiting...");
    delay(5500);
  }

    while (Serial3.available() > 0) {
        char receivedChar = Serial3.read(); // データを読み取る
        // 必要に応じて受信データを処理する
    }
    delay(150);
  /*..............変数初期化.................................*/
  count2 =0;
  bump_giveup_count =0;
  Status = 0;
  blue_count = false;
  run_len_time = 1;
}



/*アルゴリズム用変数*/
int8_t toutatu_zahyou[90][90];//そのマスの到達回数
int RightWeight = 0;
int FrontWeight = 0;
int LeftWeight = 0;

const uint8_t GoalX = 50;
const uint8_t GoalY = 50;
bool start_Gohome = false; //SSD1306に帰還アルゴリズムに入っているかどうかを送信するかの変数
uint8_t NowDirection = 0; //現在の方向

/*帰還用変数*/
int kabe_zahyou[90][90];//0000 の4ビットに絶対方向の東8西4南2北1をそれぞれ割り当てるみたいな
int cost[90][90];//マス目のコスト
bool reach_time[90][90];//到達の有無（帰還用）
long now_seconds;
long firstseconds;
bool StopFlag = false; //帰還アルゴリズムの停止フラグ

/*******************************************************************************************/
/* 座標などのデータ送信                                                                             
/*処理：Serial1を用いて帰還しているかどうか、右、前、左の重み、X,Y座標、方向のデータをSSD1306に送信する
/*
/*更新者：清田侑希　2025/1/30
/*　　　　
/*
/*******************************************************************************************/

void send_display(){
  // 各センサーの距離を取得し壁の有無を判定
  byte data_for_send[7];
  data_for_send[0] = (start_Gohome) ? 0x01 : 0x00;
  data_for_send[1] = (RightWeight);
  data_for_send[2] = (LeftWeight);
  data_for_send[3] = (FrontWeight);
  data_for_send[4] = (x);
  data_for_send[5] = (y);
  data_for_send[6] = (Direction);

  /*data_for_send[0] = (start_Gohome) ? 0x01 : 0x00;
  data_for_send[1] = (kabe_zahyou[x][y]);
  data_for_send[2] = (x);
  data_for_send[3] = (y);
  data_for_send[4] = (Direction);*/

  Serial3.write(255);
  Serial.println("Sent header: 255");
  delay(50);//wait
  // 判定データ送信
  for (int i = 0; i < 7; i++) {
    Serial3.write(data_for_send[i]);
  }
    
  

    Serial.print("Sent data:");//デバック用にシリアルモニタに送信
    for (int i = 0; i < 7; i++) {
      Serial.print(data_for_send[i], BIN);
      if (i < 6) Serial.print(", ");
    }
    Serial.println();


  delay(200); // 必要に応じて調整
}

/*******************************************************************************************/
/*行き止まりの検索                                                                              
/*処理：全てのマスについて、
/*      前後左右の三か所以上が行き止まりor壁のときそのマスの到達回数を+100する（マスの到達回数が100未満のとき）
/*
/*更新者：吉ノ薗2025/02/02いったん凍結
/*
/*******************************************************************************************/
/*void EffectiveDeadEnd(){

    int NorthReached = 0;
    int EastReached = 0;
    int WestReached = 0;
    int SouthReached = 0;

    for (int t = 1; t < 89; t++) {
        for (int j = 1; j < 89; j++) {

            int DeadEndCount = 0;

            NorthReached = toutatu_zahyou[t][j-1];
            EastReached  = toutatu_zahyou[t+1][j];
            WestReached  = toutatu_zahyou[t-1][j];
            SouthReached = toutatu_zahyou[t][j+1];
            
            if(toutatu_zahyou[t][j] < 100){

                if((NorthReached >= 100) || (kabe_zahyou[t][j] & 1)){DeadEndCount++;}//北に壁がある
                if((SouthReached >= 100) || (kabe_zahyou[t][j] & 2)){DeadEndCount++;}//南に壁がある
                if((WestReached >= 100) || (kabe_zahyou[t][j] & 4)){DeadEndCount++;}//西に壁がある
                if((EastReached >= 100) || (kabe_zahyou[t][j] & 8)){DeadEndCount++;}//東に壁がある

                //前後左右の三か所以上が行き止まりor壁のとき、そのマスの到達回数を+100する
                if(DeadEndCount >= 3){
                    toutatu_zahyou[t][j] += 100;
                }

            }
        }
    }
}*/

/*******************************************************************************************/
/* ジャッジ                                                                            
/*処理：前左右を重みづけして、行く方向を決定
/*      右+1、前+2、左+3することで基本的に右に行く拡張右手法を実現
/*      壁があったらさらにプラスする
/*
/*更新者：吉ノ薗2025/01/22
/*       清田侑希　2025/1/30    座標などのデータ更新
/*******************************************************************************************/
int8_t judge(){
    
    switch (Direction){//向きによって重みをつける
        case East:
            RightWeight = toutatu_zahyou[x][y+1] * 5 + 1;
            FrontWeight = toutatu_zahyou[x+1][y] * 5 + 2;
            LeftWeight = toutatu_zahyou[x][y-1] * 5 + 3;
            break;

        case North:
            RightWeight = toutatu_zahyou[x+1][y] * 5 + 1;
            FrontWeight = toutatu_zahyou[x][y-1] * 5 + 2;
            LeftWeight = toutatu_zahyou[x-1][y] * 5 + 3;
            break;

        case West:
            RightWeight = toutatu_zahyou[x][y-1] * 5 + 1;
            FrontWeight = toutatu_zahyou[x-1][y] * 5 + 2;
            LeftWeight = toutatu_zahyou[x][y+1] * 5 + 3;
            break;

        case South:
            RightWeight = toutatu_zahyou[x-1][y] * 5 + 1;
            FrontWeight = toutatu_zahyou[x][y+1] * 5 + 2;
            LeftWeight = toutatu_zahyou[x+1][y] * 5 + 3;
            break;

    }

    //壁がある場合１００にする
    if (right_wall){
        RightWeight = 100;
    }
    if (front_wall){
        FrontWeight = 100;
    }
    if (left_wall){
        LeftWeight = 100;
    }

    int8_t GoTo = 0;

    //どこへ行くか決める
    if(RightWeight < FrontWeight){
        GoTo = Right;
    }else{
        GoTo = Front;
    }
    if(LeftWeight < min(RightWeight,FrontWeight)){
        GoTo = Left;
    }

    if(toutatu_zahyou[x][y] > 100){toutatu_zahyou[x][y] = 100;}//捕捉：走行中に右の重みが94っていう出るはずのない値がでたためオーバーフローを疑いこの関数を導入。対処療法であるため根本的な解決には至っていない

    if(FrontWeight > 100){FrontWeight = 100;}//前・左・右の重みが100を越えていたら100にする
    if(RightWeight > 100){RightWeight = 100;}
    if(LeftWeight > 100){LeftWeight = 100;}

    if ((RightWeight == 100) && (FrontWeight == 100) && (LeftWeight == 100)){//if all wall
        GoTo = Back;
        toutatu_zahyou[x][y] = 50;//行き止まりだから効率化のため二度と行かないようにする
    }

    //send_display();
    RightWeight = 0;//怖いから初期化
    FrontWeight = 0;
    LeftWeight = 0;
    return GoTo;
}


/*******************************************************************************************/
/* 動く方向                                                                              
/*処理：それぞれの方位のときに行く方向によって座標を移動・方位を変更する
/*      後進のときは全部壁のときだから、現在の重みを+100する
/*
/*更新者：吉ノ薗2025/01/22
/*
/*******************************************************************************************/
void MoveTo(int8_t GoTo)
{
    //EffectiveDeadEnd();//より効率的な行き止まりの検索

    switch (Direction){
            case East:
                switch (GoTo){
                    case Right:
                        Status = 4;
                        y += 1;
                        Direction = South;
                        break;
                    
                    case Front:
                        Status = 3;
                        x += 1;
                        Direction = East;
                        break;
                    
                    case Left:
                        Status = 5;
                        y += -1;
                        Direction = North;
                        break;

                    case Back:
                        Status = 6;
                        x += -1;
                        Direction = West;
                        break;
                    }
                break;
            
            case North:
                switch (GoTo){
                    case Right:
                        Status = 4;
                        x += 1;
                        Direction = East;
                        break;
                    
                    case Front:
                        Status = 3;
                        y += -1;
                        Direction = North;
                        break;
                    
                    case Left:
                        Status = 5;
                        x += -1;
                        Direction = West;
                        break;

                    case Back:
                        Status = 6;
                        y += 1;
                        Direction = South;
                        break;
                    }
                break;
            
            case West:
                switch (GoTo){
                    case Right:
                        Status = 4;
                        y += -1;
                        Direction = North;
                        break;
                    
                    case Front:
                        Status = 3;
                        x += -1;
                        Direction = West;
                        break;
                    
                    case Left:
                        Status = 5;
                        y += 1;
                        Direction = South;
                        break;
                    
                    case Back:
                        Status = 6;
                        x += 1;
                        Direction = East;
                        
                        break;
                    }
                break;
            
            case South:
                switch (GoTo){
                    case Right:
                        Status = 4;
                        x += -1;
                        Direction = West;
                        break;
                    
                    case Front:
                        Status = 3;
                        y += 1;
                        Direction = South;
                        break;
                    
                    case Left:
                        Status = 5;
                        x += 1;
                        Direction = East;
                        break;
                    
                    case Back:
                        Status = 6;
                        y += -1;
                        Direction = North;
                        break;
                    }
                break;
        }
    toutatu_zahyou[x][y] += 1;//移動先のマスの到達回数をプラスしている
}



/*******************************************************************************************/
/* タイルの状態                                                                              
/*処理：それぞれの方向について、
/*      黒タイルの場合そのマスの重みを100にする
/*      黒タイルまたは"少しずれている"の信号が送られていたら一マス戻る
/*      坂の信号が送られていたら一マス進む
/*      黒タイルと坂信号の初期化
/*
/*更新者：吉ノ薗2025/01/22
/*
/*******************************************************************************************/
void TileStatus()
{
    if(BlackTile){
        toutatu_zahyou[x][y] += 100;
    }

    switch (Direction)
    {
    case North:
        if(BlackTile || Gap){//黒タイルまたはずれの信号が送られていた場合戻る
            y += 1;
        }
        if(Slope){//坂の信号が送られていた場合座標を更に進める
            y += -1;
        }
        break;

    case East:
        if(BlackTile || Gap){//黒タイルの信号が送られていた場合戻る
            x += -1;
        }
        if(Slope){//坂の信号が送られていた場合座標を更に進める
            x += 1;
        }
        break;
    
    case West:
        if(BlackTile || Gap){//黒タイルの信号が送られていた場合戻る
            x += 1;
        }
        if(Slope){//坂の信号が送られていた場合座標を更に進める
            x += -1;
        }
        break;

    case South:
        if(BlackTile || Gap){//黒タイルの信号が送られていた場合戻る
            y += -1;
        }
        if(Slope){//坂の信号が送られていた場合座標を更に進める
            y += 1;
        }
        break;
    
    
    }
    BlackTile = false;
    Slope = false;
    Gap = false;
}


  


/*******************************************************************************************/
/* 最短経路の方位                                                                              
/*処理：現在のマスのコストの、-1のコストのマスを探す
/*更新者：吉ノ薗2025/01/22
/*       吉ノ薗2025/03/28　差が１でも壁があれば行かないようにした
/*
/*******************************************************************************************/
int WhichWay(uint8_t a,uint8_t b)
{
    if ((cost[a][b] - cost[a + 1][b] == 1) && !(kabe_zahyou[a][b] & 8)) {
        return East;
    }
    if ((cost[a][b] - cost[a][b - 1] == 1) && !(kabe_zahyou[a][b] & 1)) {
        return North;
    }
    if ((cost[a][b] - cost[a - 1][b] == 1) && !(kabe_zahyou[a][b] & 4)) {
        return West;
    }
    if ((cost[a][b] - cost[a][b + 1] == 1) && !(kabe_zahyou[a][b] & 2)) {
        return South;
    }
    Serial.println("Error");
    return 0;
}



/*******************************************************************************************/
/* 幅優先探索                                                                              
/*処理：現在のマスから空いている隣接マスのコストを、現在のマスのコスト+1していくのを繰り返す
/*      kabe_zahyou[][]に0000 の4ビットに絶対方向の東8西4南2北1をそれぞれ割り当てているので、割っていって余り0のときその先の壁が無いと判定する
/*      ゴールにたどり着いたらゴールから逆に辿っていきつつ右折、左折、直進を逆からスタックに入れていく
/*
/*
/*
/*更新者：吉ノ薗2025/01/22
/*　　　　吉ノ薗2025/02/01　変更点：スタックとキューをライブラリにして座標の計算をビット演算に変更
/*       吉ノ薗2025/03/26 delay(300)を削除。遅かったのお前が原因だろ
/*       吉ノ薗2025/03/28 座標記入漏れ・ミスを修正
/*       吉ノ薗2025/03/28 逆探索終了時の向きと実際の向きを合わせるように修正
/*
/*******************************************************************************************/
void BFS()
{
    //hidari();//デバッグ用
    uint8_t a = x;
    uint8_t b = y;
    cost[a][b] = 1;//現在地のコストを1にする
    Serial.println("GotoHome:");

    while(!(a == GoalX && b == GoalY)){//ここ詰まったら中断するようにしたかった

        reach_time[a][b] = 1;//そのマスを訪問済みにする

        Serial.print("a =");
        Serial.println(a);
        Serial.print("b =");
        Serial.println(b);

        Serial.print("kabe_zahyou[a][b] ==");
        Serial.println(kabe_zahyou[a][b]);

        Serial.print("cost[a][b] == ");
        Serial.println(cost[a][b]);
        
        for(int n = 1; n <= 8; n *= 2){//そのマスの周りのマスのコストを＋１する
            //int result = static_cast<int>(pow(2, n));
            Serial.print("n =");
            Serial.print(n);

            if(!(kabe_zahyou[a][b] & n)) {//kabe_zahyou[][]は0000 の4ビットに絶対方向の東8西4南2北1をそれぞれ割り当てる

                switch(n){
                    case 1://北の壁がない
                        if((!reach_time[a][b-1]) && !(kabe_zahyou[a][b-1] & 16)){//その先のマスが訪問済みでない&その先のマスが探索済み
                            cost[a][b-1] = cost[a][b] + 1;
                            //キューの末尾に入れる
                            Q.push(a);
                            Q.push(b-1);
                        }
                        break;

                    case 2://南の壁がない
                        if((!reach_time[a][b+1]) && !(kabe_zahyou[a][b+1] & 16)){//その先のマスが訪問済みでない&その先のマスが探索済み
                            cost[a][b+1] = cost[a][b] + 1;
                            //キューの末尾に入れる
                            Q.push(a);
                            Q.push(b+1);
                        }
                        break;

                    case 4://西の壁がない
                        if((!reach_time[a-1][b]) && !(kabe_zahyou[a-1][b] & 16)){//その先のマスが訪問済みでない&その先のマスが探索済み
                            cost[a-1][b] = cost[a][b] + 1;
                            //キューの末尾に入れる
                            Q.push(a-1);
                            Q.push(b);
                        }
                        break;

                    case 8://東の壁がない
                        if((!reach_time[a+1][b]) && !(kabe_zahyou[a+1][b] & 16)) {//その先のマスが訪問済みでない&その先のマスが探索済み
                            cost[a+1][b] = cost[a][b] + 1;
                            //キューの末尾に入れる
                            Q.push(a+1);
                            Q.push(b);
                        }
                        break;
                }
            }
        }
        //キューの先頭を取り出す
        if (Q.size() < 2) break;  // キューの要素が足りない場合は終了

        a = Q.front(); Q.pop();
        b = Q.front(); Q.pop();
    }


    //スタックを使って逆探索
    a = GoalX;
    b = GoalY;
    NowDirection = Direction;
    Direction = North;

    S.push(4);//停止用


    while(!((a == x) && (b == y))){
        switch(Direction){
            case East:
                switch(WhichWay(a,b)){//前後左右のどこが最短になるか

                    case North://北マスからきたとき（ここのシグナルは探索時の曲がる→進むとは逆で、進む→曲がるじゃないと。）
                        S.push(2);//１：右折、２：左折、３：直進
                        S.push(3);
                        b += -1;
                        Direction = South;
                        break;
                    case West://西マスからきたとき
                        S.push(3);
                        a += -1;
                        break;

                    case South://南マスからきたとき
                        S.push(1);
                        S.push(3);
                        b += 1;
                        Direction = North;
                        break;
                    case East:
                        Direction =North;
                        break;

                }
                break;

            case North:
                switch(WhichWay(a,b)){//前後左右のどこが最短になるか
                    case East:
                        S.push(1);
                        S.push(3);
                        a += 1;
                        Direction = West;
                        break;

                    case West:
                        S.push(2);
                        S.push(3);
                        a += -1;
                        Direction = East;
                        break;

                    case South:
                        S.push(3);
                        b += 1;
                        break;

                    case North:
                        S.push(2);
                        S.push(2);
                        S.push(3);
                        b += -1;
                        Direction = South;
                        //Direction = East;
                        break;

                }
                break;

            case West:
                switch(WhichWay(a,b)){//前後左右のどこが最短になるか
                    case East:
                        S.push(3);
                        a += 1;
                        break;

                    case North:
                        S.push(1);
                        S.push(3);
                        b += -1;
                        Direction = South;
                        break;

                    case South:
                        S.push(2);
                        S.push(3);
                        b += 1;//ここが原因
                        Direction = North;
                        break;

                    case West:
                        Direction =North;
                        break;

                }
                break;

            case South:
                switch(WhichWay(a,b)){//前後左右のどこが最短になるか
                    case East:
                        S.push(2);
                        S.push(3);
                        a += 1;
                        Direction = West;
                        break;

                    case North:
                        S.push(3);
                        b += -1;
                        break;

                    case West:
                        S.push(1);
                        S.push(3);
                        a += -1;
                        Direction = East;
                        break;

                    case South:
                        Direction = North;
                        break;

                }
                break;

            /*デバッグ用*/
            default:
                Serial.println("Error");
                break;
        }
    }

    switch (NowDirection)//今の向きを逆探索終了時の向きに変更する
    {
    case North:
        switch (Direction)
        {
        case East:
            S.push(1);
            break;

        case South:
            S.push(1);
            S.push(1);
            break;

        case West:
            S.push(2);
            break;
        }
        break;
    
    case East:
        switch (Direction)
        {
        case North:
            S.push(2);
            break;

        case South:
            S.push(1);
            break;

        case West:
            S.push(1);
            S.push(1);
            break;
        }
        break;

    case South:
        switch (Direction)
        {
        case North:
            S.push(1);
            S.push(1);
            break;
        
        case East:
            S.push(2);
            break;

        case West:
            S.push(1);
            break;
        }
        break;

    case West:
        switch (Direction)
        {
        case North:
            S.push(1);
            break;
        
        case East:
            S.push(1);
            S.push(1);
            break;

        case South:
            S.push(2);
            break;
        }
        break;
    }
}



/*******************************************************************************************/
/* 現在のマスの壁情報を記入                                                                              
/*処理：kabe_zahyou[][]は情報未記入の場合100にしてあるので、100とき記録する
/*    kabe_zahyou[][]に0000 の4ビットに絶対方向の東8西4南2北1をそれぞれ割り当てて、値を加算
/*    kabe_zahyou[][]を-100して記録済みに
/*
/*更新者：吉ノ薗2025/01/22
/*
/*******************************************************************************************/
void WriteDownWall(uint8_t x, uint8_t y,uint8_t Direction)
{
    //壁情報の記入(ここは帰還アルゴリズム用の関数)
    if(kabe_zahyou[x][y] == 16){//記録されていない場合（そうしないと延々と加算されちゃう）
        kabe_zahyou[x][y] = 0;//4ビットの情報のみが残る
        switch (Direction){
            case East:
                if(right_wall){
                    kabe_zahyou[x][y] += 2;
                }
                if(front_wall){
                    kabe_zahyou[x][y] += 8;
                }
                if(left_wall){
                    kabe_zahyou[x][y] += 1;
                }
                break;
            
            case North:
                if(right_wall){
                    kabe_zahyou[x][y] += 8;
                }
                if(front_wall){
                    kabe_zahyou[x][y] += 1;
                }
                if(left_wall){
                    kabe_zahyou[x][y] += 4;
                }
                break;

            case West:
                if(right_wall){
                    kabe_zahyou[x][y] += 1;
                }
                if(front_wall){
                    kabe_zahyou[x][y] += 4;
                }
                if(left_wall){
                    kabe_zahyou[x][y] += 2;
                }
                break;

            case South:
                if(right_wall){
                    kabe_zahyou[x][y] += 4;
                }
                if(front_wall){
                    kabe_zahyou[x][y] += 2;
                }
                if(left_wall){
                    kabe_zahyou[x][y] += 8;
                }
                break;
            }
            //対応
            //東西南北
            //8 4 2 1
        
    }
}

/*******************************************************************************************/
/* BFSで使う左手法                                                                            
/*処理：今いるマスの壁情報をBFSWallZahyouに記入（スタート時の南方向は今回の場合わからないので記入しない）
/*　　　ゴールマスの壁情報のうち、現在の向きの後ろ側になる壁の情報をなくす
/*    　スタート時のマスの壁情報と照合
/*
/*更新者：吉ノ薗2025/03/28
/*
/*******************************************************************************************/
void ForBFSLeftGo(){
    //WriteDownWall
    int BFSWallZahyou = 0;
    //壁情報の記入(ここは帰還アルゴリズム用の関数)
    switch (Direction){
    case East:
      if(right_wall){
        //BFSWallZahyou |= 2;
      }
      if(front_wall){
        BFSWallZahyou += 8;
      }
      if(left_wall){
        BFSWallZahyou += 1;
      }
      break;
            
    case North:
      if(right_wall){
        BFSWallZahyou += 8;
      }
      if(front_wall){
        BFSWallZahyou += 1;
      }
      if(left_wall){
        BFSWallZahyou += 4;
      }
      break;

    case West:
      if(right_wall){
        BFSWallZahyou += 1;
      }
      if(front_wall){
        BFSWallZahyou += 4;
      }
      if(left_wall){
        //BFSWallZahyou |= 2;
      }
      break;

    case South:
      if(right_wall){
        BFSWallZahyou += 4;
      }
      if(front_wall){
        //BFSWallZahyou |= 2;
      }
      if(left_wall){
        BFSWallZahyou += 8;
      }
      break;
    }
    //南の壁情報をなくす
    BFSWallZahyou &= ~2;

    //ゴールマスの壁情報のうち、現在の向きの後ろ側になる壁の情報をなくす
    switch (Direction)
    {
    case North:
      kabe_zahyou[GoalX][GoalY] &= ~2;
      break;
    
    case East:
      kabe_zahyou[GoalX][GoalY] &= ~4;
      break;

    case South:
      kabe_zahyou[GoalX][GoalY] &= ~1;
      break;

    case West:
      kabe_zahyou[GoalX][GoalY] &= ~8;
      break;
    }
    if(BFSWallZahyou == kabe_zahyou[GoalX][GoalY]){////スタート時のマスの壁情報と合ってる場合停止フラグを立てる
      StopFlag = true;
      return;
    }
    
    //左手法
    uint8_t GoTo = 0;
    if     (!left_wall) {GoTo = Left ;}
    else if(!front_wall){GoTo = Front;}
    else if(!right_wall){GoTo = Right;}
    else                {GoTo = Back ;}

    switch (GoTo)
    {
    case Right:
      Status = 4;//右折
      break;

    case Front:
      Status = 2;//直進
      break;

    case Left:
      Status = 3;//左折
      break;

    case Back:
      Status = 5;//後進
      break;
      }
      
}

/*******************************************************************************************/
/* 帰還                                                                             
/*処理：スタックから値をpopして順番に動いていく
/*      全て終わる、つまりpopした値が一番最初に入れた"4"になっていたら停止（まだ停止部分はつくってない）
/*
/*更新者：吉ノ薗2025/01/22
/*　　　　吉ノ薗2025/01/29：20秒停止するようにした
/*       吉ノ薗2025/03/26 whileの条件を「スタックが空でなかった場合」に変更
/*       吉ノ薗2025/03/28 座標がずれても戻れるように南以外の壁座標が間違っていた場合（スタート時Northで南がわからないため）左手法を走行するようにした
/*
/*******************************************************************************************/
void GoHome()
{
    Direction = NowDirection;
    while(!S.empty()){
        //send_display();
        switch(S.top()){
            case 1:
                //TurnRight
                migi();
                switch (Direction)
                {
                case North:
                  Direction = East;
                  break;
                
                case East:
                  Direction = South;
                  break;

                case South:
                  Direction = West;
                  break;

                case West:
                  Direction = North;
                  break;
                }
                delay(500);
                break;

            case 2:
                //TurnLeft
                hidari();
                switch (Direction)
                {
                case North:
                  Direction = West;
                  break;
                
                case East:
                  Direction = North;
                  break;

                case South:
                  Direction = East;
                  break;

                case West:
                  Direction = South;
                  break;
                }
                delay(500);
                break;

            case 3:
                //GoStraight
                susumu_heitan();
                switch (Direction)
                {
                case North:
                  y += -1;
                  break;
                
                case East:
                  x += 1;
                  break;

                case South:
                  y += 1;
                  break;

                case West:
                  x += -1;
                  break;
                }
                delay(200);
                break;
            case 4:
                //Stop
                NeoPixel_Color(0,0,255);//うろうろさせない場合このコードで停止
                delay(20000);

                /*壁情報取得してうろうろさせる*/
                while(!StopFlag)
                {
                  switch (Status)
                  {
                  case 0:
                    get_tof_data();
                    break;
                  
                  case 1:
                    //WriteDownWall
                    ForBFSLeftGo();
                    break;

                  case 2://直進
                    susumu_heitan();
                    switch (Direction)
                    {
                    case North:
                      y += -1;
                      break;
                    
                    case East:
                      x += 1;
                      break;
    
                    case South:
                      y += 1;
                      break;
    
                    case West:
                      x += -1;
                      break;
                    }
                    delay(200);
                    TileStatus();
                    break;
                  
                  case 3://左折
                    hidari();
                    switch (Direction)
                    {
                    case North:
                      Direction = West;
                      break;
                    
                    case East:
                      Direction = North;
                      break;
    
                    case South:
                      Direction = East;
                      break;
    
                    case West:
                      Direction = South;
                      break;
                    }
                    delay(500);
                    susumu_heitan();
                    switch (Direction)
                    {
                    case North:
                      y += -1;
                      break;
                    
                    case East:
                      x += 1;
                      break;
    
                    case South:
                      y += 1;//
                      break;
    
                    case West:
                      x += -1;
                      break;
                    }
                    delay(200);
                    TileStatus();
                    break;
                  
                  case 4://右折
                    migi();
                    switch (Direction)
                    {
                    case North:
                      Direction = East;
                      break;
                    
                    case East:
                      Direction = South;
                      break;
    
                    case South:
                      Direction = West;
                      break;
    
                    case West:
                      Direction = North;
                      break;
                    }
                    delay(500);
                    susumu_heitan();
                    switch (Direction)
                    {
                    case North:
                      y += -1;
                      break;
                    
                    case East:
                      x += 1;
                      break;
    
                    case South:
                      y += 1;
                      break;
    
                    case West:
                      x += -1;
                      break;
                    }
                    delay(200);
                    TileStatus();
                    break;
                  
                  case 5://後進
                    hidari();
                    delay(500);
                    hidari();
                    switch (Direction)
                    {
                    case North:
                      Direction = South;
                      break;
                    
                    case East:
                      Direction = West;
                      break;
    
                    case South:
                      Direction = North;
                      break;
    
                    case West:
                      Direction = East;
                      break;
                    }
                    delay(500);
                    susumu_heitan();
                    switch (Direction)
                    {
                    case North:
                      y += -1;
                      break;
                    
                    case East:
                      x += 1;
                      break;
    
                    case South:
                      y += 1;
                      break;
    
                    case West:
                      x += -1;
                      break;
                    }
                    delay(200);
                    TileStatus();
                    break;
                  }

                }
                NeoPixel_Color(0,0,255);    
                delay(20000);               
                break;
        }
        S.pop();//要素の削除
    }
    
}



/*******************************************************************************************/
/* セットアップ                                                                              
/*処理：MPU、センサー、モーター、座標の初期化
/*変更点：センサーのセットアップの部分を最初に持ってくる/Serialの初期化のための適切な待機/サーボモーターの速度設定を2倍にする
/*更新者：吉ノ薗陽向　2025/01/22
/*       清田侑希　2025/01/25
/*　　　　吉ノ薗陽向　2025/01/31　変更点：初期位置の到達回数を+1した
/*
/*******************************************************************************************/
void setup(){
/*センサーのセットアップ***********************************************************************************************************/
  Serial.begin(9600);   // デバッグ用
  Serial1.begin(19200); //using_color_load
  Serial2.begin(1000000);//using_servo_sts3032
  Serial3.begin(9600);  // using_tof
  Serial7.begin(19200);//using_cam1
  Serial8.begin(19200);//using_cam2
  Wire.begin();
  pinMode(6, INPUT); //スタート用スイッチのpin設定
  pinMode(SERVO_PIN, OUTPUT);//レスキューキット用のサーボモーターのpin設定
  pinMode(SERVO_PIN2, OUTPUT);//上に同じ
  delay(2000);
  /*MPUのセットアップ***********************************************************************************************************/
  Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
  mpu.initialize();
  devStatus = mpu.dmpInitialize();
  // supply your own gyro offsets here, scaled for min sensitivity

  mpu.setXAccelOffset(-3550);
  mpu.setYAccelOffset(-1727);
  mpu.setZAccelOffset(1087);
  mpu.setXGyroOffset(25);
  mpu.setYGyroOffset(-29);
  mpu.setZGyroOffset(7);
  
  // make sure it worked (returns 0 if so)
  if (devStatus == 0) {
    // Calibration Time: generate offsets and calibrate our MPU6050
    mpu.CalibrateAccel(6);
    mpu.CalibrateGyro(6);
    mpu.setDMPEnabled(true);
    packetSize = mpu.dmpGetFIFOPacketSize();
  } else {
    Serial.print("DMP Initialization failed.");
  }
  /*モーターのセットアップ***********************************************************************************************************/
 softSerial.begin( 9600 ); //softSerial通信の速さを9600bit/秒に設定
 sms_sts.pSerial = &Serial2;
 delay(1000);
  ID[0] = 1; //右前のモーターのID  
  ID[1] = 2; //左前
  ID[2] = 3; //右後
  ID[3] = 4; //左後

  Speed[0] = 6800; //変数Speed[0]=0とする
  Speed[1] = 6800;
  Speed[2] = 6800;
  Speed[3] = 6800;

  ACC[0] = 50; //変数ACC[0]=0とする
  ACC[1] = 50;
  ACC[2] = 50;
  ACC[3] = 50;
  Serial.println("test");

    /*...........NeoPixelのセットアップ.......*/
    pixels.begin(); // NeoPixelの初期化
    pixels.setBrightness(BRIGHTNESS); // 明るさを設定


  /*スタック・キューのセットアップ***********************************************************************************************************
  init();
  st_init();
*/


  /*アルゴリズムのセットアップ***********************************************************************************************************/
  for (int t = 0; t < 90; t++) {
    for (int j = 0; j < 90; j++) {
        kabe_zahyou[t][j] = 16;
        reach_time[t][j] = 0;
        cost[t][j] = 0;
        toutatu_zahyou[t][j] = 0;
        VictimisAlready[t][j] = false;
        }
    }
    toutatu_zahyou[GoalX][GoalY] = 1;
    kabe_zahyou[GoalX][GoalY] = 0;
    // 現在の時刻を取得します
    time_t FirstTime;
    time(&FirstTime);  // 現在の時刻を取得して FirstTime に格納
    firstseconds = static_cast<long>(FirstTime); 
    
}

/*******************************************************************************************/
/* メインループ                                                                              
/*処理：スタートスイッチがONのときメインの行動を行う
/*      statusの値によって壁情報の取得・座標更新と探索・帰還・前後左右への移動を行う
/*      status1のとき壁情報を記録 → 拡張右手法て行く方向を決定 → 現在の時刻とスタートの時刻を照合して、330秒経っていたらstatusを2にするかどうか決定
/*      status2のとき幅優先探索 → 実際に動いて停止
/*      status3,4,5,6のとき前後左右に移動 → 黒タイルや坂だった場合に合わせて座標を更新
/*
/*更新者：吉ノ薗2025/01/22
/*　　　　清田侑希 2025/1/28　変更点：動きの最適化のためにdelayを追加|スタートスイッチの導入
/*　　　　吉ノ薗 2025/01/31　変更点：途中で進行停止になった場合座標をチェックポイントの場所に戻すようにした。
/*　　　　吉ノ薗 2025/02/01　変更点：チェックポイントの方位を追加
/*        清田侑希 2025/3/10 変更点：スタート時にMPUの初期化を行うようにした
/*******************************************************************************************/
void loop(){
    /*.............スタートスイッチがオンになるまで待機する部分...........*/
    while (digitalRead(6) == LOW) {
        delay(10);
        Serial.println("Waiting...");
        if (digitalRead(6) == HIGH){
             //主に進行停止の時に値がずれるので再初期化する
            Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
            mpu.initialize();
            devStatus = mpu.dmpInitialize();
            // supply your own gyro offsets here, scaled for min sensitivity

            mpu.setXAccelOffset(-3550);
            mpu.setYAccelOffset(-1727);
            mpu.setZAccelOffset(1087);
            mpu.setXGyroOffset(25);
            mpu.setYGyroOffset(-29);
            mpu.setZGyroOffset(7);

            // make sure it worked (returns 0 if so)
            if (devStatus == 0) {
                // Calibration Time: generate offsets and calibrate our MPU6050
                mpu.CalibrateAccel(6);
                mpu.CalibrateGyro(6);
                mpu.setDMPEnabled(true);
                packetSize = mpu.dmpGetFIFOPacketSize();
            } else {
                Serial.print("DMP Initialization failed.");
            }
            while (Serial3.available() > 0) {
            char receivedChar = Serial3.read(); // データを読み取る
            // 必要に応じて受信データを処理する
            }
            //チェックポイントとして保存した座標を現在の座標として代入する
            x = CheckX;
            y = CheckY;
            Direction = CheckD;
            Status = 0;
            break;
        }
    }
    /*.................................................................*/
    
    
    switch (Status)
    {
    case 0://壁情報取得

        get_tof_data();
        break;

    case 1://座標更新と探索
        WriteDownWall(x,y,Direction);//帰還用の記録
        //send_display();/*デバッグ用*/

        /*デバッグ用*/
        /*if(BFScount){BFS(x,y);}
        else{BFScount = true;}*/

        /*デバッグ用
        Homecount += 1;
        if(Homecount >= 10){
            Status = 2;//帰還開始
            start_Gohome = true;
        }*/
       // 現在の時刻を取得
        time_t NowTime;
        time(&NowTime);  // 現在の時刻を取得して NowTime に格納
        now_seconds = static_cast<long>(NowTime); 

        //330秒（＝5分半）経ったら幅優先探索を始める
        if(now_seconds - firstseconds >= 360){
            Status = 2;//帰還開始
            start_Gohome = true;
        }
        
        if(!(Status == 2)){MoveTo(judge());};//拡張右手法で行く方法を決める,実際に移動して座標を変更+到達回数を加算

        break;
    
    case 2://帰還
        //NeoPixel_Color(0,0,255);   
        BFS();
        //pixels.clear();
        //pixels.show();
        GoHome();
        break;

    case 3://直進
        susumu_heitan();
        delay(200);
        TileStatus();
        break;

    case 4://右折
        migi();
        delay(500);
        susumu_heitan();
        delay(200);
        TileStatus();
        break;

    case 5://左折
        hidari();
        delay(500);
        susumu_heitan();
        delay(200);
        TileStatus();
        break;

    case 6://後進
        hidari();
        delay(500);
        hidari();
        delay(500);
        susumu_heitan();
        delay(200);
        TileStatus();
        break;
    }
    
}
