/*
==========READ ME==========
変数や関数を宣言定義するときは必ず説明を書いてください。
それらを変更するときは変更履歴を名前フルネームと入学年度付きで付してください。(但し、あまりにも頻繁には不要。)ー2026/01/25 阿部実裕ー令和六年度入学
関数名や変数名には極力英語を用いてください。
全角スペースは用いないでください。
日付はYYYY/MM/DDを用い、0は省略しないでください。ー2026/01/26 阿部実裕ー令和六年度入学

==========掲示板==========
「~~~」は阿部実裕ー令和六年度入学が用いるチェックポイントなので消さないでください。使用後自分で消します。ー2026/01/26 阿部実裕ー令和六年度入学

*/

#if 0
//↓旧ReadME↓
/*関数の編集時には記録を残してください

このコードの見方
関数の最初にその関数の簡易的な説明が書いてあります
また編集者の名前と日付も書いてあります
検索でキーワードや編集者の名前を入れると見たい関数が探しやすいです*/
#endif

//==========以下プログラム==========

#define EVT_READY 7
#define DEBUG 1//on->1 off->0
float test = 0;

#include <stdio.h>
#include <cstdint>
#include <Arduino.h>
#include <time.h>
#include <cstdint>
#include <SCServo.h> //SCServoを使うと宣言]
#include <SoftwareSerial.h> //SoftwareSerial(複数機器とシリアル通信)を使うと宣言
#include <Adafruit_NeoPixel.h>
#include <queue>
#include <stack>
#include "MyBasicDrive.h"
#include "MyUARTs_read.h"
#include "MyYPRs.h"

#define DEBUGMODE false; //Trueでデバックモード,随時情報をSerial.printするようになる。

/** 超信地旋回のデフォルト速度
 * @note spin() の第二引数たる速度のデフォルト値
 * @note 推奨 : 440 */
const int DEFAULT_SPIN_SPEED = 440;

MyUARTs_read colorUART,tofUART,cam7,cam8;
byte tof_buf[6];

MyYPRs MPU;   // ← これが mpu + dmp + ypr の代役

MyBasicDrive drive;

std::stack<uint8_t> S;
std::queue<uint8_t> Q;
SMS_STS sms_sts;
SoftwareSerial SoftwareSerial(36,37);

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
float ypr[3];


int deb = 0;

int isItWall = 225;//@@@tyousei@@@

/*******************************************************************************************/
/* Yaw・Pitch・Roll角の取得                                                                            
/*処理：MPU6050を用いて回転や坂検知のためのRoll・Yaw角の取得
/*
/*更新者：清田侑希　2025/1/25
/*更新者:阿部実裕 202/01/25 デバックモードの実装
/*******************************************************************************************/
void getYawPitchRoll() {
  #if DEBUGMODE
    Serial.println(".----getYawPitchRoll() Started----.")
  #endif
  MPU.update();   // ★ 毎回必ず呼ぶ

  yaw   = MPU.getYaw360();    // 0〜360
  pitch = MPU.getPitch();  // deg
  roll  = MPU.getRoll();   // deg
  #if DEBUGMODE
    Serial.print("Yaw : ");
    Serial.print(yaw);
    Serial.print(" Pitch: ");
    Serial.print(pitch);
    Serial.print(" Roll: ");
    Serial.print(roll);
    Serial.println(" ----by getYawPitchRoll()")
    Serial.println("`----getYawPitchRoll() Finished----`")
  #endif
}

/*.........角度補正用の変数の定義.........*/
float body_len = 115; //機体の前のセンサーと後ろのセンサーの距離//@@@tyousei@@@
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
float run_len_time = 1; //30㎝の何倍か//@@@tyousei@@@
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
int servo1_kit = 5; //レスキューキットの残数//@@@tyousei@@@
int servo2_kit = 5; //上に同じ//@@@
/*..........................................................*/
/*センサー類変数*/
unsigned int SERVO_POS = 0; //変数SERVO_POS=0とする
byte ID[4]; //IDはそれぞれのモーターのID
s16 Position[4]; //Positionはモーターが回る角度(右一回転＝4095)
u16 Speed[4]; 
byte ACC[4]; 
int susumu_kaisuu= 40;//@@@tyousei@@@
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
/*更新者:阿部実裕 202/01/25 デバックモードの実装
/*******************************************************************************************/
void tiziki_kaitenn()
{
  #if DEBUGMODE
    Serial.println(".----tiziki_kaitenn() Started----.")
  #endif

  getYawPitchRoll();
  kakudo2 = yaw;
  kakudo_true2 = int(kakudo2);

  #if DEBUGMODE
    Serial.print("kakudo-true2: ");
    Serial.print(kakudo_true2);
    Serial.println(" ----by tiziki_kaitenn()")
    Serial.println("`----tiziki_kaitenn() Finished----`")
  #endif
}

#if 0
/*******************************************************************************************/
/* 地磁気                                                                              
/*処理：回転後に発生する機体の方向の誤差をMPU6050によって補正
/* 変更点：モーターにギアを導入したのでモーターの回転方向が逆になってます
/*更新者：清田侑希 2025/1/25
/*清田侑希 2025/3/27 変更点：基準となる角度を定数から変数に変更
/*福田峻太ー令和七年度入学 2025/12/22　変更点：ライブラリ化①
/*阿部実裕ー令和六年度入学 2026/01/25 変更点:無効化
/*******************************************************************************************/
void tiziki()
{
  getYawPitchRoll();
  kakudo = yaw;
  kakudo_true = int(kakudo);

  Serial.print("kakudo-true @tiziki");
  Serial.println(kakudo_true);

  if (kakudo_true >= 0 && kakudo_true < 45) {
    for (int i = 0; i < kakudo_true - mawasu; i++) {
      drive.turnRight(60);//@@@tyousei@@@
      delay(18);
    }

  } else if (kakudo_true >= 45 && kakudo_true < 90) {
    for (int i = 0; i < mawasu - kakudo_true; i++) {
      drive.turnLeft(60);//@@@
      delay(18);
    }

  } else if (kakudo_true >= 90 && kakudo_true < 135) {
    for (int i = 0; i < kakudo_true - mawasu; i++) {
      drive.turnRight(60);//@@@
      delay(18);
    }

  } else if (kakudo_true >= 135 && kakudo_true < 180) {
    for (int i = 0; i < mawasu - kakudo_true; i++) {
      drive.turnLeft(60);//@@@
      delay(18);
    }

  } else if (kakudo_true >= 180 && kakudo_true < 225) {
    for (int i = 0; i < kakudo_true - mawasu; i++) {
      drive.turnRight(60);//@@@
      delay(18);
    }

  } else if (kakudo_true >= 225 && kakudo_true < 270) {
    for (int i = 0; i < mawasu - kakudo_true; i++) {
      drive.turnLeft(60);//@@@
      delay(18);
    }

  } else if (kakudo_true >= 270 && kakudo_true < 315) {
    for (int i = 0; i < kakudo_true - mawasu; i++) {
      drive.turnRight(60);//@@@
      delay(18);
    }

  } else if (kakudo_true >= 315 && kakudo_true <= 360) {
    for (int i = 0; i < mawasu - kakudo_true; i++) {
      drive.turnLeft(60);//@@@
      delay(18);
    }
  }

  drive.stop();   // ★ 安全のため追加（重要）

  Serial.println("kakudo_hosei");
  delay(100);

  MPU.resetYaw();
}
#endif



/*******************************************************************************************/
/* 地磁気２                                                                              
/*処理：坂の検知に用いるRoll角の整数化
/*変更点：MPU6050の設置方向の関係からPitch角からRoll角に仕様変更
/*更新者：清田侑希 2025/1/25
/*更新者:阿部実裕ー令和六年度入学　2026/01/25 変更点:デバックモードの実装
/*******************************************************************************************/
void tiziki_2(){
  #if DEBUGMODE
    Serial.println(".----tiziki_2() Started----.")
  #endif
  getYawPitchRoll();
  katamuki = roll;
  katamuki_true = int(katamuki);
  #if DEBUGMODE
    Serial.print("katamuki_true: ");
    Serial.println(katamuki_true);
    Serial.println(" ----by tiziki_2()")
    Serial.println("`----tiziki_2() Finished----`") 
  #endif
}


/*******************************************************************************************/
/* NeoPixelのいろいろな動作関数                                                                            
/*処理：Neopixelを前後左右、赤黄緑に光らせる
/*更新者：清田侑希 2025/3/25
/*更新者:阿部実裕 202/01/25 変更点:デバックモードの実装
/*******************************************************************************************/

void NeoPixel_Front_ON() {
  #if DEBUGMODE
    Serial.println(".----NeoPixel_Front_ON() Started----.");
  #endif
  pixels.setPixelColor(0, pixels.Color(255, 0, 0)); // 0番目のLEDを(R,G,B)=(255,0,0)⇔赤に点灯
  pixels.setPixelColor(1, pixels.Color(255, 0, 0)); // 1番目のLEDを(R,G,B)=(255,0,0)⇔赤に点灯
  pixels.show(); // LEDに反映
  #if DEBUGMODE
    Serial.println("`----NeoPixel_Front_ON() Finished----`");
  #endif
}
void NeoPixel_Front_OFF() {
  #if DEBUGMODE
    Serial.println(".----NeoPixel_Front_OFF() Started----.");
  #endif
  pixels.setPixelColor(0, pixels.Color(0, 0, 0)); // 0番目のLEDを消灯
  pixels.setPixelColor(1, pixels.Color(0, 0, 0)); // 1番目のLEDを消灯
  pixels.show(); // LEDに反映
  #if DEBUGMODE
    Serial.println("`----NeoPixel_Front_OFF() Finished----`");
  #endif
}
void NeoPixel_Right_ON() {
  #if DEBUGMODE
    Serial.println(".----NeoPixel_Right_ON() Started----.");
  #endif 
  pixels.setPixelColor(2, pixels.Color(255, 0, 0)); // 2番目のLEDを(R,G,B)=(255,0,0)⇔赤に点灯
  pixels.setPixelColor(3, pixels.Color(255, 0, 0)); // 3番目のLEDを(R,G,B)=(255,0,0)⇔赤に点灯
  pixels.show(); // LEDに反映
  #if DEBUGMODE
    Serial.println("`----NeoPixel_Right_ON() Finished----`");
  #endif
}
void NeoPixel_Right_OFF() {
  #if DEBUGMODE
    Serial.pirntln(".----NeoPixel_Right_OFF() Started----.");
  #endif
  pixels.setPixelColor(2, pixels.Color(0, 0, 0)); // 2番目のLEDを消灯
  pixels.setPixelColor(3, pixels.Color(0, 0, 0)); // 3番目のLEDを消灯
  pixels.show(); // LEDに反映
  #if DEBUGMODE
    Serial.println("`----NeoPixel_Right_OFF() Finished----`");
  #endif
}
void NeoPixel_Rear_ON() {
  #if DEBUGMODE
    Serial.println(".----NeoPixel_Rear_ON() Started----.");
  #endif
    pixels.setPixelColor(4, pixels.Color(255, 0, 0)); // 4番目のLEDを(R,G,B)=(255,0,0)⇔赤に点灯
  pixels.setPixelColor(5, pixels.Color(255, 0, 0)); // 5番目のLEDを(R,G,B)=(255,0,0)⇔赤に点灯
  pixels.show(); // LEDに反映
  #if DEBUGMODE
    Serial.println("`----NeoPixel_Rear_ON() Finished----`");
  #endif
}
void NeoPixel_Rear_OFF() {
  #if DEBUGMODE
    Serial.println(".----NeoPixel_Rear_OFF() Started----.");
  #endif
  pixels.setPixelColor(4, pixels.Color(0, 0, 0)); // 4番目のLEDを消灯
  pixels.setPixelColor(5, pixels.Color(0, 0, 0)); // 5番目のLEDを消灯
  pixels.show(); // LEDに反映
  #if DEBUGMODE
    Serial.println("`----NeoPixel_Rear_OFF() Finished----`");
  #endif
}
void NeoPixel_Left_ON() {
  #if DEBUGMODE
    Serial.println(".----NeoPixel_Left_ON() Started----.");
  #endif
  pixels.setPixelColor(6, pixels.Color(255, 0, 0)); // 6番目のLEDを(R,G,B)=(255,0,0)⇔赤に点灯
  pixels.setPixelColor(7, pixels.Color(255, 0, 0)); // 7番目のLEDを(R,G,B)=(255,0,0)⇔赤に点灯
  pixels.show(); // LEDに反映
  #if DEBUGMODE
    Serial.println("`----NeoPixel_Left_ON() Finished----`");
  #endif
}
void NeoPixel_Left_OFF() {
  #if DEBUGMODE
    Serial.println(".----NeoPixel_Left_OFF() Started----.");
  #endif
  pixels.setPixelColor(6, pixels.Color(0, 0, 0)); // 6番目のLEDを消灯
  pixels.setPixelColor(7, pixels.Color(0, 0, 0)); // 7番目のLEDを消灯
  pixels.show(); // LEDに反映
  #if DEBUGMODE
    Serial.println("`----NeoPixel_Left_OFF() Finished----`");
  #endif
}
void NeoPixel_Color(uint8_t r, uint8_t g, uint8_t b) {
  #if DEBUGMODE
    Serial.println(".----NeoPixel_Color() Started----.");
  #endif
  pixels.clear();
  pixels.fill(pixels.Color(r, g, b));
  pixels.show();
  #if DEBUGMODE
    Serial.println("`----NeoPixel_Color() Finished----`");
  #endif
}


/*******************************************************************************************/
/* collect_tof_data                                                                              
/*処理： ヘッダーが来たら6バイト距離のデータを取得するこれによって各壁の検知を行う
/*
/*更新者：清田侑希　2025/1/26
/*清田侑希　2025/2/23 変更点：壁の有無を各方向の2つのセンサーがともに壁を検知したときのみ壁とみなす
/*清田侑希　2025/3/30 変更点：tofセンサーをの値を生のデータのまま送信する
/*阿部実裕ー令和六年度入学　2026/01/25 変更点：デバックモード実装
/*******************************************************************************************/
#define CMD_TOF 0x01

void get_tof_data() {
  #if DEBUGMODE
    Serial.println(".----get_tof_data() Started----.");
  #endif
  // ===== ToF 要請 + 応答 =====
  bool ok = tofUART.request(CMD_TOF, tof_buf, 200);  //@@@tyousei@@@
  // ↑ timeout は 120〜150ms 推奨（後述）

  if (!ok) {
    front_wall = false;
    left_wall  = false;
    right_wall = false;
    Status = 0;
    #if DEBUGMODE
      Serial.println("!----get_tof_data() Stopped due to TOF timeout----!");
    #endif
    return;
  }

  // ===== データ確定 =====
  for (int i = 0; i < 6; i++) {
    receivedData[i] = tof_buf[i];
  }

  #if DEBUGMODE
    Serial.print("TOF: ");
    for (int i = 0; i < 6; i++) {
      Serial.print(receivedData[i]);
      if (i < 5) Serial.print(", ");
    }
    Serial.println(" ----by get_tof_data()");
  #endif

  // ===== 壁判定 =====
  front_wall = (receivedData[0] <= isItWall && receivedData[3] <= isItWall);
  left_wall  = (receivedData[1] <= isItWall && receivedData[2] <= isItWall);
  right_wall = (receivedData[4] <= isItWall && receivedData[5] <= isItWall);

  Status = 1;
  #if DEBUGMODE
    Serial.println("`----get_tof_data() Finished----`");
  #endif
}

/*******************************************************************************************/
/* Servo_rescue                                                                              
/*処理：サーボモーターによるレスキューキットの排出
/*
/*更新者：清田侑希 2025/3/26
/*更新者：阿部実裕ー令和六年度入学 2026/01/25 デバックモード実装
/*******************************************************************************************/
void motor_servo(int motor_speed, int seconds) {
  #if DEBUGMODE
    Serial.print(".----motor_servo() Started in motor_speed: ");Serial.print(motor_speed);Serial.print(" seconds: ");Serial.println(seconds);Serial.println(" ----.");
  #endif
  int pulse_width = 1500-motor_speed*8;//@@@tyousei@@@
  int starttime = millis();
  while(true) {
    if (millis()-starttime > seconds) break;
      digitalWrite(SERVO_PIN, HIGH);
      delayMicroseconds(pulse_width);
      digitalWrite(SERVO_PIN, LOW);
      delay(20);    
  }
  #if DEBUGMODE
    Serial.println("`----motor_servo() Finished----`");
  #endif
}

void motor_servo2(int motor_speed, int seconds) {
  #if DEBUGMODE
    Serial.println(".----motor_servo2() Started----.");
  #endif
  int pulse_width = 1400-motor_speed*8;//@@@
  int starttime = millis();
  while(true) {
    if (millis()-starttime > seconds) break;
      digitalWrite(SERVO_PIN2, HIGH);
      delayMicroseconds(pulse_width);
      digitalWrite(SERVO_PIN2, LOW);
      delay(20);    
  }
  #if DEBUGMODE
    Serial.println("`----motor_servo2() Finished----`");
  #endif
}

void servo_res(){
  #if DEBUGMODE
    Serial.println(".----servo_res() Started----.");
  #endif
  motor_servo(130,220);//@@@tyousei@@@
  motor_servo(0,1000);//@@@
  motor_servo(-135,210);//@@@
  motor_servo(0,1000);//@@@
  delay(150);
  servo1_kit--;
  #if DEBUGMODE
    Serial.println("`----servo_res() Finished----`");
  #endif
}

void servo_res2(){//@@@
  #if DEBUGMODE
    Serial.println(".----servo_res2() Started----.");
  #endif
  motor_servo2(-120,200);
  motor_servo2(0,1000);
  motor_servo2(130,200);
  motor_servo2(0,1000);
  delay(150);
  servo2_kit--;
  #if DEBUGMODE
    Serial.println("`----servo_res2() Finished----`");
  #endif
}

/*******************************************************************************************/
/* get_victim_data                                                                              
/*処理：OpenMVによって被災者の発見
/*
/*更新者：清田侑希 2025/2/23
/*清田侑希 2025/3/26 変更点：被災者を発見したときにLEDを点灯させる
/* 清田侑希 2025/3/28 変更点：被災者を発見したときにサーボモーターを動かす
/*阿部実裕 2026/01/25 変更点:デバックモード実装
/*******************************************************************************************/
//右側のカメラ
void sel7() {
  #if DEBUGMODE
    Serial.println(".----sel7() Started----.");
  #endif

  // すでに処理済みなら何もしない
  if (VictimisAlready[x][y]){
    #if DEBUGMODE
      Serial.println("!----sel7() Stopped deu to Already Processed----!");
    #endif
    return;
  }

  // データが無ければ何もしない
  if (!cam7.hasEvent()){
    #if DEBUGMODE
      Serial.println("!----sel7() Stopped deu to No Data----!");
    #endif
    return;
  }

  // ===== ヘッダ探索 =====
  byte header = cam7.readEvent();
  if (header != 0xAA) {
    // 同期ずれ → 破棄
    #if DEBUGMODE
      Serial.println("!----sel7() Stopped deu to Out of Sync----!");
    #endif
    return;
  }

  // ===== 残り2バイト待ち =====
  if (Serial7.available() < 2) {
    // フレーム未完成
    #if DEBUGMODE
      Serial.println("!----sel7() Stopped deu to Unfinished Frame----!");
    #endif
    return;
  }

  byte receivedData  = cam7.readEvent();
  byte check = cam7.readEvent();

  // ===== チェックサム検証 =====
  if ((receivedData ^ 0xFF) != check) {
    // 壊れたフレーム
    #if DEBUGMODE
      Serial.println("!----sel7() Stopped deu to Broken Frame----!");
    #endif
    return;
  }

  // ===== ここから先は「正しい通信」 =====
  VictimisAlready[x][y] = true;  
  #if DEBUGMODE
    Serial.println("<----sel7() Proceeded to Correct Communication without any Error---->");
  #endif
  switch (receivedData) {

    /* ======================
       H victim
       ====================== */
    case 1:
      #if DEBUGMODE
        Serial.println("<----H_victim was Located on the Right-hand side by sel7()---->");
      #endif
      for (int i = 0; i < 5; i++) {
        NeoPixel_Color(255, 0, 0);
        delay(650);
        pixels.clear();
        pixels.show();
        delay(650);
      }

      if (servo1_kit >= 2) {
        servo_res(); delay(650);
        servo_res(); delay(650);

      } else if (servo1_kit == 1 && servo2_kit >= 1) {

        servo_res(); delay(500);

        drive.turnRight(1000); delay(1700); drive.stop(); delay(300);//@@@tyousei@@@
        drive.turnRight(1000); delay(1700); drive.stop(); delay(300);//@@@

        servo_res2(); delay(500);

        drive.turnRight(1000); delay(1700); drive.stop(); delay(300);//@@@
        drive.turnRight(1000); delay(1700); drive.stop(); delay(300);//@@@

      } else if (servo2_kit >= 2) {

        drive.turnRight(1000); delay(1700); drive.stop(); delay(300);//@@@
        drive.turnRight(1000); delay(1700); drive.stop(); delay(300);//@@@

        servo_res2(); delay(500);
        servo_res2(); delay(500);

        drive.turnRight(1000); delay(1700); drive.stop(); delay(300);//@@@
        drive.turnRight(1000); delay(1700); drive.stop(); delay(300);//@@@

      } else if (servo1_kit == 1) {

        servo_res(); delay(500);

      } else if (servo2_kit == 1) {

        drive.turnRight(1000); delay(1700); drive.stop(); delay(300);//@@@
        drive.turnRight(1000); delay(1700); drive.stop(); delay(300);//@@@

        servo_res2(); delay(500);

        drive.turnRight(1000); delay(1700); drive.stop(); delay(300);//@@@
        drive.turnRight(1000); delay(1700); drive.stop(); delay(300);//@@@
      }
      break;

    /* ======================
       S victim
       ====================== */
    case 2:
      #if DEBUGMODE
        Serial.println("<----S_victim was Located on the Right-hand side by sel7()---->");
      #endif
      for (int i = 0; i < 5; i++) {
        NeoPixel_Color(255, 200, 0);
        delay(650);
        pixels.clear();
        pixels.show();
        delay(650);
      }

      if (servo1_kit >= 1) {

        servo_res(); delay(500);

      } else if (servo2_kit >= 1) {

        drive.turnRight(1000); delay(1700); drive.stop(); delay(300);//@@@
        drive.turnRight(1000); delay(1700); drive.stop(); delay(300);//@@@

        servo_res2(); delay(500);

        drive.turnRight(1000); delay(1700); drive.stop(); delay(300);//@@@
        drive.turnRight(1000); delay(1700); drive.stop(); delay(300);//@@@
      }
      break;

    /* ======================
       Red victim
       ====================== */
    case 4:
      #if DEBUGMODE
        Serial.println("<----red_victim was Located on the Right-hand side by sel7()---->");
      #endif
      for (int i = 0; i < 3; i++) {
        NeoPixel_Color(255, 0, 0);
        delay(1000);
        pixels.clear();
        pixels.show();
        delay(1000);
      }

      if (servo1_kit >= 2) {

        servo_res(); delay(500);
        servo_res(); delay(500);

      } else if (servo1_kit == 1 && servo2_kit >= 1) {

        servo_res(); delay(500);

        drive.turnRight(1000); delay(1700); drive.stop(); delay(300);//@@@
        drive.turnRight(1000); delay(1700); drive.stop(); delay(300);//@@@

        servo_res2(); delay(500);

        drive.turnRight(1000); delay(1700); drive.stop(); delay(300);//@@@
        drive.turnRight(1000); delay(1700); drive.stop(); delay(300);//@@@

      } else if (servo2_kit >= 2) {

        drive.turnRight(1000); delay(1700); drive.stop(); delay(300);//@@@
        drive.turnRight(1000); delay(1700); drive.stop(); delay(300);//@@@

        servo_res2(); delay(500);
        servo_res2(); delay(500);

        drive.turnRight(1000); delay(1700); drive.stop(); delay(300);//@@@
        drive.turnRight(1000); delay(1700); drive.stop(); delay(300);//@@@

      } else if (servo1_kit == 1) {

        servo_res(); delay(500);

      } else if (servo2_kit == 1) {

        drive.turnRight(1000); delay(1700); drive.stop(); delay(300);//@@@
        drive.turnRight(1000); delay(1700); drive.stop(); delay(300);//@@@

        servo_res2(); delay(500);

        drive.turnRight(1000); delay(1700); drive.stop(); delay(300);//@@@
        drive.turnRight(1000); delay(1700); drive.stop(); delay(300);//@@@
      }
      break;

    /* ======================
       Yellow victim
       ====================== */
    case 5:
      #if DEBUGMODE
        Serial.println("<----yellow_victim was Located on the Right-hand side by sel7()---->");
      #endif
      for (int i = 0; i < 3; i++) {
        NeoPixel_Color(255, 200, 0);
        delay(1000);
        pixels.clear();
        pixels.show();
        delay(1000);
      }

      if (servo1_kit >= 1) {

        servo_res(); delay(500);

      } else if (servo2_kit >= 1) {

        drive.turnRight(1000); delay(1700); drive.stop(); delay(300);//@@@
        drive.turnRight(1000); delay(1700); drive.stop(); delay(300);//@@@

        servo_res2(); delay(500);

        drive.turnRight(1000); delay(1700); drive.stop(); delay(300);//@@@
        drive.turnRight(1000); delay(1700); drive.stop(); delay(300);//@@@
      }
      break;

    default:
      #if DEBUGMODE
        Serial.print("<----Unknown cam7 event: ");
        Serial.print(receivedData);
        Serial.println(" by sel7()---->");
      #endif
      break;
  }

  drive.stop();
  #if DEBUGMODE
    Serial.println("`----sel7() Finished----`");
  #endif
}

/*******************************************************************************************/
/* get_victim_data2                                                                              
/*処理：OpenMVによって被災者を発見する
/*
/*更新者：清田侑希　2025/2/23
/*清田侑希　2025/3/26 変更点：被災者を発見したときにLEDを点灯させる
/* 清田侑希　2025/3/28 変更点：被災者を発見したときにサーボモーターを動かす
/* 阿部実裕　2026/01/25 変更点:デバックモード実装
/*******************************************************************************************/
//左側のカメラ

void sel8() {//@@@tyousei@@@
  #if DEBUGMODE
    Serial.println(".----sel8() Started----.");
  #endif
  // すでに処理済みなら何もしない
  if (VictimisAlready[x][y]){
    #if DEBUGMODE
      Serial.println("!----sel8() Stopped deu to Already Processed----!");
    #endif
    return;
  }

  // データが無ければ何もしない
  if (!cam8.hasEvent()){
    #if DEBUGMODE
      Serial.println("!----sel8() Stopped deu to No Data----!");
    #endif
    return;
  }

  // ===== ヘッダ探索 =====
  byte header = cam8.readEvent();
  if (header != 0xAA) {
    // 同期ずれ → 破棄
    #if DEBUGMODE
      Serial.println("!----sel8() Stopped deu to Out of Sync----!");
    #endif
    return;
  }

  // ===== 残り2バイト待ち =====
  if (Serial8.available() < 2) {
    // フレーム未完成
    #if DEBUGMODE
      Serial.println("!----sel8()  Stopped deu to Unfinished Frame----!");
    #endif
    return;
  }

  byte receivedData  = cam8.readEvent();
  byte check = cam8.readEvent();

  // ===== チェックサム検証 =====
  if ((receivedData ^ 0xFF) != check) {
    // 壊れたフレーム
    #if DEBUGMODE
      Serial.println("!----sel8() Stopped deu to Broken Frame----!");
    #endif
    return;
  }

  // ===== ここから先は「正しい通信」 =====
  VictimisAlready[x][y] = true;
  #if DEBUGMODE
    Serial.println("<----sel8() Proceeded to Correct Communication without any Error---->");
  #endif
  
  switch (receivedData) {

    /* =========================
       1 : H victim
       ========================= */
    case 1:
      #if DEBUGMODE
        Serial.println("<----H_victim was Located on the Left-hand side by sel8()---->");
      #endif
      for (int i = 0; i < 5; i++) {
        NeoPixel_Color(255, 0, 0);
        delay(650);
        pixels.clear();
        pixels.show();
        delay(650);
      }

      if (servo2_kit >= 2) {
        servo_res2(); delay(650);
        servo_res2(); delay(650);
      }
      else if (servo2_kit == 1 && servo1_kit >= 1) {
        servo_res2(); delay(500);
        drive.turnRight(1000); delay(1700); drive.stop(); delay(300);
        drive.turnRight(1000); delay(1700); drive.stop(); delay(300);
        servo_res(); delay(500);
        drive.turnRight(1000); delay(1700); drive.stop(); delay(300);
        drive.turnRight(1000); delay(1700); drive.stop(); delay(300);
      }
      else if (servo1_kit >= 2) {
        drive.turnRight(1000); delay(1700); drive.stop(); delay(300);
        drive.turnRight(1000); delay(1700); drive.stop(); delay(300);
        servo_res(); delay(500);
        servo_res(); delay(500);
        drive.turnRight(1000); delay(1700); drive.stop(); delay(300);
        drive.turnRight(1000); delay(1700); drive.stop(); delay(300);
      }
      else if (servo2_kit == 1) {
        servo_res2(); delay(500);
      }
      else if (servo1_kit == 1) {
        drive.turnRight(1000); delay(1700); drive.stop(); delay(300);
        drive.turnRight(1000); delay(1700); drive.stop(); delay(300);
        servo_res(); delay(500);
        drive.turnRight(1000); delay(1700); drive.stop(); delay(300);
        drive.turnRight(1000); delay(1700); drive.stop(); delay(300);
      }
      break;

    /* =========================
       2 : S victim
       ========================= */
    case 2:
      #if DEBUGMODE
        Serial.println("<----S_victim was Located on the Left-hand side by sel8()---->");
      #endif
      for (int i = 0; i < 5; i++) {
        NeoPixel_Color(255, 200, 0);
        delay(650);
        pixels.clear();
        pixels.show();
        delay(650);
      }

      if (servo2_kit >= 1) {
        servo_res2(); delay(500);
      }
      else if (servo1_kit >= 1) {
        drive.turnRight(1000); delay(1700); drive.stop(); delay(300);
        drive.turnRight(1000); delay(1700); drive.stop(); delay(300);
        servo_res(); delay(500);
        drive.turnRight(1000); delay(1700); drive.stop(); delay(300);
        drive.turnRight(1000); delay(1700); drive.stop(); delay(300);
      }
      break;

    /* =========================
       4 : red victim
       ========================= */
    case 4:
      #if DEBUGMODE
        Serial.println("<----red_victim was Located on the Left-hand side by sel8()---->");
      #endif
      for (int i = 0; i < 3; i++) {
        NeoPixel_Color(255, 0, 0);
        delay(1000);
        pixels.clear();
        pixels.show();
        delay(1000);
      }

      if (servo2_kit >= 2) {
        servo_res2(); delay(500);
        servo_res2(); delay(500);
      }
      else if (servo2_kit == 1 && servo1_kit >= 1) {
        servo_res2(); delay(500);
        drive.turnRight(1000); delay(1700); drive.stop(); delay(300);
        drive.turnRight(1000); delay(1700); drive.stop(); delay(300);
        servo_res(); delay(500);
        drive.turnRight(1000); delay(1700); drive.stop(); delay(300);
        drive.turnRight(1000); delay(1700); drive.stop(); delay(300);
      }
      else if (servo1_kit >= 2) {
        drive.turnRight(1000); delay(1700); drive.stop(); delay(300);
        drive.turnRight(1000); delay(1700); drive.stop(); delay(300);
        servo_res(); delay(500);
        servo_res(); delay(500);
        drive.turnRight(1000); delay(1700); drive.stop(); delay(300);
        drive.turnRight(1000); delay(1700); drive.stop(); delay(300);
      }
      else if (servo2_kit == 1) {
        servo_res2(); delay(500);
      }
      else if (servo1_kit == 1) {
        drive.turnRight(1000); delay(1700); drive.stop(); delay(300);
        drive.turnRight(1000); delay(1700); drive.stop(); delay(300);
        servo_res(); delay(500);
        drive.turnRight(1000); delay(1700); drive.stop(); delay(300);
        drive.turnRight(1000); delay(1700); drive.stop(); delay(300);
      }
      break;

    /* =========================
       5 : yellow victim
       ========================= */
    case 5:
      #if DEBUGMODE
        Serial.println("<----yellow_victim was Located on the Left-hand side by sel8()---->");
      #endif
      for (int i = 0; i < 3; i++) {
        NeoPixel_Color(255, 200, 0);
        delay(1000);
        pixels.clear();
        pixels.show();
        delay(1000);
      }

      if (servo2_kit >= 1) {
        servo_res2(); delay(500);
      }
      else if (servo1_kit >= 1) {
        drive.turnRight(1000); delay(1700); drive.stop(); delay(300);
        drive.turnRight(1000); delay(1700); drive.stop(); delay(300);
        servo_res(); delay(500);
        drive.turnRight(1000); delay(1700); drive.stop(); delay(300);
        drive.turnRight(1000); delay(1700); drive.stop(); delay(300);
      }
      break;

    default:
      // 3, 6, その他は無視
      #if DEBUGMODE
        Serial.println("<----3, 6, or Other was Located by sel8()---->");
      #endif
      break;
  }
  #if DEBUGMODE
    Serial.println("`----sel8() Finished----`");
  #endif
}

void clearCams() {
  #if DEBUGMODE
    Serial.println(".----clearCams() Started----.");
  #endif
  while (Serial7.available() > 0) {
    Serial7.read();
  }
  while (Serial8.available() > 0) {
    Serial8.read();
  }
  #if DEBUG MODE
    Serial.println("`----clearCams() Finished----`");
  #endif
}

/** ----------spin()---------- @internal
 * @brief 指定角度に指定速度で超信地旋回する関数
 * @param {int} vto 回転して向かう角度。負が左、正が右。+180で回れ右。0も可能。
 * @param {int} vspeed 回転速度。省略推奨。デフォルトは440。
 * @endinternal
 * =====使用ライブラリ=====
 * arduino
 * time
 * MyBasicDrive
 * MyYPRs
 * =====以下変更履歴=====
 * 2026/01/25ーmigi()とひだり()に代わって制作 by.阿部実裕ー令和六年度入学
 */
void spin(int vto, int vspeed = 440)
{
  Serial.print(".----spin() Started to ");Serial.print(vto);Serial.print("° by Speed ");Serial.println(vspeed);Serial.println("----.");
  tiziki_kaitenn();
  MPU.resetYaw();
  delay(50);

  if(vto == 0){
    #if DEBUGMODE
      Serial.println("!----spin() Stopped deu to vto is 0----!");
    #endif
  }
  else if(vto < 0){
    while(MPU.getYaw360() < vto || MPU.getYaw360() > 345){
      drive.turnRight(vspeed);
      delay(100);
      MPU.update();
      #if DEBUGMODE
        Serial.print("Yaw: ");Serial.print(MPU.getYaw360());Serial.println(" ----by spin()");
      #endif
      delay(50);
    }
  }
  else{
    while (MPU.getYaw360() > vto || MPU.getYaw360() < 15) {
      drive.turnLeft(vspeed);
      delay(100);
      MPU.update();
      #if DEBUGMODE 
        Serial.print("Yaw: ");Serial.print(MPU.getYaw360());Serial.println(" ----by spin()");
      #endif
        delay(50);
    }
  }
  Serial.println("`----spin() Finished----`");
}

#if 1
/*右回転                                                                              
*処理：MPU6050で取得したデータをもとに90度右回転するまでモーターを回し続ける
*変更点：ギア搭載による回転方向の反転と適切なdelayの実装
*更新者：清田侑希　2025/1/25
*　清田侑希　2025/3/12 変更点：90度モーターのみで回転し、90度回らなかったら補正を行う
*　清田侑希　2025/3/28 変更点：↑の機能を削除する*/
void migi()
{
  //Serial.println("turnRight");
  #if DEBUGMODE
    Serial.println(".----migi() Started----.");
  #endif
  // ===== 回転前の準備 =====
  tiziki_kaitenn(); // 念のため最新値に
  MPU.resetYaw(); // ★ 相対角の基準を 0° に
  delay(50);
  const float TARGET_YAW = 85.0;// 右に90度//@@@tyousei@@@
  sel7();
  sel8();
  
  // ===== 右回転 =====
  while (MPU.getYaw360() < TARGET_YAW || MPU.getYaw360() > 345) {
    drive.turnRight(440);//@@@tyousei@@@
    delay(100);//@@@ // センサ更新
    MPU.update();
    #if DEBUGMODE
      Serial.print("yaw: ");Serial.print(MPU.getYaw360());Serial.println(" ----by migi()");
    #endif
      delay(50);//@@@
  } 

  clearCams();
  delay(2000);
  sel7();
  sel8();

  // ===== 停止 =====
  delay(200);//@@@
  // tiziki(); // ← 必要ならここで最終補正
  #if DEBUGMODE
    Serial.println("`----migi() Finished----`");
  #endif
  }

/*******************************************************************************************/
/* 左回転                                                                              
/*処理：MPU6050で取得したデータをもとに90度左回転するまでモーターを回し続ける
/*変更点：ギア搭載による回転方向の反転と適切なdelayの実装
/*更新者：清田侑希　2025/1/25
/*　清田侑希　2025/3/12 変更点：90度モーターのみで回転し、90度回らなかったら補正を行う
/*　清田侑希　2025/3/28 変更点：↑の機能を削除する
/*******************************************************************************************/
void hidari() {
  #if DEBUGMODE
    Serial.println(".----hidari() Started----.");
  #endif
  //Serial.println("turnLeft");
  // ===== 回転前の準備 =====
  tiziki_kaitenn();      // 念のため最新値に
  MPU.resetYaw();        // ★ 相対角の基準を 0° に
  delay(50);
  const float TARGET_YAW = 275.0;   // 左に90度 //@@@tyousei@@@
  sel7();
  sel8();
  // ===== 左回転 =====
  while (MPU.getYaw360() > TARGET_YAW || MPU.getYaw360() < 15) {
    drive.turnLeft(440); //@@@tyousei@@@
    delay(100);//@@@
    // センサ更新
    MPU.update();
    // デバッグ用（必要なら）
    #if DEBUGMODE
      Serial.print("yaw = ");Serial.print(MPU.getYaw360());Serial.println(" ----by hidari()");
    #endif
    delay(50);//@@@
  }
    clearCams();
    delay(2000);
    sel7();
    sel8();
  // ===== 停止 =====
  delay(200);//@@@
  // tiziki(); // ← 必要なら最終補正
  #if DEBUGMODE
    Serial.println("`----hidari() Finished----`");
  #endif
}

//これらに代わる関数spin()を製作しましたー2026/01/25 by.阿部実裕ー令和六年度入学
#endif


/*******************************************************************************************/
/*色タイルまたはバンプ                                                                              
/*処理：Serial1にデータが来たら1バイト読み込んで青、黒、ロードセルを検知する
/*Data 1:黒タイル　2：青タイル　3：左の障害物　4：右の障害物　5：前の壁　6：銀のタイル
/*更新者：清田侑希　2025/1/26
/*清田侑希　2025/2/23 変更点：障害物を検知して最初の動作の時に壁との距離を測るようにした
/*清田侑希　2025/3/28 変更点：障害物を検知したとき毎回壁との距離を測るようにした
/*阿部実裕　2026/01/26 変更点：デバックモード実装
/*******************************************************************************************/
bool initialized = false;
void s1() {
  #if DEBUGMODE
    Serial.println(".----s1() Started----.");
  #endif
  /* ======================================
      初回呼び出し時だけの初期化
      ====================================== */
  if (!initialized) {
    #if DEBUGMODE
      Serial.println("<----initialized by s1()---->");
    #endif
    while (Serial1.available() > 0) {
        Serial1.read();
    }
    initialized = true;
  }
  /* ======================================
      イベントがなければ何もしない
      ====================================== */
  if (!colorUART.hasEvent()) {
    #if DEBUGMODE
      Serial.println("!----s1() Stopped deu to No Event----!");
    #endif
    return;
  }

  /* ======================================
      ヘッダ待ち
      ====================================== */
  byte header = colorUART.readEvent();
  if (header != 0xAA) {
    #if DEBUGMODE
      Serial.println("!----s1() Stopped deu to No Header----!");
    #endif
    return;   // ヘッダじゃなければ破棄
  }

  /* ======================================
      ここから「3バイト揃うまで待つ」
      ====================================== */
  unsigned long start = millis();
  while (Serial1.available() < 2) {
    if (millis() - start > 5) {//@@@tyousei@@@
        // 5ms 以内に揃わなければ諦める
        #if DEBUGMODE
          Serial.println("!----s1() Stopped deu to Not Ready for 3-byte in 5-ms----!");
        #endif
        return;
    }
  }

  uint8_t receivedData2 = colorUART.readEvent();
  uint8_t check         = colorUART.readEvent();

  /* ======================================
      チェックサム確認
      ====================================== */
  if ((receivedData2 ^ 0xFF) != check) {
    #if DEBUGMODE
      Serial.println("!----s1() Stopped deu to UART EVENT Checksum Error----!");
    #endif
    return;
  }

  #if DEBUGMODE
    Serial.println("<----s1() Proceeded to Correct Communication without any Error---->");
    Serial.print("UART EVENT = ");Serial.print(receivedData2);Serial.println("----by s1()");
  #endif

  /* ======================================
      イベント処理（1イベント）
      ====================================== */
  switch (receivedData2) {
    case 1:  // BLACK TILE
      #if DEBUGMODE
        Serial.println("<----Black Tile was Detected by s1()---->");
      #endif
      BlackTile = true;
      while (count2 > -2) {
        drive.forward(-303);
        delay(100);
        count2--;
      }
      drive.stop();
      count2 = 40;
      break;

    case 2:  // BLUE TILE
      #if DEBUGMODE
        Serial.println("<----Blue Tile was Detected by s1()---->");
      #endif
      blue_count = true;
      break;

    case 5:  // FRONT BUMP
      #if DEBUGMODE
        Serial.println("<----Front Bump was Detected by s1()---->");
      #endif
      drive.stop();
      delay(500);
      drive.forward(-2025);
      delay(1000);
      drive.stop();
      if (count2 < 20 && bump_giveup_count > 1) {
        Gap = true;
      }
      count2 = 40;
      break;

    case 3:  // LEFT
      #if DEBUGMODE
        Serial.println("<----Left by s1()---->");
      #endif
      drive.stop(); delay(500);
      drive.turnLeft(1023); delay(500); delay(500);
      drive.forward(-1240); delay(700); delay(500);
      drive.turnRight(1534); delay(700); delay(500);
      receivedIndex = 0;
      front_wall = false;
      get_tof_data();
      if (receivedData[0] <= 90 && receivedData[3] <= 90 && abs(receivedData[0] - receivedData[3]) < 30) {
        if(count2 < 20) Gap = true; count2 = 46; 
      } 
      count2 -= 6;
      bump_giveup_count++;
      test++;      
    break;

    case 4:
      #if DEBUGMODE
        Serial.println("<----Right by s1()---->");
      #endif
      drive.stop(); delay(500);
      drive.turnRight(1023); delay(500); delay(500);
      drive.forward(-1240); delay(700); delay(500);
      drive.turnLeft(1534); delay(700); delay(500);
      receivedIndex = 0;
      front_wall = false;
      get_tof_data();
      if (receivedData[0] <= 90 && receivedData[3] <= 90 && abs(receivedData[0] - receivedData[3]) < 30) {
        if(count2 < 20) Gap = true; count2 = 46; 
      } 
      count2 -= 6;
      bump_giveup_count++;
      test++;
      break;

    case 6:  // CHECK POINT
      #if DEBUGMODE
        Serial.println("<----Check Point was Detected by s1()---->");
      #endif
      CheckX = x;
      CheckY = y;
      CheckD = Direction;
      NeoPixel_Rear_ON();
      delay(200);
      NeoPixel_Rear_OFF();
      break;

    default:
      #if DEBUGMODE
        Serial.println("<----Unkown Event by s1()---->");
      #endif
      break;
  }
  #if DEBUGMODE
    Serial.println("`----s1() Finished----`");
  #endif
}

/* ======================================
   s1 初期化リセット
   ====================================== */
void s1_reset() {
  #if DEBUGMODE
    Serial.println("<----s1 initialized by s1_reset()---->");
  #endif
  initialized = false;
}

/*******************************************************************************************/
/* 直進                                                                              
/*処理：モーターを回しながら直進この時カラーセンサーとロードセル、カメラの割り込みと坂の検知が入る
/*
/*更新者：清田侑希　2025/1/26
/*清田侑希　2025/3/27 変更点：tofセンサーの値を元にして壁と機体の角度のズレを計算し壁と常に平行になるように補正する
/*清田侑希　2025/3/30　変更点：tofセンサーの値を元にしてタイルの中心とのズレを計算し、常に左右方向でタイルの中心にいるように補正するようにする
/*阿部実裕　2025/01/25 変更点：デバックモード実装
/*******************************************************************************************/
void susumu_heitan() {
  #if DEBUGMODE
    Serial.println(".----susumu_heitan() Started----.");
  #endif
  s1_reset();
  /* ===== 固定パラメータ ===== */
  const int BIAS_PWR  = 60;//@@@tyousei@@@
  const int BIAS_TIME = 80;//@@@
  const int FORWARD_PWR  = 303;//@@@
  const int FORWARD_TIME = 90;//@@@

  /* ===== 距離補正パラメータ ===== */
  const int TOF_DIFF_TH   = 15;   // 左右差がこれ以上で補正//@@@
  const int MAX_CORRECT   = 5;    // 最大補正角（deg）//@@@
  const int CORRECT_PWR  = 70;//@@@
  const int CORRECT_DELAY = 20;//@@@

  count2 = 0;
  /* ---------- 姿勢補正（MPU・絶対角） ---------- */
  if (Status == 3) {
    #if DEBUGMODE
      Serial.println("<----Status is 3 in susumu_heitan()---->");
    #endif
    delay(80);
    tiziki();      // ★ ここはそのまま使う
    delay(200);
  }

  /* ---------- 距離による角度補正（実験用・重要） ---------- */
  if (right_wall == true) {
    #if DEBUGMODE
      Serial.println("<----right_wall is true in susumu_heiatan()---->");
    #endif
    int diff = receivedData[4] - receivedData[5];  // 左右差
    #if DEBUGMODE
      Serial.print("ToF diff: ");Serial.print(diff);Serial.println(" ----by susumu_heitan()");
    #endif
    if (abs(diff) >= TOF_DIFF_TH) {
      // 差を角度に変換（超簡易・安全版）
      int correct_deg = abs(diff) / 5;   // ← 実機で調整OK//@@@tyousei@@@
      correct_deg = constrain(correct_deg, 1, MAX_CORRECT);
      for (int i = 0; i < correct_deg; i++) {
        if (diff > 0) {
          // 左が遠い → 左向き → 右に戻す
          drive.turnRight(CORRECT_PWR);
        } else {
          drive.turnLeft(CORRECT_PWR);
        }
        delay(CORRECT_DELAY);
      }
      drive.stop();
      delay(150);
    }
  }

  /* ---------- 初動バイアス ---------- */
  #if DEBUGMODE
    Serial.println("<----Initial Bias by susumu_heitan()---->");
  #endif
  drive.forward(BIAS_PWR);
  delay(BIAS_TIME);
  
  delay(50);

  /* ---------- 本前進（1タイル） ---------- */
  #if DEBUGMODE
    Serial.println("<----Move One Tile Forward by susumu_heitan()---->");
  #endif
  while (count2 < susumu_kaisuu) {
    drive.forward(FORWARD_PWR);
    for (int t = 0; t < FORWARD_TIME; t += 1) {
      s1();
      sel7();
      sel8();
      delay(1);
      if(BlackTile) break;
    }
    if(BlackTile) break;
    Serial.println(count2);
    count2++;
  }
  drive.stop();
  delay(100);

  /* ---------- 坂判定（既存） ---------- */
  tiziki_2();
  delay(50);
  if (katamuki_true < -10) {//@@@tyousei@@@
    while (katamuki_true < -18) {//@@@
      drive.forward(400);
      delay(100);
      tiziki_2();
    }
  } 
  else if (katamuki_true > 10) {//@@@
    while (katamuki_true > 10) {//@@@
      drive.forward(400);
      delay(100);
      tiziki_2();
    }
  }

  /* ---------- タイル抜け ---------- */
  drive.forward(400);
  delay(300);
  drive.stop();

  /* ---------- 青タイル ---------- */
  if (blue_count == true) {
    #if DEBUGMODE
      Serial.println("<----Blue Tile Delay---->");
    #endif
    delay(5500);
  }

  /* ---------- 後処理 ---------- */
  count2 = 0;
  bump_giveup_count = 0;
  Status = 0;
  blue_count = false;
  drive.stop();
  delay(200);
  #if DEBUGMODE
    Serial.println("`----susumu_heitan() Finished----`");
  #endif
}



/*アルゴリズム用変数*/
int8_t toutatu_zahyou[90][90];//そのマスの到達回数//@@@
int RightWeight = 0;
int FrontWeight = 0;
int LeftWeight = 0;

const uint8_t GoalX = 50;//@@@tyousei@@@
const uint8_t GoalY = 50;//@@@
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
/*更新者：阿部実裕　2026/01/26 デバックモード実装
/*******************************************************************************************/

void send_display(){
  #if DEBUGMODE
    Serial.println(".----send_display() Started----.");
  #endif
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
  #if DEBUGMODE
    Serial.print("Sent data: ");//デバック用にシリアルモニタに送信
    for (int i = 0; i < 7; i++) {
      Serial.print(data_for_send[i], BIN);
      if (i < 6) Serial.print(", ");
    }
    Serial.println(" ----by send_display()");
  #endif
  delay(200); // 必要に応じて調整
  #if DEBUGMODE
    Serial.println("`----send_display() Finished----`");
  #endif
}

/*******************************************************************************************/
/*行き止まりの検索                                                                              
/*処理：全てのマスについて、
/*      前後左右の三か所以上が行き止まりor壁のときそのマスの到達回数を+100する（マスの到達回数が100未満のとき）
/*
/*更新者：吉ノ薗2025/02/02いったん凍結
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
/*更新者：阿部実裕　2026/01/26　デバックモード実装
/*******************************************************************************************/
int8_t judge(){
  #if DEBUGMODE
    Serial.println(".----judge() Started----.");
  #endif
  switch (Direction){//向きによって重みをつける
    case East:
      #if DEBUGMODE
        Serial.println("<----East Case in judge()---->");
      #endif
      RightWeight = toutatu_zahyou[x][y+1] * 5 + 1;//@@@tyousei@@@ hosoku: *5 tokano suuji wo hennkou suru noga iiyo
      FrontWeight = toutatu_zahyou[x+1][y] * 5 + 2;
      LeftWeight = toutatu_zahyou[x][y-1] * 5 + 3;
      break;

    case North:
      #if DEBUGMODE
        Serial.println("<----North Case in judge()---->");
      #endif
      RightWeight = toutatu_zahyou[x+1][y] * 5 + 1;
      FrontWeight = toutatu_zahyou[x][y-1] * 5 + 2;
      LeftWeight = toutatu_zahyou[x-1][y] * 5 + 3;
      break;

    case West:
      #if DEBUGMODE
        Serial.println("<----West Case in judge()---->");
      #endif
      RightWeight = toutatu_zahyou[x][y-1] * 5 + 1;
      FrontWeight = toutatu_zahyou[x-1][y] * 5 + 2;
      LeftWeight = toutatu_zahyou[x][y+1] * 5 + 3;
      break;

    case South:
      #if DEBUGMODE
        Serial.println("<----South Case in judge()---->");
      #endif
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
  #if DEBUGMODE
    Serial.println("`----judge() Finished----`");
  #endif
  return GoTo;
}

/*******************************************************************************************/
/* 動く方向                                                                              
/*処理：それぞれの方位のときに行く方向によって座標を移動・方位を変更する
/*      後進のときは全部壁のときだから、現在の重みを+100する
/*
/*更新者：吉ノ薗2025/01/22
/*更新者:阿部実裕ー令和六年度入学　2026/01/26　デバックモード実装
/*******************************************************************************************/
void MoveTo(int8_t GoTo)
{
  #if DEBUGMODE
    Serial.println(".----MoveTo() Started----.");
  #endif
  //EffectiveDeadEnd();//より効率的な行き止まりの検索
  switch (Direction){
    case East:
      switch (GoTo){
        case Right:
          #if DEBUGMODE
            Serial.println("<----Right & East Case in MoveTo()---->");
          #endif
          Status = 4;
          y += 1;
          Direction = South;
          break;
        
        case Front:
          #if DEBUGMODE
            Serial.println("<----Front & East Case in MoveTo()---->");
          #endif
          Status = 3;
          x += 1;
          Direction = East;
          break;
        
        case Left:
          #if DEBUGMODE
            Serial.println("<----Left & East Case in MoveTo()---->");
          #endif
          Status = 5;
          y += -1;
          Direction = North;
          break;

        case Back:
          #if DEBUGMODE
            Serial.println("<----Back & East Case in MoveTo()---->");
          #endif
          Status = 6;
          x += -1;
          Direction = West;
          break;
      }
      break;
    case North:
      switch (GoTo){
        case Right:
          #if DEBUGMODE
            Serial.println("<----Right & North Case in MoveTo()---->");
          #endif
          Status = 4;
          x += 1;
          Direction = East;
          break;
        
        case Front:
          #if DEBUGMODE
            Serial.println("<----Front & North Case in MoveTo()---->");
          #endif
          Status = 3;
          y += -1;
          Direction = North;
          break;
        
        case Left:
          #if DEBUGMODE
            Serial.println("<----Left & North Case in MoveTo()---->");
          #endif
          Status = 5;
          x += -1;
          Direction = West;
          break;

        case Back:
          #if DEBUGMODE
            Serial.println("<----Back & North Case in MoveTo()---->");
          #endif
          Status = 6;
          y += 1;
          Direction = South;
          break;
      }
      break;
    
    case West:
      switch (GoTo){
        case Right:
          #if DEBUGMODE
            Serial.println("<----Right & West Case in MoveTo()---->");
          #endif
          Status = 4;
          y += -1;
          Direction = North;
          break;
        
        case Front:
          #if DEBUGMODE
            Serial.println("<----Front & West Case in MoveTo()---->");
          #endif
          Status = 3;
          x += -1;
          Direction = West;
          break;
        
        case Left:
          #if DEBUGMODE
            Serial.println("<----Left & West Case in MoveTo()---->");
          #endif
          Status = 5;
          y += 1;
          Direction = South;
          break;
        
        case Back:
          #if DEBUGMODE
            Serial.println("<----Back & West Case in MoveTo()---->");
          #endif
          Status = 6;
          x += 1;
          Direction = East;
          break;
      }
      break;
    
    case South:
      switch (GoTo){
        case Right:
          #if DEBUGMODE
            Serial.println("<----Right & South Case in MoveTo()---->");
          #endif
          Status = 4;
          x += -1;
          Direction = West;
          break;
        
        case Front:
          #if DEBUGMODE
            Serial.println("<----Front & South Case in MoveTo()---->");
          #endif
          Status = 3;
          y += 1;
          Direction = South;
          break;
        
        case Left:
          #if DEBUGMODE
            Serial.println("<----Left & South Case in MoveTo()---->");
          #endif
          Status = 5;
          x += 1;
          Direction = East;
          break;
        
        case Back:
          #if DEBUGMODE
            Serial.println("<----Back & South Case in MoveTo()---->");
          #endif
          Status = 6;
          y += -1;
          Direction = North;
          break;
    }
    break;
  }
  toutatu_zahyou[x][y] += 1;//移動先のマスの到達回数をプラスしている
  #if DEBUGMODE
    Serial.println("`----MoveTo() Finished----`");
  #endif
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
/*更新者：阿部実裕　2026/01/26 デバックモード実装
/*******************************************************************************************/
void TileStatus(){
  #if DEBUGMODE
    Serial.println(".----TileStatus() Started----.");
  #endif
  if(BlackTile){
      toutatu_zahyou[x][y] += 100;
  }
  switch (Direction){
    case North:
      if(BlackTile || Gap){//黒タイルまたはずれの信号が送られていた場合戻る
        #if DEBUGMODE
          Serial.println("<----North Case BlackTile or Gap in TileStatus()---->");
        #endif
        y += 1;
      }
      if(Slope){//坂の信号が送られていた場合座標を更に進める
        #if DEBUGMODE
          Serial.println("<----North Case Slope in TileStatus()---->");
        #endif
        y += -1;
      }
      break;

    case East:
      if(BlackTile || Gap){//黒タイルの信号が送られていた場合戻る
        #if DEBUGMODE
          Serial.println("<----East Case BlackTile or Gap in TileStatus()---->");
        #endif
        x += -1;
      }
      if(Slope){//坂の信号が送られていた場合座標を更に進める
        #if DEBUGMODE
          Serial.println("<----East Case Slope in TileStatus()---->");
        #endif
        x += 1;
      }
      break;
    
    case West:
      if(BlackTile || Gap){//黒タイルの信号が送られていた場合戻る
        #if DEBUGMODE
          Serial.println("<----West Case BlackTile or Gap in TileStatus()---->");
        #endif
        x += 1;
      }
      if(Slope){//坂の信号が送られていた場合座標を更に進める
        #if DEBUGMODE
          Serial.println("<----West Case Slope in TileStatus()---->");
        #endif
        x += -1;
      }
      break;

    case South:
      if(BlackTile || Gap){//黒タイルの信号が送られていた場合戻る
        #if DEBUGMODE
          Serial.println("<----South Case BlackTile or Gap in TileStatus()---->");
        #endif
        y += -1;
      }
      if(Slope){//坂の信号が送られていた場合座標を更に進める
        #if DEBUGMODE
          Serial.println("<----South Case Slope in TileStatus()---->");
        #endif
        y += 1;
      }
      break;
  }
  BlackTile = false;
  Slope = false;
  Gap = false;
  #if DEBUGMODE
    Serial.println("`----TileStatus() Finished----`");
  #endif
}

/*******************************************************************************************/
/* 最短経路の方位                                                                              
/*処理：現在のマスのコストの、-1のコストのマスを探す
/*更新者：吉ノ薗2025/01/22
/*       吉ノ薗2025/03/28　差が１でも壁があれば行かないようにした
/*更新者：阿部実裕　2026/01/26　デバックモード実装
/*******************************************************************************************/
int WhichWay(uint8_t a,uint8_t b)
{
  if ((cost[a][b] - cost[a + 1][b] == 1) && !(kabe_zahyou[a][b] & 8)) {
    #if DEBUGMODE
      Serial.println("<----WhichWay() Returned East---->");
    #endif
    return East;
  }
  if ((cost[a][b] - cost[a][b - 1] == 1) && !(kabe_zahyou[a][b] & 1)) {
    #if DEBUGMODE
      Serial.println("<----WhichWay() Returned North---->");
    #endif
    return North;
  }
  if ((cost[a][b] - cost[a - 1][b] == 1) && !(kabe_zahyou[a][b] & 4)) {
    #if DEBUGMODE
      Serial.println("<----WhichWay() Returned West---->");
    #endif
    return West;
  }
  if ((cost[a][b] - cost[a][b + 1] == 1) && !(kabe_zahyou[a][b] & 2)) {
    #if DEBUGMODE
      Serial.println("<----WhichWay() Returned South---->");
    #endif
    return South;
  }
  #if DEBUGMODE
    Serial.println("!----WhichWay() Returned 0 : Error due to Unknown Data----!");
  #endif
  //Serial.println("Error@whichway");
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
/*更新者：阿部実裕　2026/01/26　デバックモード実装
/*******************************************************************************************/
void BFS()
{
  //hidari();//デバッグ用
  uint8_t a = x;
  uint8_t b = y;
  cost[a][b] = 1;//現在地のコストを1にする
  #if DEBUGMODE
  //Serial.println("GotoHome:");
    Serial.println(".----BFS() Started----.");
  #endif
  while(!(a == GoalX && b == GoalY)){//ここ詰まったら中断するようにしたかった

    reach_time[a][b] = 1;//そのマスを訪問済みにする

    #if DEBUGMODE
      Serial.print("a: ");
      Serial.print(a);
      Serial.print(" b: ");
      Serial.println(b);
      Serial.print("kabe_zahyou[a][b]: ");
      Serial.println(kabe_zahyou[a][b]);
      Serial.print("cost[a][b]       : ");
      Serial.println(cost[a][b]);
      Serial.println(" ----by BFS()");
    #endif

    for(int n = 1; n <= 8; n *= 2){//そのマスの周りのマスのコストを＋１する
      //int result = static_cast<int>(pow(2, n));
      #if DEBUGMODE
        Serial.print("n: ");
        Serial.print(n);
        Serial.println(" ----by BFS()")
      #endif

      if(!(kabe_zahyou[a][b] & n)) {//kabe_zahyou[][]は0000 の4ビットに絶対方向の東8西4南2北1をそれぞれ割り当てる
        switch(n){
          case 1://北の壁がない
            #if DEBUGMODE
              Serial.println("<----No Wall to the North by BFS()---->");
            #endif
            if((!reach_time[a][b-1]) && !(kabe_zahyou[a][b-1] & 16)){//その先のマスが訪問済みでない&その先のマスが探索済み
              #if DEBUGMODE
                Serial.println("<----the Square no Searched && the Square Beyond it Searched by BFS()---->");
              #endif
              cost[a][b-1] = cost[a][b] + 1;
              //キューの末尾に入れる
              Q.push(a);
              Q.push(b-1);
            }
            break;

          case 2://南の壁がない
            #if DEBUGMODE
              Serial.println("<----No Wall to the South by BFS()---->");
            #endif
            if((!reach_time[a][b+1]) && !(kabe_zahyou[a][b+1] & 16)){//その先のマスが訪問済みでない&その先のマスが探索済み
              #if DEBUGMODE
                Serial.println("<----the Square no Searched && the Square Beyond it Searched by BFS()---->");
              #endif
              cost[a][b+1] = cost[a][b] + 1;
              //キューの末尾に入れる
              Q.push(a);
              Q.push(b+1);
            }
            break;

          case 4://西の壁がない
            #if DEBUGMODE
              Serial.println("<----No Wall to the West by BFS()---->");
            #endif
            if((!reach_time[a-1][b]) && !(kabe_zahyou[a-1][b] & 16)){//その先のマスが訪問済みでない&その先のマスが探索済み
              #if DEBUGMODE
                Serial.println("<----the Square no Searched && the Square Beyond it Searched by BFS()---->");
              #endif
              cost[a-1][b] = cost[a][b] + 1;
              //キューの末尾に入れる
              Q.push(a-1);
              Q.push(b);
            }
            break;

          case 8://東の壁がない
            #if DEBUGMODE
              Serial.println("<----No Wall to the East by BFS()---->");
            #endif
            if((!reach_time[a+1][b]) && !(kabe_zahyou[a+1][b] & 16)) {//その先のマスが訪問済みでない&その先のマスが探索済み
              #if DEBUGMODE
                Serial.println("<----the Square no Searched && the Square Beyond it Searched by BFS()---->");
              #endif
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
    if (Q.size() < 2){
      #if DEBUGMODE
        Serial.println("!----the Queue is too Short----!");
      #endif
      break;  // キューの要素が足りない場合は終了
    }
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
        #if DEBUGMODE
          Serial.println("!----Error in Derection Switch in BFS()----!");
        #endif
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
  #if DEBUGMODE
    Serial.println("`----BFS() Finished----`");
  #endif
}
//~~~


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
  colorUART.begin(&Serial1);
  Serial2.begin(1000000);//using_servo_sts3032
  Serial3.begin(9600);  // using_tof
    // ToF UART ライブラリ初期化
  tofUART.begin(&Serial3);
  Serial7.begin(19200);//using_cam1
  Serial8.begin(19200);//using_cam2
  // ===== カメラUARTラッパ初期化 =====
  cam7.begin(&Serial7);
  cam8.begin(&Serial8);
  pinMode(6, INPUT); //スタート用スイッチのpin設定
  pinMode(SERVO_PIN, OUTPUT);//レスキューキット用のサーボモーターのpin設定
  pinMode(SERVO_PIN2, OUTPUT);//上に同じ
  delay(2000);
  /*MPUのセットアップ***********************************************************************************************************/
  Wire.begin();
  Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
  if (!MPU.begin()) {
    Serial.println("MyYPRs MPU init failed");
    while (1);
  }
  MPU.resetYaw();

  
  /* モーターのセットアップ */
  SoftwareSerial.begin(9600);

  sms_sts.pSerial = &Serial2;   // ← これは正しい
  delay(1000);

  ID[0] = 1; // 右前
  ID[1] = 2; // 左前
  ID[2] = 3; // 右後
  ID[3] = 4; // 左後

  Speed[0] = 6800;
  Speed[1] = 6800;
  Speed[2] = 6800;
  Speed[3] = 6800;
  
  ACC[0] = 50;
  ACC[1] = 50;
  ACC[2] = 50;
  ACC[3] = 50;

  Serial.println("test");

  /* ★ ここが正解 ★ */
  drive.begin(&sms_sts,
            ID,
            Speed,
            ACC);

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
String pcLine = "";
    int debug = 0;
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
bool sub_ready = false;
unsigned long lastBlink = 0;
bool ledState = false;

void loop(){
    /*.............スタートスイッチがオンになるまで待機する部分...........*/
while (digitalRead(6) == LOW) {
    delay(10);
    if (deb == 100){
      Serial.println("Waiting...@start-button");
      deb = 0;
    }
    deb = deb + 1; 
    // ===== PC → メインマイコン デバッグ通信 =====

#if DEBUG
if (Serial.available() > 0) {
    char cmd = Serial.read();

    if (cmd == '\n' || cmd == '\r') {
        pcLine.trim();

        // ★ 空行は無視（超重要）
        if (pcLine.length() == 0) {
            pcLine = "";
            return;
        }

        if (pcLine == "/dbg com test") {
            Serial.println("success");
        }
        else if (pcLine == "/dbg s3 test") {
            Serial.println("[PC] F command received");

            byte data[6];
            bool ok = tofUART.request(0x90, data, 150);

            if (!ok) {
                Serial.println("[MyUARTs] timeout or error");
            } else {
                Serial.print("[MyUARTs] recv: ");
                bool success = true;
                for (int i = 0; i < 6; i++) {
                    Serial.print(data[i]);
                    if (i < 5) Serial.print(",");
                    if (data[i] != i + 1) success = false;
                }
                Serial.println();

                if (success) Serial.println("success");
                else Serial.println("data mismatch");
            }
        }
        else if (pcLine == "/dbg s3 run") {
            Serial.println("[PC] F command received");

            byte data[6];
            bool ok = tofUART.request(0x01, data, 150);

            if (!ok) {
                Serial.println("[MyUARTs] timeout or error");
            } else {
                Serial.print("[MyUARTs] recv: ");
                bool success = true;
                for (int i = 0; i < 6; i++) {
                    Serial.print(data[i]);
                    if (i < 5) Serial.print(",");
                    if (data[i] != i + 1) success = false;
                }
                Serial.println();

                if (success) Serial.println("success");
                else Serial.println("data mismatch");
            }
        }
        else if (pcLine == "/dbg s3 adv") {
            Serial.println("[PC] command received");
            get_tof_data();
            Serial.print("r="); Serial.println(right_wall);
            Serial.print("f="); Serial.println(front_wall);
            Serial.print("l="); Serial.println(left_wall);
            Serial.println("end running");
        }
        else if (pcLine == "/dbg s1 run") {
            Serial.println("[PC] command received");
            s1_reset();
            for(int i = 0;i<6;i++){
              s1();
              Serial.println("running...");
              delay(1000);
            }
            Serial.println("end running");
        }
        else if (pcLine == "/dbg sel8 run") {
            Serial.println("[PC] command received");
            for(int i = 0;i<6;i++){
              sel7();
              sel8();
              Serial.println("running...");
              delay(1000);
            }
            Serial.println("end running");
        }
        else if (pcLine == "/dbg mpu run") {
            Serial.println("[PC] command received");
            for(int i = 0;i<6;i++){
              Serial.println("before update");
              MPU.update();
              Serial.println("after update");
              Serial.print("yaw=");
              Serial.print(MPU.getYaw360());
              Serial.print(" pitch=");
              Serial.print(MPU.getPitch());
              Serial.print(" roll=");
              Serial.println(MPU.getRoll());
              delay(1000);
            }
            Serial.println("end running");
        }
        else if (pcLine == "/dbg num show") {
            Serial.println("[PC] command received");
            Serial.print("[PC] test =");
            Serial.println(test);
            Serial.println(blue_count);
        }
        else if (pcLine == "/dbg num reset") {
            Serial.println("[PC] command received");
            test = 0;
            if(test == 0){
              Serial.println("success");
            }
        }
        else if (pcLine == "/dbg go forward") {
            Serial.println("[PC] command received");
            susumu_heitan();
        }
        else if (pcLine == "/dbg go left") {
            Serial.println("[PC] command received");
            hidari();
        }
        else if (pcLine == "/dbg go right") {
            Serial.println("[PC] command received");
            migi();
        }
        else if (pcLine == "/dbg sv1 run") {
            Serial.println("[PC] command received");
            servo_res();
        }
        else if (pcLine == "/dbg sv2 run") {
            Serial.println("[PC] command received");
            servo_res2();
        }
        else if (pcLine == "/set sv1 run") {
            Serial.println("[PC] command received");
            motor_servo(100,100);
        }
        else if (pcLine == "/set sv2 run") {
            Serial.println("[PC] command received");
            motor_servo2(0,1000);
            Serial.println("end running");
        }
        else {
            Serial.print("Unknown cmd: ");
            Serial.println(pcLine);
        }

        pcLine = "";
    }
    else {
        pcLine += cmd;
    }
}
#endif

    // ===== サブマイコン READY イベント監視 =====
    uint8_t evt;
    if (colorUART.readEventFrame(evt)) {
        if (evt == EVT_READY) {
            sub_ready = true;
            Serial.println("SUB MCU READY");
        }
    }

    // ===== NeoPixel 表示 =====
    unsigned long now = millis();

    if(sub_ready == true && debug == 0) {
        // 準備完了：緑点滅（0.5秒周期）
        if (now - lastBlink > 500 && debug < 3) {
            lastBlink = now;
            ledState = !ledState;

            if (ledState) {
                NeoPixel_Color(0, 0, 100);  // 緑
            } else {
                pixels.clear();
                pixels.show();
                debug++;
            }
            
        }
    }
    if (digitalRead(6) == HIGH) {
        // サブマイコンの残骸データを消す
        while (Serial3.available() > 0) {
            Serial3.read();
        }

        // チェックポイント復帰
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
        if(now_seconds - firstseconds >= 360){//@@@tyousei@@@
            Status = 2;//帰還開始
            start_Gohome = true;
        }

        Serial.print("keika jikann :"); Serial.println(now_seconds - firstseconds);//debug
        
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
        delay(200);//@@@
        TileStatus();
        break;

    case 4://右折
        migi();
        delay(500);//@@@
        susumu_heitan();
        delay(200);//@@@
        TileStatus();
        break;

    case 5://左折
        hidari();
        delay(500);//@@@
        susumu_heitan();
        delay(200);//@@@
        TileStatus();
        break;

    case 6://後進
        hidari();
        delay(500);//@@@
        hidari();
        delay(500);//@@@
        susumu_heitan();
        delay(200);//@@@
        TileStatus();
        break;
    }
    
}
