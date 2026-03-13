/*ここにある関数
ライブラリ
変数定義
setup
loop
*/


#define DEBUG 1//on->1 off->0


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
#include <Adafruit_NeoPixel.h>
#include <queue>
#include <stack>
#include "MyBasicDrive.h"
#include "MyUARTs_read.h"
#include "MyYPRs.h"

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
int servo1_kit = 100; //レスキューキットの残数//@@@tyousei@@@
int servo2_kit = 100; //上に同じ//@@@
/*..........................................................*/
/*センサー類変数*/
unsigned int SERVO_POS = 0; //変数SERVO_POS=0とする
byte ID[4]; //IDはそれぞれのモーターのID
s16 Position[4]; //Positionはモーターが回る角度(右一回転＝4095)
u16 Speed[4]; 
byte ACC[4]; 
int susumu_kaisuu= 42;//@@@tyousei@@@
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

#define CMD_TOF 0x01
#define EVT_READY 7

bool sub_ready = false;
unsigned long lastBlink = 0;
bool ledState = false;

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


#if DEBUG

float test = 0;

#endif

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
debug_commands();
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
