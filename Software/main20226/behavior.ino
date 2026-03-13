/*ここにある関数
getYawPitchRoll
tiziki_kaitenn
tiziki
tiziki_2
NeoPixel群
get_tof_data
motor_servo
servo_res
sel7 / sel8
clearCams
migi / hidari
s1
susumu_heitann
send_display
*/


/*******************************************************************************************/
/* Yaw・Pitch・Roll角の取得                                                                            
/*処理：MPU6050を用いて回転や坂検知のためのRoll・Yaw角の取得
/*
/*更新者：清田侑希　2025/1/25
/*
/*******************************************************************************************/
void getYawPitchRoll() {
  MPU.update();   // ★ 毎回必ず呼ぶ

  yaw   = MPU.getYaw360();    // 0〜360
  pitch = MPU.getPitch();  // deg
  roll  = MPU.getRoll();   // deg

  Serial.print("Yaw: ");
  Serial.print(yaw);
  Serial.print(" Pitch: ");
  Serial.print(pitch);
  Serial.print(" Roll: ");
  Serial.println(roll);
}


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
  Serial.print("kakudo-ture2 @tzk-kaitenn:");
  Serial.println(kakudo_true2);
}



/*******************************************************************************************/
/* 地磁気                                                                              
/*処理：回転後に発生する機体の方向の誤差をMPU6050によって補正
/* 変更点：モーターにギアを導入したのでモーターの回転方向が逆になってます
/*更新者：清田侑希 2025/1/25
/*清田侑希 2025/3/27 変更点：基準となる角度を定数から変数に変更
/*hukuda 2025/12/22　変更点：ライブラリ化①
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
  Serial.print("katamuki-true:");
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

  // ===== ToF 要請 + 応答 =====
  bool ok = tofUART.request(CMD_TOF, tof_buf, 200);  //@@@tyousei@@@
  // ↑ timeout は 120〜150ms 推奨（後述）

  if (!ok) {
    Serial.println("TOF timeout");

    front_wall = false;
    left_wall  = false;
    right_wall = false;
    Status = 0;
    return;
  }

  // ===== データ確定 =====
  for (int i = 0; i < 6; i++) {
    receivedData[i] = tof_buf[i];
  }

  Serial.print("TOF: ");
  for (int i = 0; i < 6; i++) {
    Serial.print(receivedData[i]);
    if (i < 5) Serial.print(",");
  }
  Serial.println();

  // ===== 壁判定 =====
  front_wall = (receivedData[0] <= isItWall && receivedData[3] <= isItWall);
  left_wall  = (receivedData[1] <= isItWall && receivedData[2] <= isItWall);
  right_wall = (receivedData[4] <= isItWall && receivedData[5] <= isItWall);

  Status = 1;
}

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

/*******************************************************************************************/
/* get_victim_data                                                                              
/*処理：OpenMVによって被災者の発見
/*
/*更新者：清田侑希 2025/2/23
/*清田侑希 2025/3/26 変更点：被災者を発見したときにLEDを点灯させる
/* 清田侑希 2025/3/28 変更点：被災者を発見したときにサーボモーターを動かす
/*******************************************************************************************/
//右側のカメラ
void sel7() {

  // すでに処理済みなら何もしない
  if (VictimisAlready[x][y]) return;

  // データが無ければ何もしない
  if (!cam7.hasEvent()) return;

  // ===== ヘッダ探索 =====
  byte header = cam7.readEvent();
  if (header != 0xAA) {
    // 同期ずれ → 破棄
    return;
  }

  // ===== 残り2バイト待ち =====
  if (Serial7.available() < 2) {
    // フレーム未完成
    return;
  }

  byte receivedData  = cam7.readEvent();
  byte check = cam7.readEvent();

  // ===== チェックサム検証 =====
  if ((receivedData ^ 0xFF) != check) {
    // 壊れたフレーム
    return;
  }

  // ===== ここから先は「正しい通信」 =====
  VictimisAlready[x][y] = true;  
  
  switch (receivedData) {

    /* ======================
       H victim
       ====================== */
    case 1:
      Serial.println("H_victim");

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
      Serial.println("S_victim");

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
      Serial.println("red_victim");

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
      Serial.println("yellow_victim");

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
      Serial.print("Unknown cam7 event: ");
      Serial.println(receivedData);
      break;
  }

  drive.stop();
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

void sel8() {//@@@tyousei@@@

  // すでに処理済みなら何もしない
  if (VictimisAlready[x][y]) return;

  // データが無ければ何もしない
  if (!cam8.hasEvent()) return;

  // ===== ヘッダ探索 =====
  byte header = cam8.readEvent();
  if (header != 0xAA) {
    // 同期ずれ → 破棄
    return;
  }

  // ===== 残り2バイト待ち =====
  if (Serial8.available() < 2) {
    // フレーム未完成
    return;
  }

  byte receivedData  = cam8.readEvent();
  byte check = cam8.readEvent();

  // ===== チェックサム検証 =====
  if ((receivedData ^ 0xFF) != check) {
    // 壊れたフレーム
    return;
  }

  // ===== ここから先は「正しい通信」 =====
  VictimisAlready[x][y] = true;
  
     switch (receivedData) {

    /* =========================
       1 : H victim
       ========================= */
    case 1:
      Serial.println("H_victim");

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
      Serial.println("S_victim");

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
      Serial.println("red_victim");

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
      Serial.println("yellow_victim");

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
      break;
  }
}

void clearCams() {
  while (Serial7.available() > 0) {
    Serial7.read();
  }
  while (Serial8.available() > 0) {
    Serial8.read();
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
void migi() {
  Serial.println("turnRight");
  // ===== 回転前の準備 =====
  tiziki_kaitenn(); // 念のため最新値に
  MPU.resetYaw(); // ★ 相対角の基準を 0° に
  delay(50);
  const float TARGET_YAW = 82.0;// 右に90度//@@@tyousei@@@
  sel7();
  sel8();
  
  // ===== 右回転 =====
  while (MPU.getYaw360() < TARGET_YAW || MPU.getYaw360() > 345) {
    drive.turnRight(440);//@@@tyousei@@@
    delay(100);//@@@ // センサ更新
    MPU.update();
    Serial.print("yaw = ");
    Serial.println(MPU.getYaw360());
    delay(50);//@@@
  } 

  clearCams();
  delay(2000);
  sel7();
  sel8();

  // ===== 停止 =====
  delay(200);//@@@
  // tiziki(); // ← 必要ならここで最終補正
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

  Serial.println("turnLeft");

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
    Serial.print("yaw = ");
    Serial.println(MPU.getYaw360());

    delay(50);//@@@
  }

    clearCams();
    delay(2000);
    sel7();
    sel8();

  // ===== 停止 =====
  delay(200);//@@@
  // tiziki(); // ← 必要なら最終補正
}


/*******************************************************************************************/
/*色タイルまたはバンプ                                                                              
/*処理：Serial1にデータが来たら1バイト読み込んで青、黒、ロードセルを検知する
/*Data 1:黒タイル　2：青タイル　3：左の障害物　4：右の障害物　5：前の壁　6：銀のタイル
/*更新者：清田侑希　2025/1/26
/*清田侑希　2025/2/23 変更点：障害物を検知して最初の動作の時に壁との距離を測るようにした
/*清田侑希　2025/3/28 変更点：障害物を検知したとき毎回壁との距離を測るようにした
/*******************************************************************************************/
bool initialized = false;

void s1() {

    /* ======================================
       初回呼び出し時だけの初期化
       ====================================== */
    if (!initialized) {
        while (Serial1.available() > 0) {
            Serial1.read();
        }
        initialized = true;
    }

    /* ======================================
       イベントがなければ何もしない
       ====================================== */
    if (!colorUART.hasEvent()) {
        return;
    }

    /* ======================================
       ヘッダ待ち
       ====================================== */
    byte header = colorUART.readEvent();
    if (header != 0xAA) {
        return;   // ヘッダじゃなければ破棄
    }

    /* ======================================
       ここから「3バイト揃うまで待つ」
       ====================================== */
    unsigned long start = millis();
    while (Serial1.available() < 2) {
        if (millis() - start > 5) {//@@@tyousei@@@
            // 5ms 以内に揃わなければ諦める
            return;
        }
    }

    uint8_t receivedData2 = colorUART.readEvent();
    uint8_t check         = colorUART.readEvent();

    /* ======================================
       チェックサム確認
       ====================================== */
    if ((receivedData2 ^ 0xFF) != check) {
        Serial.println("UART EVENT checksum error");
        return;
    }

    Serial.print("UART EVENT = ");
    Serial.println(receivedData2);

    /* ======================================
       イベント処理（1イベント）
       ====================================== */

    switch (receivedData2) {

        case 1:  // BLACK TILE
            Serial.println("BLACK TILE DETECTED");
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
            Serial.println("BLUE TILE");
            blue_count = true;
            break;

        case 5:  // FRONT BUMP
            Serial.println("FRONT BUMP");

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
            Serial.println("LEFT");
            
            drive.stop(); delay(500);
            
            drive.turnLeft(1023); delay(500); delay(500);
            
            drive.forward(-1240); delay(700); delay(500);
            
            drive.turnRight(1534); delay(700); delay(500);
            
            receivedIndex = 0;
            front_wall = false;
            
            get_tof_data();
            if (receivedData[0] <= 90 && receivedData[3] <= 90 && abs(receivedData[0] - receivedData[3]) < 30) {
              if (count2 < 20) Gap = true; count2 = 46; 
            } 
            
            count2 -= 6;
            bump_giveup_count++;
            test++;      
        break;

        case 4:
            Serial.println("RIGHT");
            
            drive.stop(); delay(500);
            
            drive.turnRight(1023); delay(500); delay(500);
            
            drive.forward(-1240); delay(700); delay(500);
            
            drive.turnLeft(1534); delay(700); delay(500);
            
            receivedIndex = 0;
            front_wall = false;
            
            get_tof_data();
            if (receivedData[0] <= 90 && receivedData[3] <= 90 && abs(receivedData[0] - receivedData[3]) < 30) {
              if (count2 < 20) Gap = true; count2 = 46; 
            } 
            
            count2 -= 6;
            bump_giveup_count++;
            test++;
            break;

        case 6:  // CHECK POINT
            Serial.println("CHECK POINT");

            CheckX = x;
            CheckY = y;
            CheckD = Direction;

            NeoPixel_Rear_ON();
            delay(200);
            NeoPixel_Rear_OFF();
            break;

        default:
            Serial.println("UNKNOWN EVENT");
            break;
    }
}

/* ======================================
   s1 初期化リセット
   ====================================== */
void s1_reset() {
    initialized = false;
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

  s1_reset();


  count2 = 0;

  /* ---------- 姿勢補正（MPU・絶対角） ---------- */
  if (Status == 3) {
    delay(80);
    tiziki();      // ★ ここはそのまま使う
    delay(200);
  }

  /* ---------- 距離による角度補正（実験用・重要） ---------- */
  if (right_wall == true) {

    int diff = receivedData[4] - receivedData[5];  // 左右差
    Serial.print("ToF diff = ");
    Serial.println(diff);

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
  drive.forward(BIAS_PWR);
  delay(BIAS_TIME);
  
  delay(50);

  /* ---------- 本前進（1タイル） ---------- */
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
    delay(5500);
  }

  /* ---------- 後処理 ---------- */
  count2 = 0;
  bump_giveup_count = 0;
  Status = 0;
  blue_count = false;

  drive.stop();
  delay(200);
}

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
