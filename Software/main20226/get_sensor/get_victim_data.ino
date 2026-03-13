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