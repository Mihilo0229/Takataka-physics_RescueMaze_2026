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