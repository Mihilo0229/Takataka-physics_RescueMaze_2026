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