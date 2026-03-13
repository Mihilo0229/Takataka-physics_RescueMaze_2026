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