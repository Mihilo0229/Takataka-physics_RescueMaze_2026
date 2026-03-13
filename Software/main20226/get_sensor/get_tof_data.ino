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