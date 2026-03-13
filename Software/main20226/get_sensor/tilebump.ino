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