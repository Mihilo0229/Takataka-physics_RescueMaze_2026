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
    #if DEBUGMODE
        Serial.println("`----spin() Finished----`");
    #endif
}