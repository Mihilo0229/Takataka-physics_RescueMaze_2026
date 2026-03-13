
#define DEBUG 1//on->1 off->0

#if DEBUG

void debug_commands(){
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

}

#endif