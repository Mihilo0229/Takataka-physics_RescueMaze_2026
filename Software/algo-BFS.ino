/*ここにある関数
BFS
WriteDownWall
GoHome
ForBFSLedtGo
*/


/*******************************************************************************************/
/* 幅優先探索                                                                              
/*処理：現在のマスから空いている隣接マスのコストを、現在のマスのコスト+1していくのを繰り返す
/*      kabe_zahyou[][]に0000 の4ビットに絶対方向の東8西4南2北1をそれぞれ割り当てているので、割っていって余り0のときその先の壁が無いと判定する
/*      ゴールにたどり着いたらゴールから逆に辿っていきつつ右折、左折、直進を逆からスタックに入れていく
/*
/*
/*
/*更新者：吉ノ薗2025/01/22
/*　　　　吉ノ薗2025/02/01　変更点：スタックとキューをライブラリにして座標の計算をビット演算に変更
/*       吉ノ薗2025/03/26 delay(300)を削除。遅かったのお前が原因だろ
/*       吉ノ薗2025/03/28 座標記入漏れ・ミスを修正
/*       吉ノ薗2025/03/28 逆探索終了時の向きと実際の向きを合わせるように修正
/*
/*******************************************************************************************/
void BFS()
{
    //hidari();//デバッグ用
    uint8_t a = x;
    uint8_t b = y;
    cost[a][b] = 1;//現在地のコストを1にする
    Serial.println("GotoHome:");

    while(!(a == GoalX && b == GoalY)){//ここ詰まったら中断するようにしたかった

        reach_time[a][b] = 1;//そのマスを訪問済みにする

        Serial.print("a =");
        Serial.println(a);
        Serial.print("b =");
        Serial.println(b);

        Serial.print("kabe_zahyou[a][b] ==");
        Serial.println(kabe_zahyou[a][b]);

        Serial.print("cost[a][b] == ");
        Serial.println(cost[a][b]);
        
        for(int n = 1; n <= 8; n *= 2){//そのマスの周りのマスのコストを＋１する
            //int result = static_cast<int>(pow(2, n));
            Serial.print("n =");
            Serial.print(n);

            if(!(kabe_zahyou[a][b] & n)) {//kabe_zahyou[][]は0000 の4ビットに絶対方向の東8西4南2北1をそれぞれ割り当てる

                switch(n){
                    case 1://北の壁がない
                        if((!reach_time[a][b-1]) && !(kabe_zahyou[a][b-1] & 16)){//その先のマスが訪問済みでない&その先のマスが探索済み
                            cost[a][b-1] = cost[a][b] + 1;
                            //キューの末尾に入れる
                            Q.push(a);
                            Q.push(b-1);
                        }
                        break;

                    case 2://南の壁がない
                        if((!reach_time[a][b+1]) && !(kabe_zahyou[a][b+1] & 16)){//その先のマスが訪問済みでない&その先のマスが探索済み
                            cost[a][b+1] = cost[a][b] + 1;
                            //キューの末尾に入れる
                            Q.push(a);
                            Q.push(b+1);
                        }
                        break;

                    case 4://西の壁がない
                        if((!reach_time[a-1][b]) && !(kabe_zahyou[a-1][b] & 16)){//その先のマスが訪問済みでない&その先のマスが探索済み
                            cost[a-1][b] = cost[a][b] + 1;
                            //キューの末尾に入れる
                            Q.push(a-1);
                            Q.push(b);
                        }
                        break;

                    case 8://東の壁がない
                        if((!reach_time[a+1][b]) && !(kabe_zahyou[a+1][b] & 16)) {//その先のマスが訪問済みでない&その先のマスが探索済み
                            cost[a+1][b] = cost[a][b] + 1;
                            //キューの末尾に入れる
                            Q.push(a+1);
                            Q.push(b);
                        }
                        break;
                }
            }
        }
        //キューの先頭を取り出す
        if (Q.size() < 2) break;  // キューの要素が足りない場合は終了

        a = Q.front(); Q.pop();
        b = Q.front(); Q.pop();
    }


    //スタックを使って逆探索
    a = GoalX;
    b = GoalY;
    NowDirection = Direction;
    Direction = North;

    S.push(4);//停止用


    while(!((a == x) && (b == y))){
        switch(Direction){
            case East:
                switch(WhichWay(a,b)){//前後左右のどこが最短になるか

                    case North://北マスからきたとき（ここのシグナルは探索時の曲がる→進むとは逆で、進む→曲がるじゃないと。）
                        S.push(2);//１：右折、２：左折、３：直進
                        S.push(3);
                        b += -1;
                        Direction = South;
                        break;
                    case West://西マスからきたとき
                        S.push(3);
                        a += -1;
                        break;

                    case South://南マスからきたとき
                        S.push(1);
                        S.push(3);
                        b += 1;
                        Direction = North;
                        break;
                    case East:
                        Direction =North;
                        break;

                }
                break;

            case North:
                switch(WhichWay(a,b)){//前後左右のどこが最短になるか
                    case East:
                        S.push(1);
                        S.push(3);
                        a += 1;
                        Direction = West;
                        break;

                    case West:
                        S.push(2);
                        S.push(3);
                        a += -1;
                        Direction = East;
                        break;

                    case South:
                        S.push(3);
                        b += 1;
                        break;

                    case North:
                        S.push(2);
                        S.push(2);
                        S.push(3);
                        b += -1;
                        Direction = South;
                        //Direction = East;
                        break;

                }
                break;

            case West:
                switch(WhichWay(a,b)){//前後左右のどこが最短になるか
                    case East:
                        S.push(3);
                        a += 1;
                        break;

                    case North:
                        S.push(1);
                        S.push(3);
                        b += -1;
                        Direction = South;
                        break;

                    case South:
                        S.push(2);
                        S.push(3);
                        b += 1;//ここが原因
                        Direction = North;
                        break;

                    case West:
                        Direction =North;
                        break;

                }
                break;

            case South:
                switch(WhichWay(a,b)){//前後左右のどこが最短になるか
                    case East:
                        S.push(2);
                        S.push(3);
                        a += 1;
                        Direction = West;
                        break;

                    case North:
                        S.push(3);
                        b += -1;
                        break;

                    case West:
                        S.push(1);
                        S.push(3);
                        a += -1;
                        Direction = East;
                        break;

                    case South:
                        Direction = North;
                        break;

                }
                break;

            /*デバッグ用*/
            default:
                Serial.println("Error");
                break;
        }
    }

    switch (NowDirection)//今の向きを逆探索終了時の向きに変更する
    {
    case North:
        switch (Direction)
        {
        case East:
            S.push(1);
            break;

        case South:
            S.push(1);
            S.push(1);
            break;

        case West:
            S.push(2);
            break;
        }
        break;
    
    case East:
        switch (Direction)
        {
        case North:
            S.push(2);
            break;

        case South:
            S.push(1);
            break;

        case West:
            S.push(1);
            S.push(1);
            break;
        }
        break;

    case South:
        switch (Direction)
        {
        case North:
            S.push(1);
            S.push(1);
            break;
        
        case East:
            S.push(2);
            break;

        case West:
            S.push(1);
            break;
        }
        break;

    case West:
        switch (Direction)
        {
        case North:
            S.push(1);
            break;
        
        case East:
            S.push(1);
            S.push(1);
            break;

        case South:
            S.push(2);
            break;
        }
        break;
    }
}



/*******************************************************************************************/
/* 現在のマスの壁情報を記入                                                                              
/*処理：kabe_zahyou[][]は情報未記入の場合100にしてあるので、100とき記録する
/*    kabe_zahyou[][]に0000 の4ビットに絶対方向の東8西4南2北1をそれぞれ割り当てて、値を加算
/*    kabe_zahyou[][]を-100して記録済みに
/*
/*更新者：吉ノ薗2025/01/22
/*
/*******************************************************************************************/
void WriteDownWall(uint8_t x, uint8_t y,uint8_t Direction)
{
    //壁情報の記入(ここは帰還アルゴリズム用の関数)
    if(kabe_zahyou[x][y] == 16){//記録されていない場合（そうしないと延々と加算されちゃう）
        kabe_zahyou[x][y] = 0;//4ビットの情報のみが残る
        switch (Direction){
            case East:
                if(right_wall){
                    kabe_zahyou[x][y] += 2;
                }
                if(front_wall){
                    kabe_zahyou[x][y] += 8;
                }
                if(left_wall){
                    kabe_zahyou[x][y] += 1;
                }
                break;
            
            case North:
                if(right_wall){
                    kabe_zahyou[x][y] += 8;
                }
                if(front_wall){
                    kabe_zahyou[x][y] += 1;
                }
                if(left_wall){
                    kabe_zahyou[x][y] += 4;
                }
                break;

            case West:
                if(right_wall){
                    kabe_zahyou[x][y] += 1;
                }
                if(front_wall){
                    kabe_zahyou[x][y] += 4;
                }
                if(left_wall){
                    kabe_zahyou[x][y] += 2;
                }
                break;

            case South:
                if(right_wall){
                    kabe_zahyou[x][y] += 4;
                }
                if(front_wall){
                    kabe_zahyou[x][y] += 2;
                }
                if(left_wall){
                    kabe_zahyou[x][y] += 8;
                }
                break;
            }
            //対応
            //東西南北
            //8 4 2 1
        
    }
}

/*******************************************************************************************/
/* BFSで使う左手法                                                                            
/*処理：今いるマスの壁情報をBFSWallZahyouに記入（スタート時の南方向は今回の場合わからないので記入しない）
/*　　　ゴールマスの壁情報のうち、現在の向きの後ろ側になる壁の情報をなくす
/*    　スタート時のマスの壁情報と照合
/*
/*更新者：吉ノ薗2025/03/28
/*
/*******************************************************************************************/
void ForBFSLeftGo(){
    //WriteDownWall
    int BFSWallZahyou = 0;
    //壁情報の記入(ここは帰還アルゴリズム用の関数)
    switch (Direction){
    case East:
      if(right_wall){
        //BFSWallZahyou |= 2;
      }
      if(front_wall){
        BFSWallZahyou += 8;
      }
      if(left_wall){
        BFSWallZahyou += 1;
      }
      break;
            
    case North:
      if(right_wall){
        BFSWallZahyou += 8;
      }
      if(front_wall){
        BFSWallZahyou += 1;
      }
      if(left_wall){
        BFSWallZahyou += 4;
      }
      break;

    case West:
      if(right_wall){
        BFSWallZahyou += 1;
      }
      if(front_wall){
        BFSWallZahyou += 4;
      }
      if(left_wall){
        //BFSWallZahyou |= 2;
      }
      break;

    case South:
      if(right_wall){
        BFSWallZahyou += 4;
      }
      if(front_wall){
        //BFSWallZahyou |= 2;
      }
      if(left_wall){
        BFSWallZahyou += 8;
      }
      break;
    }
    //南の壁情報をなくす
    BFSWallZahyou &= ~2;

    //ゴールマスの壁情報のうち、現在の向きの後ろ側になる壁の情報をなくす
    switch (Direction)
    {
    case North:
      kabe_zahyou[GoalX][GoalY] &= ~2;
      break;
    
    case East:
      kabe_zahyou[GoalX][GoalY] &= ~4;
      break;

    case South:
      kabe_zahyou[GoalX][GoalY] &= ~1;
      break;

    case West:
      kabe_zahyou[GoalX][GoalY] &= ~8;
      break;
    }
    if(BFSWallZahyou == kabe_zahyou[GoalX][GoalY]){////スタート時のマスの壁情報と合ってる場合停止フラグを立てる
      StopFlag = true;
      return;
    }
    
    //左手法
    uint8_t GoTo = 0;
    if     (!left_wall) {GoTo = Left ;}
    else if(!front_wall){GoTo = Front;}
    else if(!right_wall){GoTo = Right;}
    else                {GoTo = Back ;}

    switch (GoTo)
    {
    case Right:
      Status = 4;//右折
      break;

    case Front:
      Status = 2;//直進
      break;

    case Left:
      Status = 3;//左折
      break;

    case Back:
      Status = 5;//後進
      break;
      }
      
}

/*******************************************************************************************/
/* 帰還                                                                             
/*処理：スタックから値をpopして順番に動いていく
/*      全て終わる、つまりpopした値が一番最初に入れた"4"になっていたら停止（まだ停止部分はつくってない）
/*
/*更新者：吉ノ薗2025/01/22
/*　　　　吉ノ薗2025/01/29：20秒停止するようにした
/*       吉ノ薗2025/03/26 whileの条件を「スタックが空でなかった場合」に変更
/*       吉ノ薗2025/03/28 座標がずれても戻れるように南以外の壁座標が間違っていた場合（スタート時Northで南がわからないため）左手法を走行するようにした
/*
/*******************************************************************************************/
void GoHome()
{
    Direction = NowDirection;
    while(!S.empty()){
        //send_display();
        switch(S.top()){
            case 1:
                //TurnRight
                migi();
                switch (Direction)
                {
                case North:
                  Direction = East;
                  break;
                
                case East:
                  Direction = South;
                  break;

                case South:
                  Direction = West;
                  break;

                case West:
                  Direction = North;
                  break;
                }
                delay(500);
                break;

            case 2:
                //TurnLeft
                hidari();
                switch (Direction)
                {
                case North:
                  Direction = West;
                  break;
                
                case East:
                  Direction = North;
                  break;

                case South:
                  Direction = East;
                  break;

                case West:
                  Direction = South;
                  break;
                }
                delay(500);
                break;

            case 3:
                //GoStraight
                susumu_heitan();
                switch (Direction)
                {
                case North:
                  y += -1;
                  break;
                
                case East:
                  x += 1;
                  break;

                case South:
                  y += 1;
                  break;

                case West:
                  x += -1;
                  break;
                }
                delay(200);
                break;
            case 4:
                //Stop
                NeoPixel_Color(0,0,255);//うろうろさせない場合このコードで停止
                delay(20000);

                /*壁情報取得してうろうろさせる*/
                while(!StopFlag)
                {
                  switch (Status)
                  {
                  case 0:
                    get_tof_data();
                    break;
                  
                  case 1:
                    //WriteDownWall
                    ForBFSLeftGo();
                    break;

                  case 2://直進
                    susumu_heitan();
                    switch (Direction)
                    {
                    case North:
                      y += -1;
                      break;
                    
                    case East:
                      x += 1;
                      break;
    
                    case South:
                      y += 1;
                      break;
    
                    case West:
                      x += -1;
                      break;
                    }
                    delay(200);
                    TileStatus();
                    break;
                  
                  case 3://左折
                    hidari();
                    switch (Direction)
                    {
                    case North:
                      Direction = West;
                      break;
                    
                    case East:
                      Direction = North;
                      break;
    
                    case South:
                      Direction = East;
                      break;
    
                    case West:
                      Direction = South;
                      break;
                    }
                    delay(500);
                    susumu_heitan();
                    switch (Direction)
                    {
                    case North:
                      y += -1;
                      break;
                    
                    case East:
                      x += 1;
                      break;
    
                    case South:
                      y += 1;//
                      break;
    
                    case West:
                      x += -1;
                      break;
                    }
                    delay(200);
                    TileStatus();
                    break;
                  
                  case 4://右折
                    migi();
                    switch (Direction)
                    {
                    case North:
                      Direction = East;
                      break;
                    
                    case East:
                      Direction = South;
                      break;
    
                    case South:
                      Direction = West;
                      break;
    
                    case West:
                      Direction = North;
                      break;
                    }
                    delay(500);
                    susumu_heitan();
                    switch (Direction)
                    {
                    case North:
                      y += -1;
                      break;
                    
                    case East:
                      x += 1;
                      break;
    
                    case South:
                      y += 1;
                      break;
    
                    case West:
                      x += -1;
                      break;
                    }
                    delay(200);
                    TileStatus();
                    break;
                  
                  case 5://後進
                    hidari();
                    delay(500);
                    hidari();
                    switch (Direction)
                    {
                    case North:
                      Direction = South;
                      break;
                    
                    case East:
                      Direction = West;
                      break;
    
                    case South:
                      Direction = North;
                      break;
    
                    case West:
                      Direction = East;
                      break;
                    }
                    delay(500);
                    susumu_heitan();
                    switch (Direction)
                    {
                    case North:
                      y += -1;
                      break;
                    
                    case East:
                      x += 1;
                      break;
    
                    case South:
                      y += 1;
                      break;
    
                    case West:
                      x += -1;
                      break;
                    }
                    delay(200);
                    TileStatus();
                    break;
                  }

                }
                NeoPixel_Color(0,0,255);    
                delay(20000);               
                break;
        }
        S.pop();//要素の削除
    }
    
}

