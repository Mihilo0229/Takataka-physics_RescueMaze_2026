/*ここにある関数
judge
MoveTo
TileStatus
WhichWay
*/


/*******************************************************************************************/
/*行き止まりの検索                                                                              
/*処理：全てのマスについて、
/*      前後左右の三か所以上が行き止まりor壁のときそのマスの到達回数を+100する（マスの到達回数が100未満のとき）
/*
/*更新者：吉ノ薗2025/02/02いったん凍結
/*
/*******************************************************************************************/
/*void EffectiveDeadEnd(){

    int NorthReached = 0;
    int EastReached = 0;
    int WestReached = 0;
    int SouthReached = 0;

    for (int t = 1; t < 89; t++) {
        for (int j = 1; j < 89; j++) {

            int DeadEndCount = 0;

            NorthReached = toutatu_zahyou[t][j-1];
            EastReached  = toutatu_zahyou[t+1][j];
            WestReached  = toutatu_zahyou[t-1][j];
            SouthReached = toutatu_zahyou[t][j+1];
            
            if(toutatu_zahyou[t][j] < 100){

                if((NorthReached >= 100) || (kabe_zahyou[t][j] & 1)){DeadEndCount++;}//北に壁がある
                if((SouthReached >= 100) || (kabe_zahyou[t][j] & 2)){DeadEndCount++;}//南に壁がある
                if((WestReached >= 100) || (kabe_zahyou[t][j] & 4)){DeadEndCount++;}//西に壁がある
                if((EastReached >= 100) || (kabe_zahyou[t][j] & 8)){DeadEndCount++;}//東に壁がある

                //前後左右の三か所以上が行き止まりor壁のとき、そのマスの到達回数を+100する
                if(DeadEndCount >= 3){
                    toutatu_zahyou[t][j] += 100;
                }

            }
        }
    }
}*/

/*******************************************************************************************/
/* ジャッジ                                                                            
/*処理：前左右を重みづけして、行く方向を決定
/*      右+1、前+2、左+3することで基本的に右に行く拡張右手法を実現
/*      壁があったらさらにプラスする
/*
/*更新者：吉ノ薗2025/01/22
/*       清田侑希　2025/1/30    座標などのデータ更新
/*******************************************************************************************/


int8_t judge(){
    
    switch (Direction){//向きによって重みをつける
        case East:
            RightWeight = toutatu_zahyou[x][y+1] * 5 + 1;//@@@tyousei@@@ hosoku: *5 tokano suuji wo hennkou suru noga iiyo
            FrontWeight = toutatu_zahyou[x+1][y] * 5 + 2;
            LeftWeight = toutatu_zahyou[x][y-1] * 5 + 3;
            break;

        case North:
            RightWeight = toutatu_zahyou[x+1][y] * 5 + 1;
            FrontWeight = toutatu_zahyou[x][y-1] * 5 + 2;
            LeftWeight = toutatu_zahyou[x-1][y] * 5 + 3;
            break;

        case West:
            RightWeight = toutatu_zahyou[x][y-1] * 5 + 1;
            FrontWeight = toutatu_zahyou[x-1][y] * 5 + 2;
            LeftWeight = toutatu_zahyou[x][y+1] * 5 + 3;
            break;

        case South:
            RightWeight = toutatu_zahyou[x-1][y] * 5 + 1;
            FrontWeight = toutatu_zahyou[x][y+1] * 5 + 2;
            LeftWeight = toutatu_zahyou[x+1][y] * 5 + 3;
            break;

    }

    //壁がある場合１００にする
    if (right_wall){
        RightWeight = 100;
    }
    if (front_wall){
        FrontWeight = 100;
    }
    if (left_wall){
        LeftWeight = 100;
    }

    int8_t GoTo = 0;

    //どこへ行くか決める
    if(RightWeight < FrontWeight){
        GoTo = Right;
    }else{
        GoTo = Front;
    }
    if(LeftWeight < min(RightWeight,FrontWeight)){
        GoTo = Left;
    }

    if(toutatu_zahyou[x][y] > 100){toutatu_zahyou[x][y] = 100;}//捕捉：走行中に右の重みが94っていう出るはずのない値がでたためオーバーフローを疑いこの関数を導入。対処療法であるため根本的な解決には至っていない

    if(FrontWeight > 100){FrontWeight = 100;}//前・左・右の重みが100を越えていたら100にする
    if(RightWeight > 100){RightWeight = 100;}
    if(LeftWeight > 100){LeftWeight = 100;}

    if ((RightWeight == 100) && (FrontWeight == 100) && (LeftWeight == 100)){//if all wall
        GoTo = Back;
        toutatu_zahyou[x][y] = 50;//行き止まりだから効率化のため二度と行かないようにする
    }

    //send_display();
    RightWeight = 0;//怖いから初期化
    FrontWeight = 0;
    LeftWeight = 0;
    return GoTo;
}


/*******************************************************************************************/
/* 動く方向                                                                              
/*処理：それぞれの方位のときに行く方向によって座標を移動・方位を変更する
/*      後進のときは全部壁のときだから、現在の重みを+100する
/*
/*更新者：吉ノ薗2025/01/22
/*
/*******************************************************************************************/
void MoveTo(int8_t GoTo)
{
    //EffectiveDeadEnd();//より効率的な行き止まりの検索

    switch (Direction){
            case East:
                switch (GoTo){
                    case Right:
                        Status = 4;
                        y += 1;
                        Direction = South;
                        break;
                    
                    case Front:
                        Status = 3;
                        x += 1;
                        Direction = East;
                        break;
                    
                    case Left:
                        Status = 5;
                        y += -1;
                        Direction = North;
                        break;

                    case Back:
                        Status = 6;
                        x += -1;
                        Direction = West;
                        break;
                    }
                break;
            
            case North:
                switch (GoTo){
                    case Right:
                        Status = 4;
                        x += 1;
                        Direction = East;
                        break;
                    
                    case Front:
                        Status = 3;
                        y += -1;
                        Direction = North;
                        break;
                    
                    case Left:
                        Status = 5;
                        x += -1;
                        Direction = West;
                        break;

                    case Back:
                        Status = 6;
                        y += 1;
                        Direction = South;
                        break;
                    }
                break;
            
            case West:
                switch (GoTo){
                    case Right:
                        Status = 4;
                        y += -1;
                        Direction = North;
                        break;
                    
                    case Front:
                        Status = 3;
                        x += -1;
                        Direction = West;
                        break;
                    
                    case Left:
                        Status = 5;
                        y += 1;
                        Direction = South;
                        break;
                    
                    case Back:
                        Status = 6;
                        x += 1;
                        Direction = East;
                        
                        break;
                    }
                break;
            
            case South:
                switch (GoTo){
                    case Right:
                        Status = 4;
                        x += -1;
                        Direction = West;
                        break;
                    
                    case Front:
                        Status = 3;
                        y += 1;
                        Direction = South;
                        break;
                    
                    case Left:
                        Status = 5;
                        x += 1;
                        Direction = East;
                        break;
                    
                    case Back:
                        Status = 6;
                        y += -1;
                        Direction = North;
                        break;
                    }
                break;
        }
    toutatu_zahyou[x][y] += 1;//移動先のマスの到達回数をプラスしている
}



/*******************************************************************************************/
/* タイルの状態                                                                              
/*処理：それぞれの方向について、
/*      黒タイルの場合そのマスの重みを100にする
/*      黒タイルまたは"少しずれている"の信号が送られていたら一マス戻る
/*      坂の信号が送られていたら一マス進む
/*      黒タイルと坂信号の初期化
/*
/*更新者：吉ノ薗2025/01/22
/*
/*******************************************************************************************/
void TileStatus()
{
    if(BlackTile){
        toutatu_zahyou[x][y] += 100;
    }

    switch (Direction)
    {
    case North:
        if(BlackTile || Gap){//黒タイルまたはずれの信号が送られていた場合戻る
            y += 1;
        }
        if(Slope){//坂の信号が送られていた場合座標を更に進める
            y += -1;
        }
        break;

    case East:
        if(BlackTile || Gap){//黒タイルの信号が送られていた場合戻る
            x += -1;
        }
        if(Slope){//坂の信号が送られていた場合座標を更に進める
            x += 1;
        }
        break;
    
    case West:
        if(BlackTile || Gap){//黒タイルの信号が送られていた場合戻る
            x += 1;
        }
        if(Slope){//坂の信号が送られていた場合座標を更に進める
            x += -1;
        }
        break;

    case South:
        if(BlackTile || Gap){//黒タイルの信号が送られていた場合戻る

            y += -1;
        }
        if(Slope){//坂の信号が送られていた場合座標を更に進める
            y += 1;
        }
        break;
    
    
    }
    BlackTile = false;
    Slope = false;
    Gap = false;
}


  


/*******************************************************************************************/
/* 最短経路の方位                                                                              
/*処理：現在のマスのコストの、-1のコストのマスを探す
/*更新者：吉ノ薗2025/01/22
/*       吉ノ薗2025/03/28　差が１でも壁があれば行かないようにした
/*
/*******************************************************************************************/
int WhichWay(uint8_t a,uint8_t b)
{
    if ((cost[a][b] - cost[a + 1][b] == 1) && !(kabe_zahyou[a][b] & 8)) {
        return East;
    }
    if ((cost[a][b] - cost[a][b - 1] == 1) && !(kabe_zahyou[a][b] & 1)) {
        return North;
    }
    if ((cost[a][b] - cost[a - 1][b] == 1) && !(kabe_zahyou[a][b] & 4)) {
        return West;
    }
    if ((cost[a][b] - cost[a][b + 1] == 1) && !(kabe_zahyou[a][b] & 2)) {
        return South;
    }
    Serial.println("Error@whichway");
    return 0;
}
