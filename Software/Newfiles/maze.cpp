#include "Arduino lib/Ard-core.h"
#include "Arduino lib/Ard-Serial.h"

//written by fukuda ryouta
#include <queue>
#include <random>

#define right 2
#define forward 3
#define left 1
#define back 0

#define North 0
#define East 1
#define South 2
#define West 3
#define Err 4


const int maxX = 5;//横軸最大値
const int maxY = 5;//縦軸最大値

struct wallDirection{
  uint8_t N = 0b1000;
  uint8_t E = 0b0100;
  uint8_t S = 0b0010;
  uint8_t W = 0b0001;
};

struct NodeD{
  int x;
  int y;
  int dir;
};
struct Wall{
  bool rightW;
  bool forwardW;
  bool leftW;
  int dir;
};

wallDirection wallD;

std::queue<NodeD> Map_Q;
std::queue<int> way_q;

int went[maxX][maxY];

int cost[maxX][maxY];

uint8_t map_walls[maxX][maxY];

Wall  walls[maxX][maxY];

uint8_t Maps[maxX][maxY];


void addWalls(uint8_t &here, uint8_t add){
    here = here - add;
}

void carve(int x,int y,int maxX,int maxY,uint8_t maps[5][5],bool visited[5][5]){

    int dx[4] = {0,1,0,-1};
    int dy[4] = {1,0,-1,0};

    visited[x][y] = true;

    int dirs[4] = {0,1,2,3};

    //方向をシャッフル
    for(int i=0;i<4;i++){
        int r = rand()%4;
        int t = dirs[i];
        dirs[i] = dirs[r];
        dirs[r] = t;
    }

    for(int i=0;i<4;i++){

        int d = dirs[i];

        int nx = x + dx[d];
        int ny = y + dy[d];

        if(nx<0 || ny<0 || nx>=maxX || ny>=maxY) continue;

        if(visited[nx][ny]) continue;

        //壁を壊す
        switch(d){

            case North:
                addWalls(maps[x][y], wallD.N);
                addWalls(maps[nx][ny], wallD.S);
                break;

            case East:
                addWalls(maps[x][y], wallD.E);
                addWalls(maps[nx][ny], wallD.W);
                break;

            case South:
                addWalls(maps[x][y], wallD.S);
                addWalls(maps[nx][ny], wallD.N);
                break;

            case West:
                addWalls(maps[x][y], wallD.W);
                addWalls(maps[nx][ny], wallD.E);
                break;
        }

        carve(nx,ny,maxX,maxY,maps,visited);
    }
}

void makeMaze(int maxX, int maxY, uint8_t maps[5][5]){
    for(int i = 0; i < maxX; i++){
        for(int j = 0; j < maxY; j++){
            maps[i][j] = 0b1111;
        }
    }

    bool visited[5][5] = {};
    
    carve(0,0,maxX,maxY,maps,visited);
}


void showMap(int x, int y, uint8_t wallmap[maxX][maxY], NodeD here){

  for(int i = y-1; i >= 0; i--){
    for(int j = 0; j < x; j++){
      if(wallmap[j][i] & wallD.N){
        Serial.print("##");
        Serial.print("##");
      }
      else{
        Serial.print("##");
        Serial.print("  ");
      }
      
    }
      

    Serial.println("##");
    
    for(int j = 0; j < x; j++){
      if(wallmap[j][i] & wallD.W){
        Serial.print("##");
      }
      else{
        Serial.print("  ");
      }

      if(here.x == j && here.y == i){
        switch(here.dir){
          case North:
            Serial.print("^^");
            break;
          case East:
            Serial.print(">>");
            break;
          case South:
            Serial.print("vv");
            break;
          case West:
            Serial.print("<<");
            break;
        }
      }
      else Serial.print("  ");
      
    }
      
    
    if(wallmap[x-1][i] & wallD.E) Serial.println("##");
    else Serial.println("  ");
  }
  for(int j = 0; j < x; j++){
    if(wallmap[j][0] & wallD.S){
      Serial.print("##");
      Serial.print("##");
    }
    else{
      Serial.print("##");
      Serial.print("  ");
    }
  }
  Serial.println("##");

}

void showCostMap(int x, int y, uint8_t wallmap[maxX][maxY], int cost[maxX][maxY]){

  for(int i = y-1; i >= 0; i--){
    for(int j = 0; j < x; j++){
      if(wallmap[j][i] & wallD.N){
        Serial.print("##");
        Serial.print("##");
      }
      else{
        Serial.print("##");
        Serial.print("  ");
      }
      
    }
      

    Serial.println("##");
    
    for(int j = 0; j < x; j++){
      if(wallmap[j][i] & wallD.W){
        Serial.print("##");
      }
      else{
        Serial.print("  ");
      }

      if(cost[j][i] < 10){
        Serial.print("0"); Serial.print(cost[j][i]);
      }
      else{
        Serial.print(cost[j][i]);
      }
    }
      
    
    if(wallmap[x-1][i] & wallD.E) Serial.println("##");
    else Serial.println("  ");
  }
  for(int j = 0; j < x; j++){
    if(wallmap[j][0] & wallD.S){
      Serial.print("##");
      Serial.print("##");
    }
    else{
      Serial.print("##");
      Serial.print("  ");
    }
  }
  Serial.println("##");

}



int clearD(int rawD){
  int ans = (rawD % 4 + 4) % 4;
  return ans;
}

int RHR_whichWay(Wall here, NodeD hereD){
  int rwall=0, fwall=0, lwall=0;

  if(here.rightW) rwall = 1;
  if(here.forwardW) fwall = 1;
  if(here.leftW) lwall = 1;
  
  int right_waight, forward_waight, left_waight, back_waight;
  
  right_waight = min(here.rightW * 100 + went[hereD.x][hereD.y] * 5 + 1,100);
  forward_waight = min(here.forwardW * 100 + went[hereD.x][hereD.y] * 5 + 2,100);
  left_waight = min(here.leftW * 100 + went[hereD.x][hereD.y] * 5 + 3,100);
  back_waight = 399 - right_waight - forward_waight - left_waight;

//  Serial.print(right_waight);Serial.print(" - ");
//  Serial.print(forward_waight);Serial.print(" - ");
//  Serial.print(left_waight);Serial.print(" - ");
//  Serial.println(back_waight);

  if (right_waight < min(forward_waight,left_waight)) return right;
  else if (forward_waight < left_waight) return forward;
  else if (left_waight < back_waight) return left;
  else return back;
}

NodeD RHR_nextPosition(NodeD here,int next){
  if (here.dir == North){
    if(next == right) return {here.x+1,here.y,clearD(here.dir + 1)};
    else if(next == forward) return {here.x,here.y+1,clearD(here.dir + 0)};
    else if(next == left) return {here.x-1,here.y,clearD(here.dir - 1)};
    else if(next == back) return {here.x,here.y-1,clearD(here.dir - 2)};
  }
  else if (here.dir == East){
    if(next == right) return {here.x,here.y-1,clearD(here.dir + 1)};
    else if(next == forward) return {here.x+1,here.y,clearD(here.dir + 0)};
    else if(next == left) return {here.x,here.y+1,clearD(here.dir - 1)};
    else if(next == back) return {here.x-1,here.y,clearD(here.dir - 2)};
  }
  else if (here.dir == South){
    if(next == right) return {here.x-1,here.y,clearD(here.dir + 1)};
    else if(next == forward) return {here.x,here.y-1,clearD(here.dir + 0)};
    else if(next == left) return {here.x+1,here.y,clearD(here.dir - 1)};
    else if(next == back) return {here.x,here.y+1,clearD(here.dir - 2)};
  }
  else if (here.dir == West){
    if(next == right) return {here.x,here.y+1,clearD(here.dir + 1)};
    else if(next == forward) return {here.x-1,here.y,clearD(here.dir + 0)};
    else if(next == left) return {here.x,here.y-1,clearD(here.dir - 1)};
    else if(next == back) return {here.x+1,here.y,clearD(here.dir - 2)};
  }
  return {0,0,North};
}

void RHR_inputWalls(Wall &here, NodeD hereD){

  //koko ni kabe jouhou wo ireru ko-do kaku

  int rightWall, forwardWall, leftWall;
  switch(hereD.dir){
    case North:
      rightWall = map_walls[hereD.x][hereD.y] & wallD.E;
      forwardWall = map_walls[hereD.x][hereD.y] & wallD.N;
      leftWall = map_walls[hereD.x][hereD.y] & wallD.W;
      break;
    case East:
      rightWall = map_walls[hereD.x][hereD.y] & wallD.S;
      forwardWall = map_walls[hereD.x][hereD.y] & wallD.E;
      leftWall = map_walls[hereD.x][hereD.y] & wallD.N;
      break;
    case South:
      rightWall = map_walls[hereD.x][hereD.y] & wallD.W;
      forwardWall = map_walls[hereD.x][hereD.y] & wallD.S;
      leftWall = map_walls[hereD.x][hereD.y] & wallD.E;
      break;
    case West:
      rightWall = map_walls[hereD.x][hereD.y] & wallD.N;
      forwardWall = map_walls[hereD.x][hereD.y] & wallD.W;
      leftWall = map_walls[hereD.x][hereD.y] & wallD.S;
      break;
  }

  if(rightWall != 0) here.rightW = true;
  else here.rightW = false;
  if(forwardWall != 0) here.forwardW = true;
  else here.forwardW = false;
  if(leftWall != 0) here.leftW = true;
  else here.leftW = false;

  here.dir = hereD.dir;
}

void RHR_inputMaps(Wall here,NodeD hereD){
  uint8_t ans = 0;
  switch(hereD.dir){
    case North:
      if(here.rightW) ans = ans + wallD.E;
      if(here.forwardW) ans = ans + wallD.N;
      if(here.leftW) ans = ans + wallD.W;
      Maps[hereD.x][hereD.y] = ans;
      break;
    case East:
      if(here.rightW) ans = ans + wallD.S;
      if(here.forwardW) ans = ans + wallD.E;
      if(here.leftW) ans = ans + wallD.N;
      Maps[hereD.x][hereD.y] = ans;
      break;
    case South:
      if(here.rightW) ans = ans + wallD.W;
      if(here.forwardW) ans = ans + wallD.S;
      if(here.leftW) ans = ans + wallD.E;
      Maps[hereD.x][hereD.y] = ans;
      break;
    case West:
      if(here.rightW) ans = ans + wallD.N;
      if(here.forwardW) ans = ans + wallD.W;
      if(here.leftW) ans = ans + wallD.S;
      Maps[hereD.x][hereD.y] = ans;
      break;
  }
}


int BFS_whichWay(NodeD here){
  if(here.x + 1 < maxX//エリア内
     && cost[here.x+1][here.y] == cost[here.x][here.y]-1//コストが一つ低い方へ
     && !(Maps[here.x][here.y] & wallD.E)) return East;//壁が無かったら
  else if(here.y + 1 < maxY
     && cost[here.x][here.y+1] == cost[here.x][here.y]-1
     && !(Maps[here.x][here.y] & wallD.N)) return North;
  else if(here.x - 1 >= 0
     && cost[here.x-1][here.y] == cost[here.x][here.y]-1
     && !(Maps[here.x][here.y] & wallD.W)) return West;
  else if(here.y - 1 >= 0
     && cost[here.x][here.y-1] == cost[here.x][here.y]-1
     && !(Maps[here.x][here.y] & wallD.S)) return South;

  else return Err;
}

void BFS_makeMap(){
  while(!Map_Q.empty()){
    NodeD check = Map_Q.front();
    Map_Q.pop();

    //Serial.print(check.x);Serial.println(check.y);

    if(check.x + 1 < maxX
       && cost[check.x+1][check.y] == -1
       && !(Maps[check.x][check.y] & wallD.E)){
          cost[check.x+1][check.y] = cost[check.x][check.y] + 1;
          Map_Q.push({check.x+1,check.y});
    //      Serial.print("E");
    }
    if(check.y + 1 < maxY 
       && cost[check.x][check.y+1] == -1 
       && !(Maps[check.x][check.y] & wallD.N)){
          cost[check.x][check.y+1] = cost[check.x][check.y] + 1;
          Map_Q.push({check.x,check.y+1});
    //      Serial.print("N");
    }
    if(check.x - 1 >= 0 
       && cost[check.x-1][check.y] == -1 
       && !(Maps[check.x][check.y] & wallD.W)){
          cost[check.x-1][check.y] = cost[check.x][check.y] + 1;
          Map_Q.push({check.x-1,check.y});
    //      Serial.print("W");
    }
    if(check.y - 1 >= 0 
       && cost[check.x][check.y-1] == -1 
       && !(Maps[check.x][check.y] & wallD.S)){
          cost[check.x][check.y-1] = cost[check.x][check.y] + 1;
          Map_Q.push({check.x,check.y-1});
    //      Serial.print("S");
    }
    //Serial.println();
  }
  //Serial.println();
  //Serial.println("end mapping");
  //Serial.println();

  for(int i = maxY - 1; i >= 0; i--){
    for(int j = 0; j < maxX; j++){
    //  Serial.print(cost[j][i]);
    //  Serial.print(",");
    }
    //Serial.println();
  }
}

void BFS_wayHome(NodeD now){
  NodeD goal = {0,0,North};

  while (!(now.x == goal.x && now.y == goal.y)){//現在地がゴールじゃなかったら、
    int next = BFS_whichWay(now);//絶対方向で生きたい方向を取得
    if(next == Err){
      Serial.print("Err Err Err");
      break;
    }

    if(clearD(next - now.dir) == 0){//差分で行きたい方向を定める
      way_q.push(forward);//前進
      switch(now.dir){//座標更新
        case North:
          now = {now.x,now.y+1,clearD(now.dir +0)};
    //      Serial.print(now.x);Serial.println(now.y);
          break;
        case East:
          now = {now.x+1,now.y,clearD(now.dir +0)};
    //      Serial.print(now.x);Serial.println(now.y);
          break;
        case South:
          now = {now.x,now.y-1,clearD(now.dir +0)};
    //      Serial.print(now.x);Serial.println(now.y);
          break;
        case West:
          now = {now.x-1,now.y,clearD(now.dir +0)};
    //      Serial.print(now.x);Serial.println(now.y);
          break;
        default:
          now = {0,0,North};
      }
    }
    else if(clearD(next - now.dir) == 1){
      way_q.push(left);//左折
      way_q.push(forward);//前進
      switch(now.dir){
        case North:
          now = {now.x+1,now.y,clearD(now.dir +1)};
    //      Serial.print(now.x);Serial.println(now.y);
          break;
        case East:
          now = {now.x,now.y-1,clearD(now.dir +1)};
    //      Serial.print(now.x);Serial.println(now.y);
          break;
        case South:
          now = {now.x-1,now.y,clearD(now.dir +1)};
    //      Serial.print(now.x);Serial.println(now.y);
          break;
        case West:
          now = {now.x,now.y+1,clearD(now.dir +1)};
    //      Serial.print(now.x);Serial.println(now.y);
          break;
        default:
          now = {0,0,North};
      }
    }
    else if(clearD(next - now.dir) == 2){
      way_q.push(right);//右折
      way_q.push(right);//２回かけて後ろ向きに（つまり、左折でもOK）
      way_q.push(forward);//前進
      switch(now.dir){
        case North:
          now = {now.x,now.y-1,clearD(now.dir +2)};
    //      Serial.print(now.x);Serial.println(now.y);
          break;
        case East:
          now = {now.x-1,now.y,clearD(now.dir +2)};
    //      Serial.print(now.x);Serial.println(now.y);
          break;
        case South:
          now = {now.x,now.y+1,clearD(now.dir +2)};
    //      Serial.print(now.x);Serial.println(now.y);
          break;
        case West:
          now = {now.x+1,now.y,clearD(now.dir +2)};
    //      Serial.print(now.x);Serial.println(now.y);
          break;
        default:
          now = {0,0,North};
      }
    }
    else if(clearD(next - now.dir) == 3){
      way_q.push(right);//右折
      way_q.push(forward);//前進
      switch(now.dir){
        case North:
          now = {now.x-1,now.y,clearD(now.dir +3)};
    //      Serial.print(now.x);Serial.println(now.y);//Nw
          break;
        case East:
          now = {now.x,now.y+1,clearD(now.dir +3)};
    //      Serial.print(now.x);Serial.println(now.y);//En
          break;
        case South:
          now = {now.x+1,now.y,clearD(now.dir +3)};
    //      Serial.print(now.x);Serial.println(now.y);//Se
          break;
        case West:
          now = {now.x,now.y-1,clearD(now.dir +3)};
    //      Serial.print(now.x);Serial.println(now.y);//Ws
          break;
        default:
          now = {0,0,North};
      }
    }
  }
  //Serial.println(clearD(goal.dir-now.dir));
  if(!(clearD(goal.dir-now.dir) == 2)){//方向を調整
    switch(clearD(goal.dir-now.dir)){
      
      case 1:
        way_q.push(right);//右折
        break;
      case 0:
        way_q.push(right);//反転
        way_q.push(right);
        break;
      case 3:
        way_q.push(left);//左折
        break;
    }
  }
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);

    srand(time(NULL));

    int maxX = 5;
    int maxY = 5;
    NodeD now = {0,0,North};

    makeMaze(maxX,maxY,map_walls);

    showMap(maxX,maxY,map_walls,now);

  for(int i = 0; i < maxY; i++){
    for(int j = 0; j < maxX; j++){
      cost[j][i] = -1;
    }
  }

  Serial.println("start RHR");

  while(true){
    RHR_inputWalls(walls[now.x][now.y],now);

    //Serial.print(walls[now.x][now.y].rightW);Serial.print("//");
    //Serial.print(walls[now.x][now.y].forwardW);Serial.print("//");
    //Serial.println(walls[now.x][now.y].leftW);

    RHR_inputMaps(walls[now.x][now.y],now);
    now = RHR_nextPosition(now, RHR_whichWay(walls[now.x][now.y],now));
    went[now.x][now.y]++;
    if(now.x == 4 && now.y == 4) break;

    delay(1000);

    showMap(maxX, maxY, map_walls, now);

    //Serial.print(now.x);Serial.print(",");Serial.print(now.y);Serial.print(",");Serial.println(now.dir);
  }
  Serial.println("RHR ok");


  
  NodeD start = {0,0,North};
  cost[0][0] = 0;
  Map_Q.push(start);

  BFS_makeMap();

  BFS_wayHome(now);

  showCostMap(maxX,maxY,Maps,cost);

  while(!way_q.empty()){
    Serial.print(way_q.front());
    way_q.pop();
    Serial.print(",");
  }


}

void loop() {
  // put your main code here, to run repeatedly:

}
