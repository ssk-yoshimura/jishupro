#include <Wire.h>
#include <PCA9685.h>            //PCA9685用ヘッダーファイル（秋月電子通商作成）
#undef ESP32
#include <ros.h>
#define ESP32
#include <std_msgs/Float32MultiArray.h>

PCA9685 pwm = PCA9685(0x40);    //PCA9685のアドレス指定（アドレスジャンパ未接続時）
PCA9685 pwm2 = PCA9685(0x41);



// パルスの割合
// 4096がmax
// パルス幅1000-2000usecが動く範囲
#define SERVOMIN 246            //最小パルス幅 (標準的なサーボパルスに設定)
#define SERVOMAX 492            //最大パルス幅 (標準的なサーボパルスに設定)

const int n=10; // サーボのこすう

const int init_pin = 2; // サーボ初期化スイッチ入力
int servo_zero[10]; // サーボの初期角度
int servo_n[10]; // サーボのかいてんすう、最初0をかていする
int enc[10]; // エンコーダの値
int enc_raw[10]; // エンコーダの生の値
float angle[10]; // 角度 deg
bool init_done = false;
const int maxV = 4096;

// deg/s
float vel_limit_max[10] = {16.0,16.0,16.0,16.0,16.0,16.0,16.0,16.0,16.0,16.0};
float vel_limit_min[10] = {-20.-20,-20,-20,-20,-20,-20,-20,-20,-20,-20};
// pulse (-100 to 100)
int pulse_limit_max[10] = {17,17,17,17,17,17,17,17,17,17}; 
int pulse_limit_min[10] = {-8,-8,-8,-8,-8,-8,-8,-8,-8,-8};

int set_angle_count[10];
int set_angle(int ch, float vel, float theta, float cthre=30.0, float thre=5.0);

float wire2angle(int ch, float w);
float angle2wire(int ch, float agl);

float vels[30]; // 速度測定用

/*
const int s0 = 32;
const int s1 = 25;
const int s2 = 34;
const int s3 = 39;
const int SIG_PIN = 36;
*/

//const int enc_pin[10] = {36, 39, 34, 35, 32, 14, 27, 26, 25, 33};
const int enc_pin[10] = {36, 39, 34, 35, 32, 33, 25, 26, 27, 14};


ros::NodeHandle nh;
std_msgs::Float32MultiArray wirelen;
ros::Publisher p("arduino", &wirelen);
float wire_list_goal[10];

//init-poseのときのwire-list-arduino
// eusで計算する(:wire-calc後にwire-list-to-arduinoする)
const float wire_init_pose[10] = {
  82.0293, 49.619, 34.055, 262.47, 61.8248,
  82.0293, 49.619, 34.055, 262.47, 61.8248
};

// init-poseのときのangle
// wire_initializeで求める
const float angle_init_pose[10] = {
  //120.32, 135.97, 332.93, 259.89, 230.89, 288.63, 264.38, 19.25, 60.47, 111.97
  //71.89, 132.45, 348.93, 279.84, 246.09, 285.73, 307.18, 9.32, 42.45, 122.08
  88.77, 176.92, 288.72, 227.90, 226.41, 231.77, 281.25, 0.00, 61.79, 143.53
};

// サーボ回転角の係数
// pull when counterclockwise : 1
// pull when clockwise : -1
const int pull_direction[10] = {
  1,1,-1,-1,-1,
  -1,-1,1,1,1
};

float wire_list[10];

// servo check
bool wire_init_check = false;

// pulley radius
const float r_pulley = 11.0;

// ここでワイヤー長さを得て、角度を制御する
void messageCb(const std_msgs::Float32MultiArray& msg){
  for(int i=0;i<msg.data_length;i++){
    wire_list_goal[i] = msg.data[i];
    // wirelen.data[i] = wire_list_goal[i];
  }
}

float goal_sequence[30][12]; // (idx, time, wl)
int goal_sequence_length = 0;
int get_sequence_state = 0; //0:no 1:getting
int get_sequence_count = 0;
int seq_test = 0;

void messageCbsequence(const std_msgs::Float32MultiArray& msg){

  if(msg.data[0] == 0.0){ // 受信開始
    get_sequence_state = 1;
    get_sequence_count = 0;
    goal_sequence_length = 0;
    for(int i=0;i<30;i++){
      goal_sequence[i][0] = 0.0;
      goal_sequence[i][1] = 0.0;
    }
  }
  if(get_sequence_state == 1){ // 受信
    //seq_test++;
    if(msg.data[1] == -99){ //終了
      goal_sequence_length = msg.data[0];
      get_sequence_state = 0;
    }
    for(int i=0;i<msg.data_length;i++){
      goal_sequence[get_sequence_count][i] = msg.data[i];
    }
    if(get_sequence_count != (int)goal_sequence[get_sequence_count][0]){ //値が飛んだとき
      get_sequence_state = 0;
    }
    get_sequence_count++;
  }
}

ros::Subscriber<std_msgs::Float32MultiArray> s("wireus", messageCb);
ros::Subscriber<std_msgs::Float32MultiArray> seq("wire_sequence", messageCbsequence);

void setup() {
 // Serial.begin(57600);
 pwm.begin();                   //初期設定 (アドレス0x40用)
 pwm.setPWMFreq(60);            //PWM周期を60Hzに設定 (アドレス0x40用)
 pwm2.begin();
 pwm2.setPWMFreq(60);
 pinMode(init_pin, INPUT_PULLUP);
 for(int i=0;i<n;i++){
  wire_list_goal[i] = wire_init_pose[i];
 }
 nh.initNode();
 wirelen.data = (float*)malloc(sizeof(float) * 10);
 wirelen.data_length = 10;
 for(int i=0;i<wirelen.data_length;i++){
  wirelen.data[i] = 123.0;
 }
 nh.advertise(p);
 nh.subscribe(s);
 nh.subscribe(seq);
 for(int i=0;i<n;i++){
    servo_write(i, 0);
 }

 while(!init_done){
    //init_enc();
    init_enc_allrange();
 }
}

// -100から100にできないか -> できた
// 時計回りが正

int a = 0;
int b = 0;

// 
void loop() {
  read_enc();
  if(!wire_init_check){
     for(int i=0;i<wirelen.data_length;i++){
        wirelen.data[i] = -99.0;
     } 
     wirelen.data[0] = (float)goal_sequence_length;
     //wirelen.data[0] = (float)seq_test;
    p.publish(&wirelen);
    nh.spinOnce();
    delay(100);
    return;
  }
  for(int i=0;i<n;i++){ // wire
    float angle_goal = wire2angle(i, wire_list_goal[i]);
    set_angle(i, 100.0, angle_goal);
    // wirelen.data[i] = angle2wire(i, angle[i]); // 現在の長さ
    // wirelen.data[i] = wire_list_goal[i]; // 目標の長さ
    wirelen.data[i] = wire_list_goal[i] - angle2wire(i, angle[i]); // 現在と目標の差分
  }
  p.publish(&wirelen);
  nh.spinOnce();
  delay(100);  
}

// vel[deg/s] to pulse in (-100,100)
int vel2pulse(int ch, float vel){
  int pulse = 0;
  if(vel >= vel_limit_max[ch]){
    pulse = pulse_limit_max[ch];
    pulse += (int)((vel - vel_limit_max[ch]) / 3.7);
  }else if(vel <= vel_limit_min[ch]){
    pulse = pulse_limit_min[ch];
    pulse += (int)((vel - vel_limit_min[ch]) / 3.7);
  }
  return pulse;
}

// set servo to theta, vel > 0
// use in loop
// thre: end threshold
// cthre: control start threshold
int set_angle(int ch, float vel, float theta, float cthre, float thre){

  cthre = 30.0;
  thre = 5.0;
   float dist = theta - angle[ch];
   if(abs(dist) < thre){
    set_angle_count[ch]++;
    servo_write(ch, 0);
    return set_angle_count[ch];
   }
   if(abs(dist) > cthre){
     if(dist < 0)vel = -vel;
     servo_write(ch, vel2pulse(ch, vel));
   }else{
    // dist<=cthreの場合
    /* // シンプル
    vel = 50 *(dist / cthre);
    servo_write(ch, vel2pulse(ch, vel));
    */ 
    // 線形
    float vel_max_tmp = 50.0;
    if(dist > 0){
      vel = vel_limit_max[ch] + (vel_max_tmp - vel_limit_max[ch]) * dist / cthre;
    }else{
      vel = vel_limit_min[ch] + (vel_max_tmp + vel_limit_min[ch]) * dist / cthre;
    }
    servo_write(ch, vel2pulse(ch, vel));
   }
   set_angle_count[ch] = 0;
   return set_angle_count[ch];
}

// set wire using ros
void set_wire(){
  // wire_list_goal[] : goal length, from eus
  // wire_init_pose[] : wire length when init-pose
  // angle_init_pose[] : servo angle when init-pose
  // pull_direction[] : pull direction
  for(int i=0;i<n;i++){
    float angle_goal = wire2angle(i, wire_list_goal[i]);
    set_angle(i, 50.0, angle_goal,0.0,0.0);
  }
}

float wire2angle(int ch, float w){
  float angle_goal_tmp = angle_init_pose[ch] +
    (float)pull_direction[ch] * (w - wire_init_pose[ch]) / r_pulley * 180.0 / PI;
  return angle_goal_tmp;
}

float angle2wire(int ch, float agl){
  float wire_goal_tmp = wire_init_pose[ch] +
    (float)pull_direction[ch] * (agl - angle_init_pose[ch]) * r_pulley * PI / 180.0;
  return wire_goal_tmp;
}

void servo_write(int ch, int p){ //動かすサーボチャンネルと角度を指定
  if(p > 100) p = 100;
  if(p < -100) p = -100;
  p = map(p, -100, 100, SERVOMIN, SERVOMAX); //角度（0～180）をPWMのパルス幅（150～600）に変換
  if(ch < 5){
    pwm.setPWM(15-ch, 0, p);
  }else{
    pwm2.setPWM(ch-5, 0, p);
  }
  //delay(1);
}


// enc、角度の初期化
void init_enc() {
  int init_value = digitalRead(init_pin);
  bool wire_check_tmp = true;
  if(init_value == 0){ //init
    for(int i=0;i<n;i++){
      // servo_zero[i] = muxRead(i); 
      servo_zero[i] = analogRead(enc_pin[i]);
      servo_n[i] = 0;
      enc[i] = servo_zero[i];
      enc_raw[i] = enc[i];
      angle[i] = ((float)enc[i]) / (float)maxV * 360.0;
      set_angle_count[i] = 0;
      if(abs(angle[i] - angle_init_pose[i]) > 50){
        wire_check_tmp = false;
      }
    }
    // init成功
    init_done = true;
    wire_init_check = wire_check_tmp;
  }
}

void init_enc_allrange() {
  int init_value = digitalRead(init_pin);
  bool wire_check_tmp = true;
  if(init_value == 0){ //init
    for(int i=0;i<n;i++){
      // servo_zero[i] = muxRead(i); 
      servo_zero[i] = analogRead(enc_pin[i]);
      enc_raw[i] = servo_zero[i];

      float min_dist = 10000;
      int min_dist_idx = 0;
      for(int j=-1;j<2;j++){
        int enc_next = maxV * j + enc_raw[i];
        float angle_next = ((float)enc_next) / (float)maxV * 360.0;
        float angle_next_distance = abs(angle_next - angle_init_pose[i]);
        if(angle_next_distance < min_dist){
          min_dist = angle_next_distance;
          min_dist_idx = j;
        }
      }
      
      servo_n[i] = min_dist_idx;
      
      enc[i] = servo_n[i] * maxV + enc_raw[i];
      angle[i] = ((float)enc[i]) / (float)maxV * 360.0;
      set_angle_count[i] = 0;
      if(abs(angle[i] - angle_init_pose[i]) > 50){
        wire_check_tmp = false;
      }
    }
    // init成功
    init_done = true;
    wire_init_check = wire_check_tmp;
  }
}

// エンコーダをよむ
// encはenc_raw, servo_n, enc, angleの4つの値で表される
void read_enc() {
  // 何周目かの更新
  for(int i=0;i<n;i++){
    // int enc_r = analogRead(i);
    // int enc_r = muxRead(i);
    int enc_r = analogRead(enc_pin[i]);
    if(enc_r < 0) enc_r = 0;
    if(enc_r > maxV) enc_r = maxV;
    int min_dist = 10000;
    int min_dist_idx = 1;
    for(int j=0;j<3;j++){
      int enc_next = maxV * (servo_n[i]+j-1) + enc_r;
      int enc_next_distance = abs(enc_next - enc[i]);
      if(enc_next_distance < min_dist){
        min_dist = enc_next_distance;
        min_dist_idx = j;
      }
    }
    enc_raw[i] = enc_r;
    servo_n[i] += (min_dist_idx - 1);
    enc[i] = servo_n[i] * maxV + enc_raw[i];
    angle[i] = ((float)enc[i]) / (float)maxV * 360.0;
  }
}
