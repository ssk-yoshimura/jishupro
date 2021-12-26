#include <Wire.h>
#include <PCA9685.h>            //PCA9685用ヘッダーファイル（秋月電子通商作成）
// #include <ros.h>
// #include <std_msgs/Float32MultiArray.h>

PCA9685 pwm = PCA9685(0x40);    //PCA9685のアドレス指定（アドレスジャンパ未接続時）

// パルスの割合
// 4096がmax
// パルス幅1000-2000usecが動く範囲
#define SERVOMIN 246            //最小パルス幅 (標準的なサーボパルスに設定)
#define SERVOMAX 492            //最大パルス幅 (標準的なサーボパルスに設定)
// #define PI 3.141592653589793

const int n=2; // サーボのこすう

const int init_pin = 7; // サーボ初期化スイッチ入力
int servo_zero[10]; // サーボの初期角度
int servo_n[10]; // サーボのかいてんすう、最初0をかていする
int enc[10]; // エンコーダの値
int enc_raw[10]; // エンコーダの生の値
float angle[10]; // 角度 deg
bool init_done = false;
const int maxV = 675;

// deg/s
float vel_limit_max[10] = {16.0,16.0,16.0,16.0,16.0,16.0,16.0,16.0,16.0,16.0};
float vel_limit_min[10] = {-20.-20,-20,-20,-20,-20,-20,-20,-20,-20,-20};
// pulse (-100 to 100)
int pulse_limit_max[10] = {17,0,17.0,17.0,17.0,17.0,17.0,17.0,17.0,17.0}; 
int pulse_limit_min[10] = {-8,-8,-8,-8,-8,-8,-8,-8,-8,-8};

int set_angle_count[10];
int set_angle(int ch, float vel, float theta, float cthre = 30.0, float thre = 3.0);

float wire2angle(int ch, float w);
float angle2wire(int ch, float agl);

float vels[30]; // 速度測定用

const int s0 = 11;
const int s1 = 10;
const int s2 = 9;
const int s3 = 8;
const int SIG_PIN = 0;

/*
ros::NodeHandle nh;
std_msgs::Float32MultiArray wirelen;
ros::Publisher p("arduino", &wirelen);
*/
float wire_list_goal[10];

//init-poseのときのwire-list
const float wire_init_pose[10] = {
  97.33,
  53.8,
  37.7,
  178.34,
  59.5,
  97.3,
  53.8,
  37.7,
  178.34,
  59.5
};

// init-poseのときのangle
const float angle_init_pose[10] = {
  244.27,
  245.87,
  0,0,0,0,0,0,0,0
};

// サーボ回転角の係数
// pull when counterclockwise : 1
// pull when clockwise : -1
const float pull_direction[10] = {
  1,-1,
  1,1,1,1,1,1,1,1
};

float wire_list[10];

// servo check
bool wire_init_check = false;

// pulley radius
const float r_pulley = 11.0;

// いまは返すだけ
// ここでワイヤー長さを得て、角度を制御する
/*
void messageCb(const std_msgs::Float32MultiArray& msg){
  for(int i=0;i<msg.data_length;i++){
    wire_list_goal[i] = msg.data[i];
    // wirelen.data[i] = wire_list_goal[i];
  }
}
*/

// ros::Subscriber<std_msgs::Float32MultiArray> s("wires", messageCb);

void setup() {
 pwm.begin();                   //初期設定 (アドレス0x40用)
 pwm.setPWMFreq(60);            //PWM周期を60Hzに設定 (アドレス0x40用)
 pinMode(init_pin, INPUT_PULLUP);
 pinMode(s0, OUTPUT);
 pinMode(s1, OUTPUT);
 pinMode(s2, OUTPUT);
 pinMode(s3, OUTPUT);
 Serial.begin(9600);
 for(int i=0;i<n;i++){
  wire_list_goal[i] = wire_init_pose[i] + 5.76;
 }
 //nh.initNode();
 //wirelen.data = (float*)malloc(sizeof(float) * 10);
 //wirelen.data_length = 10;
 /*
 for(int i=0;i<wirelen.data_length;i++){
  wirelen.data[i] = 123.0;
 }
 */
 //nh.advertise(p);
 //nh.subscribe(s);
 for(int i=0;i<n;i++){
    servo_write(i, 0);
 }

 while(!init_done){
    init_enc();
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
     /*
     for(int i=0;i<wirelen.data_length;i++){
        wirelen.data[i] = -99.0;
     }
    p.publish(&wirelen);
    nh.spinOnce();
    */
    delay(100);
    return;
  }
  set_wire();
  float aaa = wire2angle(0, wire_list_goal[0]);
  Serial.print(String(aaa) + " " + String(wire2angle(1, wire_list_goal[1])) + "\n");
  Serial.print(String(angle2wire(0, aaa)) + "\n");
  //p.publish(&wirelen);
  //nh.spinOnce();
  delay(500);  
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
int set_angle(int ch, float vel, float theta, float cthre = 30.0, float thre = 3.0){
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

void set_wire(){
  // wire_list_goal[] : goal length
  // wire_init_pose[] : wire length when init-pose
  // angle_init_pose[] : servo angle when init-pose
  // pull_direction[] : pull direction
  for(int i=0;i<n;i++){
    float angle_goal = wire2angle(i, wire_list_goal[i]);
    // wirelen.data[i] = angle_goal;
    // set_angle(i, 50.0, angle_goal);
  }
}

float wire2angle(int ch, float w){
  float angle_goal_tmp = angle_init_pose[ch] +
    pull_direction[ch] * (w - wire_init_pose[ch]) / r_pulley * 180.0 / PI;
  return angle_goal_tmp;
}

float angle2wire(int ch, float agl){
  float wire_goal_tmp = wire_init_pose[ch] +
    pull_direction[ch] * (agl - angle_init_pose[ch]) * r_pulley * PI / 180.0;
  return wire_goal_tmp;
}

void servo_write(int ch, int p){ //動かすサーボチャンネルと角度を指定
  if(p > 100) p = 100;
  if(p < -100) p = -100;
  p = map(p, -100, 100, SERVOMIN, SERVOMAX); //角度（0～180）をPWMのパルス幅（150～600）に変換
  pwm.setPWM(ch, 0, p);
  //delay(1);
}

int muxRead(int m){
  digitalWrite(s0, (m>>0 & 1));
  digitalWrite(s1, (m>>1 & 1));
  digitalWrite(s2, (m>>2 & 1));
  digitalWrite(s3, (m>>3 & 1));
  return analogRead(SIG_PIN);
}

// enc、角度の初期化
void init_enc() {
  int init_value = digitalRead(init_pin);
  bool wire_check_tmp = true;
  if(init_value == 0){ //init
    for(int i=0;i<n;i++){
      servo_zero[i] = muxRead(i); 
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

// エンコーダをよむ
// encはenc_raw, servo_n, enc, angleの4つの値で表される
void read_enc() {
  // 何周目かの更新
  for(int i=0;i<n;i++){
    // int enc_r = analogRead(i);
    int enc_r = muxRead(i);
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
