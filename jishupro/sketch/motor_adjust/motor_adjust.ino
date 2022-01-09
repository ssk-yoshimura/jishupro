#include <Wire.h>
#include <PCA9685.h>            //PCA9685用ヘッダーファイル（秋月電子通商作成）
#undef ESP32
#include <ros.h>
#define ESP32
#include <std_msgs/Float32MultiArray.h>

PCA9685 pwm = PCA9685(0x40);    //PCA9685のアドレス指定（アドレスジャンパ未接続時）
PCA9685 pwm2 = PCA9685(0x41);

// for motor adjustment
// goal: complete rotate_angle
// change rotate_angle, vel2pulse, const value, rotate_data
// set_angle: call when update. do not call this every time

// パルスの割合
// 4096がmax
// パルス幅1000-2000usecが動く範囲
#define SERVOMIN 246            //最小パルス幅 (標準的なサーボパルスに設定)
#define SERVOMAX 492            //最大パルス幅 (標準的なサーボパルスに設定)

const int n=5; // サーボのこすう

const int init_pin = 2; // サーボ初期化スイッチ入力
int servo_zero[10]; // サーボの初期角度
int servo_n[10]; // サーボのかいてんすう、最初0をかていする
int enc[10]; // エンコーダの値
int enc_raw[10]; // エンコーダの生の値
float angle[10]; // 角度 deg
bool init_done = false;
const int maxV = 4096;

// deg/s
float vel_limit_max[10] = {16.0,12.0,17.0,18.0,17.5,16.0,16.0,16.0,16.0,16.0};
float vel_limit_min[10] = {-20,-20,-16,-15,-20,-20,-20,-20,-20,-20};
// pulse (-100 to 100)
int pulse_limit_max[10] = {17,16,18,16,16,17,17,17,17,17}; 
int pulse_limit_min[10] = {-7,-6,-8,-8,-7,-8,-8,-8,-8,-8};
// grad : vel / pulse
float k_p[10] = {3.9, 3.7, 3.7, 3.8, 3.9, 3.7, 3.7, 3.7, 3.7, 3.7};
float k_m[10] = {3.7, 3.9, 3.3, 3.6, 3.7, 3.7, 3.7, 3.7, 3.7, 3.7};

int set_angle_count[10];
void set_angle(int ch, float vel, float theta, float cthre=10.0, float thre=5.0);
void rotate_angle();
struct rotate_data{
  float vel; //set vel
  float theta;
  float cthre=10.0;
  float thre=5.0;
  float dangle=0.0; 
  float rvel=0.0; //real vel
};

rotate_data Rdata[10];

float wire2angle(int ch, float w);
float angle2wire(int ch, float agl);

float vels[30]; // 速度測定用

//const int enc_pin[10] = {36, 39, 34, 35, 32, 14, 27, 26, 25, 33};
const int enc_pin[10] = {36, 39, 34, 35, 32, 33, 25, 26, 27, 14};

const float wire_init_pose[10] = {
  82.0293, 49.619, 34.055, 262.47, 61.8248,
  82.0293, 49.619, 34.055, 262.47, 61.8248
};
const float angle_init_pose[10] = {
  57.74, 289.60, 19.69, 248.91, 196.08, 236.95, 234.84, 27.16, 124.98, 223.33
};

float wire_list_goal[10];

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

float goal_sequence[50][12]; // (idx, time, wl)
int goal_sequence_length = 0;
int get_sequence_state = 0; //0:no 1:getting 2:moving
int get_sequence_count = 0;
int seq_test = 0;
int seq_crt_idx = 0;
int goal_offset = 0;

void setup() {
 Serial.begin(9600);
 pwm.begin();                   //初期設定 (アドレス0x40用)
 pwm.setPWMFreq(60);            //PWM周期を60Hzに設定 (アドレス0x40用)
 pwm2.begin();
 pwm2.setPWMFreq(60);
 pinMode(init_pin, INPUT_PULLUP);

 for(int i=0;i<n;i++){
    servo_write(i, 0);
 }

 while(!init_done){
    init_enc();
    for(int i=0;i<n;i++){
      Rdata[i].theta = angle[i]; // !!!!!!!!!!!!! initialize Rdata[i].theta !!!!!!!!
    }
    Serial.print("waiting initialize\n");
    //init_enc_allrange();
 }

  /*
 Serial.print(calc_speed(0, vel2pulse(0, -50)));
 Serial.print((int)(vel2pulse(0, -50)));
 Serial.print("hellooo\n");
  */
  /*
  for(int i=0;i<5;i++){
    calc_min_max(i);
    calc_min_max(i);
  }
  */
  // (ch, vel, theta, cthre, thre)
  set_angle(1, 200, 270);
 
}

// -100から100にできないか -> できた
// 時計回りが正

void loop() {
  read_enc();

  /*
  Serial.print("rvel ");
  for(int i=0;i<n;i++){
    Serial.print(String(Rdata[i].rvel) + " ");
  }
  Serial.print("\n");
  Serial.print("dangle ");
  for(int i=0;i<n;i++){
    Serial.print(String(Rdata[i].dangle) + " ");
  }
  Serial.print("\n");
  */


  rotate_angle(); // call in every loop
  int cht = 1;
  Serial.println(String(Rdata[cht].theta) + " " + String(angle[cht]) + " " + String(Rdata[cht].rvel) + " " + String(Rdata[0].dangle));

  delay(100);  
}

void calc_min_max(int ch){
  // max
  int pmx,pmn;
  float vmx, vmn;
  for(int pls=0;pls<30;pls++){
    float spd = calc_speed(ch, pls);
    if(spd > 10){
      vmx = spd;
      pmx = pls;
      break;
    }
  }
  //Serial.println("!!!!!servo" + String(ch) + "  vmx=" + String(vmx) + ", pmx=" + String(pmx));
  for(int pls=0;pls>-30;pls--){
    float spd = calc_speed(ch, pls);
    if(spd < -10){
      vmn = spd;
      pmn = pls;
      break;
    }
  }
  //Serial.println("!!!!!servo" + String(ch) + "  vmn=" + String(vmn) + ", pmn=" + String(pmn));
  float v30 = calc_speed(ch, 30);
  float v50 = calc_speed(ch, 50);
  float k = (v50-v30) / 20.0;
  //Serial.println("!!!!!servo" + String(ch) + "  k=" + String(k));
  float m30 = calc_speed(ch, -30);
  float m50 = calc_speed(ch, -50);
  float km = (m30-m50) / 20.0;
  //Serial.println("!!!!!servo" + String(ch) + "  k=" + String(km));
  Serial.println("Result: " + String(ch) + " " + String(vmx) + " " + String(vmn) + " " + String(pmx) + " " + String(pmn) + " " + String(k) + " " + String(km));
}

// p in (-100,100), return vel [deg/s]
float calc_speed(int ch, int p){
  servo_write(ch, 0);
  for(int i=0;i<10;i++){
    read_enc();
    delay(100);
  }
  servo_write(ch, p);
  for(int i=0;i<10;i++){
    read_enc();
    delay(100);
  }
  // Serial.print("Start calc: servo " + String(ch) + ", p=" + String(p) + "\n");
  unsigned long tm0 = micros();
  float ang0 = angle[ch];
  for(int i=0;i<20;i++){
    read_enc();
    delay(100);
  }
  unsigned long tm1 = micros();
  float ang1 = angle[ch];
  unsigned long tm = tm1 - tm0;
  float ang_diff = ang1 - ang0;
  float vel = ang_diff / (float)(tm / 1000000.0);
  //Serial.print("ch " + String(ch) + ", p " + String(p) + ", vel " + String(vel) + "\n");
  // Serial.print("tm is " + String(tm) + " angle is " + String(ang_diff) + ", vel is " + String(vel));
  // Serial.print("\n");
  // Serial.println(vel);
  for(int i=0;i<5;i++){
    read_enc();
    delay(100);
  }
  servo_write(ch, 0);
  return vel;
}



// vel[deg/s] to pulse in (-100,100)
int vel2pulse(int ch, float vel){
  int pulse = 0;
  if(vel >= vel_limit_max[ch]){
    pulse = pulse_limit_max[ch];
    pulse += (int)((vel - vel_limit_max[ch]) / k_p[ch]);
  }else if(vel <= vel_limit_min[ch]){
    pulse = pulse_limit_min[ch];
    pulse += (int)((vel - vel_limit_min[ch]) / k_m[ch]);
    // Serial.print((vel - vel_limit_min[ch]) / 3.7);
  }
  return pulse;
}

// set servo to theta, vel > 0
// use in loop
// thre: end threshold
// cthre: control start threshold
void set_angle(int ch, float vel, float theta, float cthre, float thre){
  // default
  // cthre = 10.0;
  // thre = 5.0;
  Rdata[ch].vel = vel;
  Rdata[ch].theta = theta;
  Rdata[ch].cthre = cthre;
  Rdata[ch].thre = thre;
}

void rotate_angle(){
  for(int ch=0;ch<n;ch++){

    float vel = Rdata[ch].vel;
    float theta = Rdata[ch].theta;
    float cthre = Rdata[ch].cthre;
    float thre = Rdata[ch].thre;
    
    // 安全
    if(vel < 0)vel *= -1.0;
    if(vel > 200) vel = 200;
    if(vel < 25) vel = 25;
 
    float dist = theta - angle[ch];
    Rdata[ch].dangle = dist;
    if(abs(dist) < thre){
      set_angle_count[ch]++;
      servo_write(ch, 0);
      Rdata[ch].rvel = 0.0;
      continue;
    }
    /*
    if(abs(dist) > cthre){
      if(dist < 0)vel = -vel;
      servo_write(ch, vel2pulse(ch, vel));
      Rdata[ch].rvel = vel;
    }else{
    // dist<=cthreの場合
     // シンプル
    //vel = 50 *(dist / cthre);
    //servo_write(ch, vel2pulse(ch, vel));
     
      // 線形
      float vel_max_tmp = 30.0;
      if(dist > 0){
        vel = vel_limit_max[ch] + (vel_max_tmp - vel_limit_max[ch]) * dist / cthre;
      }else{
        vel = vel_limit_min[ch] + (vel_max_tmp + vel_limit_min[ch]) * dist / cthre;
      }
      servo_write(ch, vel2pulse(ch, vel));
      Rdata[ch].rvel = vel;
    }
    */
    // linear, min, not use cthre
    vel = min(vel, float(25.0 + 175.0 / 45.0 * (abs(dist) - 5.0)));
    vel = min(vel, float(100.0 + 0.4 * 175.0 / 45.0 * (abs(dist) - 24.0)));
    if(dist < 0) vel *= -1.0;
    servo_write(ch, vel2pulse(ch, vel));
    Rdata[ch].rvel = vel;
    set_angle_count[ch] = 0;
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
      /*
      if(abs(angle[i] - angle_init_pose[i]) > 50){
        wire_check_tmp = false;
      }
      */
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
