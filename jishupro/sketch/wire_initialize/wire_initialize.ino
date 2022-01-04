
#include <Wire.h>
#include <PCA9685.h>            //PCA9685用ヘッダーファイル（秋月電子通商作成）

PCA9685 pwm = PCA9685(0x40);    //PCA9685のアドレス指定（アドレスジャンパ未接続時）

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
float vel_limit_max[10] = {16.0,16.0,16.0,16.0,16.0,16.0,16.0,16.0,16.0,16.0};
float vel_limit_min[10] = {-20.-20,-20,-20,-20,-20,-20,-20,-20,-20,-20};
// pulse (-100 to 100)
int pulse_limit_max[10] = {17,17,17,17,17,17,17,17,17,17}; 
int pulse_limit_min[10] = {-8,-8,-8,-8,-8,-8,-8,-8,-8,-8};

int set_angle_count[10];
int set_angle(int ch, float vel, float theta, float cthre = 30.0, float thre = 3.0);

float vels[30]; // 速度測定用

/*
const int s0 = 32;
const int s1 = 25;
const int s2 = 34;
const int s3 = 39;
const int SIG_PIN = 36;
*/

const int enc_pin[10] = {12, 14, 27, 26, 25, 33, 32, 35, 34, 39};

void setup() {
 pwm.begin();                   //初期設定 (アドレス0x40用)
 pwm.setPWMFreq(60);            //PWM周期を60Hzに設定 (アドレス0x40用)
 pinMode(init_pin, INPUT_PULLUP);
 /*
 pinMode(s0, OUTPUT);
 pinMode(s1, OUTPUT);
 pinMode(s2, OUTPUT);
 pinMode(s3, OUTPUT);
 */
 Serial.begin(9600);
 for(int i=0;i<n;i++){
    servo_write(i, 0);
 }

 while(!init_done){
    init_enc();
 }
}

// 時計回りが正

void loop() {
  read_enc();
  bool check_initialize = true;
  for(int i=0;i<n;i++){
    Serial.print(String(angle[i]) + ", ");
    if(angle[i] < 60 || angle[i] > 300){
      check_initialize = false;
    }
  }
  if(check_initialize){
    Serial.print("   test passed\n");
  }else{
    Serial.print("   test FAILED!!!!! you must rewrite program or reset\n");
  }
  /*
  for(int i=0;i<n;i++){
    Serial.print(String(servo_zero[i]) + " ");
  }
  Serial.print("\n");
  */
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

float calc_speed(int ch, int p){
  servo_write(ch, p);
  for(int i=0;i<20;i++){
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
  Serial.print("ch " + String(ch) + ", p " + String(p) + ", vel " + String(vel) + "\n");
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

// set servo to theta, vel > 0
// use in loop
// thre: end threshold
// cthre: control start threshold
int set_angle(int ch, float vel, float theta, float cthre, float thre){
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

void servo_write(int ch, int p){ //動かすサーボチャンネルと角度を指定
  if(p > 100) p = 100;
  if(p < -100) p = -100;
  p = map(p, -100, 100, SERVOMIN, SERVOMAX); //角度（0～180）をPWMのパルス幅（150～600）に変換
  pwm.setPWM(ch, 0, p);
  //delay(1);
}

/*
int muxRead(int m){
  digitalWrite(s0, (m>>0 & 1));
  digitalWrite(s1, (m>>1 & 1));
  digitalWrite(s2, (m>>2 & 1));
  digitalWrite(s3, (m>>3 & 1));
  delay(10);
  return analogRead(SIG_PIN);
}
*/

// enc、角度の初期化
void init_enc() {
  int init_value = digitalRead(init_pin);
  if(init_value == 0){ //init
    for(int i=0;i<n;i++){
      // servo_zero[i] = muxRead(i); 
      servo_zero[i] = analogRead(enc_pin[i]);
      servo_n[i] = 0;
      enc[i] = servo_zero[i];
      enc_raw[i] = enc[i];
      angle[i] = ((float)enc[i]) / (float)maxV * 360.0;
      set_angle_count[i] = 0;
    }
    // init成功
    init_done = true;
  }
}

// エンコーダをよむ
// encはenc_raw, servo_n, enc, angleの4つの値で表される
void read_enc() {
  // 何周目かの更新
  for(int i=0;i<n;i++){
    int enc_r = analogRead(enc_pin[i]);
    // int enc_r = muxRead(i);
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
