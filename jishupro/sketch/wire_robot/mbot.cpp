

class mbot{
  public:
  const int n = 10;  
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
    
};
