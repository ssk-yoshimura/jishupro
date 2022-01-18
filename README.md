# 腱之介（けんのすけ）
自主プロ2021のために製作した、ワイヤー駆動ロボット

#### アピールポイント

- 6自由度を筋肉10本で動かすロボット（足回り2輪付き）
- 機構を工夫し、安いモータで大きなロボットを動かす（ロボットアーム長0.5m）
- IKを解き、関節角度軌道->ワイヤー長軌道を計算して追従&モータの協調動作を実現
- 画像認識もできる

## メカ構成
基本的に、3Dプリンタとレーザー加工機で製作。骨組み等はホームセンターで入手可能で安価なものを利用。

モータは秋月の連続回転サーボ（エンコーダ付き）FB5311M-360を使用。
- 根本にアクチュエータを集約し、ロボットアームを軽量化
- 肩関節は2自由度3ワイヤー、肘関節は1自由度2ワイヤー
- 両腕と足回り2輪で、合計12個のモータを使用
- モーメントアームを大きくし、減速比を上げてトルクを大きくした。

## 回路構成
- ESP32を使用
- 秋月のI2C接続16チャンネル　サーボ＆ＰＷＭ駆動キットを使用
- 電源は5v4AのACアダプターx2

## ソフトウェア構成
ROSを使用し、IK等にはEusLispを利用した。次のような順番で処理を行っている。

1. (python)Coral TPUからPoseArrayトピックをサブスクライブし、人の姿勢の判別等を行ったり、色で画像処理を行ったりする。実際はAction Serverとして、呼び出された時のみ処理を行う
3. (euslisp)画像認識の場合はAction Clientとなって利用
4. (euslisp)ロボット手先の軌道を作成
5. (euslisp)手先の軌道をもとにIKを解き、関節角度軌道を作成
6. (euslisp)関節角度軌道から順運動学等でワイヤー長軌道を作成
7. (euslisp)ワイヤー長軌道と時間、制御パラメータ等をまとめたものを分割し、Float32MultiArrayトピックとしてESP32に送る
8. (ESP32)受信した軌道に欠落がないことを確認し、モータの角度軌道に変換
9. (ESP32)モータの角度軌道をもとに、エンコーダのフィードバックを使って追従制御&協調して動作させる

CAD(SolidWorks)で設計したデータをもとにURDFファイルを作成し、STLを貼り付けた。これを使用して、eusのビューワやRvizでシミュレーションを行った。eusはURDF,yamlから変換してロボットモデルを自動生成した。

## 実演手順
### 準備
```
roscore
```
#### wire_initialize
- 電源は外し、init_pinも外して書き込む
- リセットボタンを押す->電源つける->init_pinつける
- シリアルモニタで異常なしを確認、値をコピー
#### wire_pull
- コピーした値を貼り付ける
- 電源は外し、init_pinも外して書き込む
- リセットボタンを押す->電源つける->init_pinつける
```
rosrun rosserial_python serial_node.py
```
#### euslisp
jishupro/euslispに入る
```
em demo.l
```
シェルを開いて
```
roseus "demo.l
```
```
walk-prepare
```
#### USBカメラ 
ここで初めて、カメラとCoralを付ける。
- フリーズした場合、カメラを挿し直すと改善することがある。

```ls /dev/video```でvideo1があることを確認する。

```
rosrun uvc_camera uvc_camera_node _device:=/dev/video1
```

- PCのカメラの場合```rosrun usb_cam usb_cam_node```

#### Edge TPU 
```
source /opt/ros/melodic/setup.bash
```
```
source ~/coral_ws/devel/setup.bash
```
```
roslaunch coral_usb edgetpu_human_pose_estimator.launch INPUT_IMAGE:=/image_raw
```
#### Action Server
```
python cv_pose_action.py
```
### 発表
#### eusプログラム
- walk-prepare
- walk-both
- walk-back
- rotate
- yc
## ESP32のプログラム起動の諸注意
- モータの電源を抜き、init_pinを繋げない状態で書き込む。
- リセットボタンを押してから、モータの電源を入れて、その後init_pinを繋げる。
- rosrun rosserial_...する。だめそうだったらリセットボタンからやり直す。
- モータが異常回転している場合、エンコーダの線が抜けている（initialize時にエンコーダの値をよく見て、異常がないことを確認すること）
- 最悪、動画を見せる。

## Coral TPUの諸注意
- カメラ・coralを挿さずにまずesp32のセットアップをする。
- coralはPC（右側）に直接挿す。
- usbカメラはハブのPC側、esp32はハブの先側（横）に挿す。
- 背筋をまっすぐ、少し前屈み。

## 発表
休憩時間に必ずテストランすること。
- 本質にしか触れない
- 2枚目：ワイヤーの話のみ
- 3枚目：おおざっぱに
- 4枚目：位置制御（速度指定サーボ、エンコーダにも軽く触れる）、姿勢推定

## ESP32のプログラム起動がうまくいかないとき
- モータの電源を抜いて試す
- i2cの線を抜く
- リセットボタンを押す
- 替えのESP32を買ってくる
- モータの電源なしでプログラム起動->電源入れる->init_pin入れる
- i2cの線を抜いた状態で書き込み直す
- i2cの5v, GNDを抜いた状態で書き込み直す

- i2cの5v, GNDを抜いた状態で書き込み、これらを挿し戻した後、モータの電源を抜いてリセットボタンを押してから、init_pin, モータ電源の順
- i2cのピンさえ抜いていれば電源はつけたままでよい？

- まともに動かなくなった。
- 買い換える


## esp32でrosを使う手順
### 無線

https://github.com/ssk-yoshimura/jishupro/blob/master/jishupro/sketch/esp_ros_wifi_sample/esp_ros_wifi_sample.ino

スケッチは```Serial.begin(57600)```としておくとラク

PC

```bash
roscore
rosrun rosserial_python serial_node.py tcp
```

次のように表示される
```
[INFO] [1640525677.837441]: ROS Serial Python Node
Fork_server is:  False
[INFO] [1640525677.857589]: Waiting for socket connections on port 11411
waiting for socket connection
```

ここでESP32のENボタンを押すなどして、電源を入れ直す（重要）

少し待つと繋がる
```
[INFO] [1640525688.632853]: Established a socket connection from 192.168.1.17 on port 64560
[INFO] [1640525688.638527]: calling startSerialClient
...
```

### 有線

https://github.com/ssk-yoshimura/jishupro/blob/master/jishupro/sketch/esp32_array_test/esp32_array_test.ino

スケッチで
```c
#undef ESP32
#include <ros.h>
#define ESP32
```
としておくと、```rosrun rosserial_python serial_node.py```で繋がる

```Serial.begin```は書かない（重要）

# jishupro

## eusモデルの作り方
SolidWorksからstlを出力するときは、解像度を下げて、単位をmにする

https://github.com/jsk-ros-pkg/jsk_model_tools

例
```bash
roscd jishupro/models
rosrun collada_urdf_jsk_patch urdf_to_collada sample_model.urdf sample_model.dae
rosrun euscollada collada2eus sample_model.dae sample_model.l
```
https://github.com/ssk-yoshimura/jishupro/tree/master/jishupro/models にMakefileを書いたのでmakeすればできる

## サーボとワイヤーの初期化手順
### 初期設定（一度だけ）
```:init-pose```のときのワイヤー長とサーボ角度を調べて記録する。
#### eusの設定(eusモデル、初期ポーズ変更時以外はやらなくてよい)
```
roseus
(load "mbot-utils.l)
(load "mbot-ri.l")
(send *mbot* :wire-init)
(send *mbot* :init-pose)
(send *mbot* :wire-calc)
(wire-list-to-arduino)
wire-list-arduino
```
これでワイヤー初期長さが表示されるので、wire_pullスケッチのwire_init_pose[]にその値を代入する
#### esp32の設定
- 手動で```:init-pose```の状態にする
  - ここでワイヤーをしっかり張ることが超重要
- wire_initializeスケッチを実行し、シリアルモニタを見る
- エンコーダの値が表示されるので、異常がないことを確認する
- 設定したいサーボの角度を記録する
- wire_pullスケッチのangle_init_pose[]にその値を代入する
### 通常時の設定
- wire_initializeスケッチで、サーボの角度がangle_init_pose +- 50degの範囲にあることを確認する
- 範囲外なら調整して範囲内にし、リセットボタンを押してやり直す
- 範囲外の場合、wire_pullスケッチはwire_init_checkがfalseのままなので動かない。
- (TODO) 範囲外の場合、調整用の角度情報をパブリッシュするようにする

## スケッチ実行時の手順
モータの電源が入っていることを確かめるためにやること
 - wire_initializeやwire_pullスケッチを実行する
 - モータの電源が入っていることを確認する（青いLEDが光る）
 - GPIO2をGNDにつなげる
