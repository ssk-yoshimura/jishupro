## ESP32のプログラム起動の諸注意
- モータの電源を抜き、init_pinを繋げない状態で書き込む。
- リセットボタンを押してから、モータの電源を入れて、その後init_pinを繋げる。
- rosrun rosserial_...する。だめそうだったらリセットボタンからやり直す。
- 最悪、動画を見せる。

## Coral TPUの諸注意
- coralはPC（右側）に直接挿す。
- usbカメラはハブのPC側、esp32はハブの先側（横）に挿す。

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
