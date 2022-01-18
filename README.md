# 腱之介（けんのすけ）
自主プロ2021のために製作した、ワイヤー駆動ロボット

<img src=https://user-images.githubusercontent.com/93033349/149900406-44b384b8-ff05-49f0-ae49-7815ed4fadda.png width=500>

発表スライド：

https://github.com/ssk-yoshimura/jishupro/blob/master/kennosuke_.pdf

#### アピールポイント

- 6自由度を筋肉10本で動かすロボット（足回り2輪付き）
- 機構を工夫し、安いモータで大きなロボットを動かす（ロボットアーム長0.5m）
- IKを解き、関節角度軌道->ワイヤー長軌道を計算して追従&モータの協調動作を実現
- 画像認識も行っている

#### できること
- 歩行（前進、後退、回転）
- 認識（お手）

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

#### サーボの初期化
特定の姿勢の状態で、ワイヤーを手動で張り詰めた状態にする。このときのアブソリュートエンコーダの値を読み、サーボ角度とワイヤー長の関係を得る。

使用時は、まずこの初期化のプログラム実行してから、PCとの通信・制御等のプログラムを実行する。

# 発表実演手順
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

# リンク集
#### eusモデル、関数など
https://github.com/ssk-yoshimura/jishupro/tree/master/jishupro#readme
#### 諸注意など
https://github.com/ssk-yoshimura/jishupro/tree/master/jishupro/euslisp#readme
