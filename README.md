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
## esp32でrosを使う手順
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

ここでESP32の電源を入れる（重要）

少し待つと繋がる
```
[INFO] [1640525688.632853]: Established a socket connection from 192.168.1.17 on port 64560
[INFO] [1640525688.638527]: calling startSerialClient
...
```
