# 腱之介のeusモデル

eusでは```*mbot*```という名前

## 関節
本体側から順
- ```:rarm-shoulder-y```
- ```:rarm-shoulder-p```
- ```:rarm-elbow-p```
- ```:larm-shoulder-y```
- ```:larm-shoulder-p```
- ```:larm-elbow-p```

使用例

```
(send *mbot* :rarm-shoulder-y :joint-angle -30)
```

## 末端
- ```:rarm-end-coords```
- ```:larm-end-coords```

使用例

```
(send *mbot* :rarm-end-coords)
```
または
```
(send *mbot* :rarm :end-coords)
```

## ポーズ
- ```:zero-pose```
- ```:open-pose```
- ```:init-pose``` (サーボ・ワイヤーの初期化用）

使用例

```
(send *mbot* :init-pose)
```
## 関数
重要なもののみ

### ワイヤー関数
```
(load "mbot-utils.l")
(send *mbot* :wire-init) ;; 初期化時に必ず呼ぶ
(send *mbot* :wire-calc) ;; ワイヤー長計算時に必ず呼ぶ
(draw-wire-end) ;; ワイヤー中継地点、関節などを描画
```
### 軌道生成関数
```mbot-ri.l```の関数
```
mbot2ri-avs (avs wheels time_all sequence_mode &key thre_dist goal_offset)
```
引数：関節角度軌道、ホイールの角度軌道、時間、軌道追従の方式、収束判定の閾値、軌道追従のパラメータ

機能：関節角度軌道をワイヤー長軌道に変換し、ESP32に送る

```traj.l```の関数
```
mbot2ri-avs-ik (rtarget ltarget wrtarget wltarget time_all sequence_mode &key thre_dist goal_offset)
```
引数：手のik目標、ホイールの目標、時間、軌道追従の方式、収束判定の閾値、軌道追従のパラメータ

機能：目標をもとにikを解いて軌道を生成し、ESP32に送る

### デモ用の関数
```load "demo.l```でデモに必要な全てのeusプログラムがロードされる
- ```walk-prepare``` 歩行用の姿勢にする
- ```walk-prepare 1```歩行用の姿勢から初期状態へ戻す
- ```walk-both```歩行
- ```walk-back```後ろ向きに歩行
- ```rotate```回転
- ```yc```認識のAction Client（二回行う）
