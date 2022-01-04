# mbotのeusモデル

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

## ワイヤー関数
```
(load "mbot-utils.l")
(send *mbot* :wire-init) ;; 初期化時に必ず呼ぶ
(send *mbot* :wire-calc) ;; ワイヤー長計算時に必ず呼ぶ
(draw-wire-end) ;; ワイヤー中継地点、関節などを描画
```
