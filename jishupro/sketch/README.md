1/9夜

1/11試験後にやること


~~スケッチmotor_adjustでrotate_angleとvel2pulseを改良したのでwire_pullに移して試す~~ 1/10やった

~~set_angleは目標変更時にしか呼ばないことを確認する~~ 1/10 OK
- 実際は最初何度も呼んでいるがOK

~~常に位置制御をかけている状態になるのでRdata[i].thetaを初期値とかinit-poseの角度とかで初期化すること~~ 1/10 OK

~~delayを30にする~~ 1/10 OK
- pwmの周波数を上げてもいいかもしれない

サーボ0が動かないバグは直ったので軌道追従がましになってるはず

- プーリーなしの状態で試したら正常に軌道追従するようになった。スピードを下げるタイミングが早くなったので細かい軌道追従だとゆっくりになるかもしれない
- 速さのminの関数の傾きを大きくしてもいいかもしれないが一応これでも動く

motor_adjustでモーターの初期値を調整できるけどあんまり意味がないかも

軌道追従時の次の目標までの距離の閾値(is_near_enoughの不等号の値)をもっと大きくしてもいいかも
- 実際はis_nearestのほうで更新されてそう

↑この値をeusから送るようにして、閾値を変えて色々試す?
- 意味ないかも。やるならis_nearest

目標：1/11で軌道追従をそこそこちゃんとできるようにする！ →プーリーをつけて試す！

## wheel追加 1/11 ##
wheelを追加するにあたってwire_pullを変更した

wheel_motor_numを決める必要がある。

eusでワイヤー長の次にホイール角度[deg]を送るようにする必要がある。

- サーボ関連の定数の一部をサイズ10から12にした
- CB関数で、ワイヤー長の次にホイール角度を受け取るようにした
- wirelenの長さを12にした
- loopの軌道追従にwheel用の処理を追加
- rotate_angleをサーボ12個分処理するようにした
- servo_writeでch=10,11のときwheelを回すようにした
- init_enc_allrange, read_encでwheelのencを読むようにした

## 予定 ##

1/11(火) 軌道追従完成&wheel設計、加工
- 軌道追従完成、wheel設計完成
- esp32側のwheel用プログラムを作成

1/12(水) wheel加工、組立、wheel用のプログラム作成

1/13(木) 前進完成

1/14(金) 動きの追加、認識など

1/15,16 予備
