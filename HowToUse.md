# How to Use 

&nbsp; delivery_robotノードを使用したナビゲーションを行います。
※事前にgmappinng等で2D占有格子地図を作成しておき、ファイルの格納と、Navigation_Stackへの設定を行って下さい。

####（１）ロボット設定
・ロボットのナビゲーション時の挙動に関わる設定を編集します。以下コマンドを入力し、ファイルを編集します。既定値でも問題なく動作するため、必要に応じて変更して下さい。
```bash
$ cd ~/catkin_ws/src/delivery_robot/param
$ vi delivery_robot_node_tb3_01.yaml
```

![image](https://github.com/jadsys/DeliveryRobotNode/wiki/images/11.png)

　編集可能なパラメータを以下に記述します。
```bash
No.3「wp_sleep_time」は、とあるWPに到達し、次のWPに移動を開始するまでの時間を示します。
No.4「goal_tolerance_range」は、目標地点座標の誤差の範囲を示します。厳密にする場合は小さい値を、ある程度の誤差を許容する場合は大きな値を設定して下さい。
No.5「goal_allowable_range」は、ロボット停止タイマーを開始する範囲を指定します。目標地点付近で、誤差の影響でロボットが止まらない場合の対応として、タイマーを発行し、タイマーT.O.が発生したらロボットは目標地点に到達したと見なし、ロボットを停止させます。
No.6「goal_allowable_time」は、ロボット停止タイマーの時間を指定します。用途はNo.5と同様です。
No.7「goal_allowable_angle」は、目標地点の向きの誤差の範囲を示します。厳密にする場合は小さい値を、ある程度の誤差を許容する場合は大きな値を設定して下さい。
```
 

####（２）ロボットの電源を入れる
・ロボット、センサー、PCの電源を入れて下さい。

####（３）ロボットの移動
・付属のコントローラで、地図の初期位置にロボットを移動させます。
 
####（４）ターミナルの準備
・ナビゲーションを行うには、ロボット1台につき、計8つのターミナルにてコマンド入力を行う必要があります。①～⑥については、状態表示用PCからロボット制御用PCにssh接続します。
```bash
$ ssh azu001@[ロボット制御用PCのIPアドレス]
```
```bash
【ロボット制御用PCにssh接続するターミナル】
　　ターミナル①：roscore起動用
　　ターミナル②：turtlebot3_coreノード起動用
　　ターミナル③：Velodyneドライバノード起動用
　　ターミナル④：move_baseノード起動用
　　ターミナル⑤：delivery_robotノード起動用
```
```bash
【状態表示用PCで起動するターミナル】
　　ターミナル⑦：状態表示（rviz）ノード起動用
　　ターミナル⑧：緊急停止コマンド配信用
　　ターミナル⑨：移動指示用ノード起動用
```
####（５）ロボット側プログラムの起動
・（５）にて起動したターミナル上で以下のコマンドを入力し、プログラムを順次起動します。⑦はキャリブレーションの状態確認のため⑤の前に起動します。コマンド投入後のターミナルの画面を以下に添付致しますので、同じメッセージが表示されることをご確認下さい。
【ターミナル①】
```bash
$ roscore
```
![image](https://github.com/jadsys/DeliveryRobotNode/wiki/images/12.png)
 
【ターミナル②】
```bash
$ roslaunch turtlebot3_bringup turtlebot3_robot.launch
```
![image](https://github.com/jadsys/DeliveryRobotNode/wiki/images/13.png)

【ターミナル③】
```bash
$ roslaunch velodyne_pointcloud VLP16_points.launch
```
![image](https://github.com/jadsys/DeliveryRobotNode/wiki/images/14.png)
【ターミナル④】
```bash
$ roslaunch turtlebot3_navigation turtlebot3_navigation_L.launch
```
![image](https://github.com/jadsys/DeliveryRobotNode/wiki/images/15.png)

【ターミナル⑦】※⑤の前に起動します。
```bash
$ export ROS_HOSTNAME=[状態表示用PCのIPアドレス]
$ export ROS_MASTER_URI=http://[Turtlebot3制御用PCのIPアドレス]:11311
$ rosrun rviz rviz -d `rospack find turtlebot3_navigation`/rviz/egomap_viewer_tb3.rviz __name:=egomap_viewer_tb3 __ns:=turtlebot_01
```
![image](https://github.com/jadsys/DeliveryRobotNode/wiki/images/16.png)

【ターミナル⑤】
```bash
$ roslaunch delivery_robot delivery_manager_tb3_01.launch
```
![image](https://github.com/jadsys/DeliveryRobotNode/wiki/images/17.png)
　ロボット制御統括ノードが起動するとキャリブレーション（360~450°旋回）を行います。⑦で起動したrviz画面上で、ロボット付近の緑色の点群（パーティクル）が収束していることを確認します。「Standby…」メッセージの出力を以って、ナビゲーション準備完了となります。
![image](https://github.com/jadsys/DeliveryRobotNode/wiki/images/18.png)

【ターミナル⑧】
　緊急停止コマンド投入用のターミナルであるため、事前コマンドのみ入力しておきます。（緊急停止コマンドはターミナルに貼り付けておき、有事の際に「enter」ボタンを押下するだけにしておくとよろしいかと思います）

```bash
【事前コマンド】
$ export ROS_HOSTNAME=[状態表示用PCのIPアドレス]
$ export ROS_MASTER_URI=http://[Turtlebot3制御用PCのIPアドレス]:11311
```
```bash
【緊急停止コマンド】
【Turtlebot3の場合】
$ rostopic pub -r 10 /turtlebot_01/cmd_vel geometry_msgs/Twist  '{linear:  {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0,y: 0.0,z: 0.0}}'
```
####（６）移動指示用ノードの起動
・ナビゲーションを実行するための移動指示ノード起動用のターミナル⑨を起動します。

【事前コマンド】
```bash
$ export ROS_HOSTNAME=[状態表示用PCのIPアドレス]
$ export ROS_MASTER_URI=http://[Turtlebot3制御用PCのIPアドレス]:11311
```
・Remote PC Setup－（８）の項でプログラム修正した経路パターンを指定し、移動指示用ノードを起動します。
```bash
$ rosrun delivery_robot edge_node_beta turtlebot_01 turtlebot 113  // navi
```

　コマンドを入力すると以下のようなメッセージが出力され、ロボットにメッセージをTOPIC配信し、ロボットがナビゲーションを開始します。
![image](https://github.com/jadsys/DeliveryRobotNode/wiki/images/19.png)
 

ロボットがナビゲーションを終了すると、以下のようにカウンタを更新するメッセージのみ表示されるようになるので、「Ctrl+C」を押下し、コマンドを終了します。再度ナビゲーションを行う場合は、別途navi用のコマンドを入力して下さい。
![image](https://github.com/jadsys/DeliveryRobotNode/wiki/images/20.png)

####（７）ナビゲーション
・ダミークラウドノード用のターミナルから、navi用のコマンドを実行し、ロボットにナビゲーションを指示します。ロボットが移動指示メッセージを受信するとターミナル⑤に「Applying goal」のメッセージが出力され、ナビゲーションを開始します。

【ナビゲーション開始時のターミナル⑤の出力】
 ![image](https://github.com/jadsys/DeliveryRobotNode/wiki/images/21.png)

【ナビゲーション中のrvizの表示】
![image](https://github.com/jadsys/DeliveryRobotNode/wiki/images/22.png)
　　　　　目標地点は赤い矢印、経路は青い線で表示されます。

　ロボットがWPに到着すると、「way point Goal」のメッセージがターミナル⑤に出力されます。navi用コマンドに複数のWPが指定されている場合には、既定値で5秒静止した後（この停止時間は5.3（１）にて編集可能です）、次のWPに向かって移動を開始します。
![image](https://github.com/jadsys/DeliveryRobotNode/wiki/images/23.png)

　navi用コマンドに指定されたWPを全て移動し終えた場合は、以下のメッセージを出力し、ロボットは停止し、次の移動指示待ちの状態となります。「movebase GOAL(3)」のメッセージは正常時でも常時出力され続け、ログが流れてしまいますので、以下のメッセージはターミナルをスクロールしてご確認下さい。
![image](https://github.com/jadsys/DeliveryRobotNode/wiki/images/24.png)
 

####（８）プログラムの終了
・ナビゲーションを終了する場合、またバッテリー交換等で作業を終了する場合は、各プログラムを終了させます。⑨→①に「Ctrl+C」を押下し、プログラムを終了します。ロボット制御用PCにssh接続したターミナルで、以下コマンドを入力し、ロボット制御用PCをシャットダウンします。
```bash
$ sudo shutdown –h now
```
####（９）ロボットの電源を切る
・PC、センサー、ロボットの電源を順次切ります。


# Recovery procedure
&nbsp; ロボットが自己位置を失った際のリカバリ手順について、以下に記述します。

####（１）	緊急停止
・予め起動してあるターミナル⑧にて緊急停止用コマンドを入力します。

・緊急停止用コマンドを受信するとロボットは微動状態になりますので、次にターミナル④上で「Ctrl+C」を押下し、move_baseノードを停止します。

・move_baseノード停止により、ロボットは動作を停止します。ロボットの停止確認後、ターミナル⑨、⑤の順に「Ctrl+C」を押下し、edge_node_betaノード、delivery_robotノードを停止します。

　※伝送遅延やタイミングの問題で、停止が遅れる場合がございますため、オペレーターとは別の要員が、ロボットが壁や障害物にぶつからないようにロボットを持ち上げる、障害物とロボットの間に体を差し込むといった対応を取られた方が良いです。

####（２）リカバリ
・リカバリはプログラムの再起動となります。まずロボットを初期位置に移動させます。
　（手動搬送、コントローラによる移動、どちらでも問題ありません）

・プログラムをターミナル⑨→②まで「Ctrl+C」を押下して停止させます。

・停止させた後、ターミナル②から順次プログラムを起動します。
　キャリブレーションのためロボットが旋回行動を取りますので、誤動作と間違われないようお願い致します）
