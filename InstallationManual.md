# Installations 

## <u>1. Robot Setup</u>
&nbsp; システムの要件を満たすロボット、センサーをご準備下さい。<br>

## <u>2. Robot control PC Setup</u>
&nbsp; ロボット制御用のPCをセットアップします。ロボットはTurtlebot3、LiderはVelodyne-VLP16を例に手順を記述致します。
&nbsp; ※事前に時刻同期、ssh接続、オートログインの設定を行って下さい。

#### （１）パッケージの更新
　・ターミナルを起動し、 以下のコマンドを打ちます。<br>
```bash
$ sudo apt-get update
$ sudo apt-get upgrade
```

#### （２）ROS環境構築
　・以下のコマンドを入力し、ROSパッケージをインストールします。
```bash
$ sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
$ sudo apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
$ sudo apt-get update && sudo apt-get upgrade -y
$ sudo apt-get upgrade
$ sudo apt-get autoremove
$ sudo apt-get upgrade
$ sudo apt-get install ros-kinetic-desktop-full
$ sudo apt-get install ros-kinetic-rqt*
$ sudo apt-get install ros-kinetic-joystick-drivers
$ sudo rosdep init
$ rosdep update
$ sudo apt-get install python-rosinstall
$ source /opt/ros/kinetic/setup.bash
```

・ワークスペース用のディレクトリを作成し、ワークスペースの初期化を行います。本システムではワークスペース名を「~/catkin_ws」としておりますが、ご使用の環境に合わせて別名にして頂いても問題ありません。
```bash
$ mkdir -p ~/catkin_ws/src
$ cd ~/catkin_ws/src
$ catkin_init_workspace
$ cd ~/catkin_ws/
$ catkin_make
$ source ~/catkin_ws/devel/setup.bash 	
```
・ユーザ環境変数「~/.bashrc」に以下の内容を追記し、ログイン時に環境変数を有効にします。
```bash
$ gedit ~/.bashrc
```
```bash
【追加内容】
  # Set ROS Kinetic
  source /opt/ros/kinetic/setup.bash
  source ~/catkin_ws/devel/setup.bash

  # Set ROS Network
  export ROS_HOSTNAME=[PCのIPアドレス]
  export ROS_MASTER_URI=http://${ROS_HOSTNAME}:11311

  # Set ROS alias command
  alias cw=`cd ~/catkin_ws`
  alias cs=`cd ~/catkin_ws/src`
  alias cm=`cd ~/catkin_ws && catkin_make`

  export PATH=${PATH}:/opt/ros/kinetic/bin
  export TURTLEBOT3_MODEL=waffle_pi
```
・ターミナルを立ち上げ直すか、以下コマンド入力で、環境変数を有効化します。
```bash
$ source ~/.bashrc
```

#### （３）メタパッケージのインストール
・以下のコマンドを入力し、Turtlebot3関連パッケージをインストールします。
```bash
$ sudo apt-get install ros-kinetic-joy ros-kinetic-teleop-twist-joy ros-kinetic-teleop-twist-keyboard ros-kinetic-laser-proc ros-kinetic-rgbd-launch ros-kinetic-depthimage-to-laserscan ros-kinetic-rosserial-arduino ros-kinetic-rosserial-python ros-kinetic-rosserial-server ros-kinetic-rosserial-client ros-kinetic-rosserial-msgs ros-kinetic-amcl ros-kinetic-map-server ros-kinetic-move-base ros-kinetic-urdf ros-kinetic-xacro ros-kinetic-compressed-image-transport ros-kinetic-rqt-image-view ros-kinetic-gmapping ros-kinetic-navigation ros-kinetic-interactive-markers
$ cd ~/catkin_ws/src/
$ git clone https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git
$ git clone https://github.com/ROBOTIS-GIT/turtlebot3.git
$ git clone https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git
$ cd ~/catkin_ws && catkin_make
```

#### （４）pclパッケージのインストール
・以下のコマンドを入力し、点群データ処理用ライブラリ（Point Cloud Library）パッケージをインストールします。
```bash
$ sudo apt install pcl-tools
```

#### （５）Velodyneパッケージのインストール
・以下のコマンドを入力し、Velodyneパッケージをインストールします。
```bash
$ sudo apt-get remove ros-kinetic-velodyne*
$ sudo apt-get install ros-kinetic-velodyne
$ cd ~/catkin_ws/src
$ git clone https://github.com/ros-drivers/velodyne.git
$ cd ..
$ catkin_make -DCMAKE_BUILD_TYPE=Release
```

#### （６）Entity_Type／Entity_IDの設定
・ [Entity_Type]、[Entity_ID]を決定し、src/delivery_robot/param/ delivery_navi_node_tb3_01.yamlに反映します。

#### （７）ROS NODE／TOPIC／TF名の変更
・ROS NODE/TOPIC/TF名に識別子が入るよう、各種設定ファイルを変更します。configフォルダ配下にTurtlebot3とVelodyneのlaunch/yamlファイルを参考に格納しておりますので設定変更時、ご参考下さい。<br>
<font color="Red">※参考ファイルは添付しませんが、ロボット側のArduinoプログラムも同様の変更を行う必要がございます。Turtlebot3であれば、「turtlebot3_core.ino」の変更が必要です。</font>

#### （８）プログラムのリリースとビルド
・src/delivery_robotを[catkin_workspace]/src配下にリリースして下さい。<br>
・devel/include/delivery_robotを[catkin_workspace]/devel/include配下にリリースして下さい。<br>
・リリース後、以下コマンドを入力し、プログラムをビルドします。<br>
```bash
$ cd ~/catkin_ws
$ catkin_make
```

## <u>3. Remote PC Setup</u>
&nbsp; 遠隔監視用のPCをセットアップします。
&nbsp; ※事前に時刻同期、ssh接続の設定を行って下さい。

#### （１）パッケージの更新
・ターミナルを起動し、 以下のコマンドを打ちます。
```bash
$ sudo apt-get update  
$ sudo apt-get upgrade  
```

#### （２）ROS環境構築
・以下のコマンドを入力し、ROSパッケージをインストールします。
```bash
$ sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
$ sudo apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
$ sudo apt-get update && sudo apt-get upgrade -y
$ sudo apt-get upgrade
$ sudo apt-get autoremove
$ sudo apt-get upgrade
$ sudo apt-get install ros-kinetic-desktop-full
$ sudo apt-get install ros-kinetic-rqt*
$ sudo apt-get install ros-kinetic-joystick-drivers
$ sudo rosdep init
$ rosdep update
$ sudo apt-get install python-rosinstall
$ source /opt/ros/kinetic/setup.bash
```

・ワークスペース用のディレクトリを作成し、ワークスペースの初期化を行います。本システムではワークスペース名を「~/catkin_ws」としておりますが、ご使用の環境に合わせて別名にして頂いても問題ありません。
```bash
$ mkdir -p ~/catkin_ws/src
$ cd ~/catkin_ws/src
$ catkin_init_workspace
$ cd ~/catkin_ws/
$ catkin_make
$ source ~/catkin_ws/devel/setup.bash 	
```
・ユーザ環境変数「~/.bashrc」に以下の内容を追記し、ログイン時に環境変数を有効にします。「ROS_MASTER_URI」に指定するIPアドレスは、roscoreを起動するPCのIPアドレスを指定して下さい。
```bash
$ gedit ~/.bashrc
```

```bash
【追加内容】
  # Set ROS Kinetic
  source /opt/ros/kinetic/setup.bash
  source ~/catkin_ws/devel/setup.bash

  # Set ROS Network
  export ROS_HOSTNAME=[PCのIPアドレス]
  export ROS_MASTER_URI=http://[roscoreを起動するPCのIPアドレス]:11311

  # Set ROS alias command
  alias cw=`cd ~/catkin_ws`
  alias cs=`cd ~/catkin_ws/src`
  alias cm=`cd ~/catkin_ws && catkin_make`

  export PATH=${PATH}:/opt/ros/kinetic/bin
  export TURTLEBOT3_MODEL=waffle_pi
```
・ターミナルを立ち上げ直すか、以下コマンド入力で、環境変数を有効化します。
```bash
$ source ~/.bashrc
```

#### （３）メタパッケージのインストール
・以下のコマンドを入力し、ロボット関連パッケージをインストールします。
```bash
$ sudo apt-get install ros-kinetic-joy ros-kinetic-teleop-twist-joy ros-kinetic-teleop-twist-keyboard ros-kinetic-laser-proc ros-kinetic-rgbd-launch ros-kinetic-depthimage-to-laserscan ros-kinetic-rosserial-arduino ros-kinetic-rosserial-python ros-kinetic-rosserial-server ros-kinetic-rosserial-client ros-kinetic-rosserial-msgs ros-kinetic-amcl ros-kinetic-map-server ros-kinetic-move-base ros-kinetic-urdf ros-kinetic-xacro ros-kinetic-compressed-image-transport ros-kinetic-rqt-image-view ros-kinetic-gmapping ros-kinetic-navigation ros-kinetic-interactive-markers
$ cd ~/catkin_ws/src/
$ git clone https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git
$ git clone https://github.com/ROBOTIS-GIT/turtlebot3.git
$ git clone https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git
$ cd ~/catkin_ws && catkin_make
```
#### （４）Entity_Type／Entity_IDの設定
・[Entity_Type]、[Entity_ID]を決定し、src/delivery_robot/param/ delivery_robot_node_tb3_01.yamlに反映します。

#### （５）可視化ツールrvizの設定
・config/turtlebot3/turtlebot3_navigation/rvizフォルダ配下にrvizファイルを参考に格納しておりますので任意のフォルダにリリース頂き、ご使用下さい。

#### （６）プログラムのリリース
・src/delivery_robotを[catkin_workspace]/src配下にリリースして下さい。<br>
・devel/include/delivery_robotを[catkin_workspace]/devel/include配下にリリースして下さい。<br>

#### （７）WPの座標確認
・ナビゲーションを実施するに当たり、地図内の目標ポイント（WP）の座標をロボットに通知する必要があります。<br>
以下に会津大学様施設で実証実験を行った際に使用した地図を参考に示します。<br>
以下の図に示すアルファベットがWayPoint（目的地）を示しており、WayPointの座標を一覧化しておきます。<br>
地図の原点は、地図作成開始時点のロボット位置が原点となります。<br>
　
【WayPointを明示した地図】<br>
![image](https://github.com/jadsys/DeliveryRobotNode/wiki/images/06.png)

　【WayPoint一覧】<br>
![image](https://github.com/jadsys/DeliveryRobotNode/wiki/images/07.png)


・WayPointの座標調査は、Navigation_Stackを起動した状態で行います。起動したrvizにて以下の操作を行います。<br>
①rviz画面上部の「2D Pose Estimate」ボタンを押下する。<br>
![image](https://github.com/jadsys/DeliveryRobotNode/wiki/images/08.png)

②地図上で、座標を知りたいポイント上で左クリック（押したままを維持）し、緑色の矢印が出力されたら、矢印を任意の方角に向け、左クリックを離す。<br>
![image](https://github.com/jadsys/DeliveryRobotNode/wiki/images/09.png)
 
③Rvizを起動したターミナル上に、Informationとして「Setting Pose：[x] [y] [θ]」が出力されるため、こちらの座標をメモする。<br>
![image](https://github.com/jadsys/DeliveryRobotNode/wiki/images/10.png)

#### （８）移動指示用ノードプログラム修正
・移動指示用ノードプログラムには、会津大学様施設での実証実験用に取得した地図を元に参考経路をハードコーディングしております。<br>
（７）で整理したWayPoint一覧を元に同プログラムを修正して下さい。該当ソースは以下になります。
　
　【ソース名】<br>
　　ディレクトリパス：~/[catkin_ws]/src/delivery_robot/src<br>
　　ソース名　　　　：edge_node_beta.cpp<br>
　
以下にプログラム起動イメージを示します。第1引数に[Entity_ID]、第2引数に[Entity_Type]、第3引数に経路パターンを指定します。<br>
上記のプログラム内のswitch処理内で、実証実験で使用した経路パターン「113」,「114」を記述しておりますので、こちらの座標を修正して下さい。<br>

```bash
　【プログラム起動イメージ】
$ rosrun delivery_robot edge_node_beta turtlebot_01 turtlebot 113  // navi
$ rosrun delivery_robot edge_node_beta turtlebot_01 turtlebot 114  // Refresh（受付前）

$ rosrun delivery_robot edge_node_beta turtlebot_01 turtlebot 115  // navi（待機場所1）
$ rosrun delivery_robot edge_node_beta turtlebot_01 turtlebot 116  // navi（待機場所2）
$ rosrun delivery_robot edge_node_beta turtlebot_01 turtlebot 117  // refresh（待機場所1）
$ rosrun delivery_robot edge_node_beta turtlebot_01 turtlebot 118  // refresh（待機場所1） 

$ rosrun delivery_robot edge_node_beta turtlebot_01 turtlebot 888  // suspend
$ rosrun delivery_robot edge_node_beta turtlebot_01 turtlebot 777  // resume
$ rosrun delivery_robot edge_node_beta turtlebot_01 turtlebot 999  // emergency stop
```
```bash
　[指示種別]
　　・navi	…通常の移動指示
　　・refresh	…現在の移動指示命令を破棄し、新たに指定されたWPに向かう。
　　・suspend	…移動を中断し、その場で停止する。
　　・resume	…移動を再開する。
　　・emergency stop	…移動指示を破棄し、その場に停止する（再開不可）。
```

#### （９）プログラムのビルド
・プログラム修正後、以下コマンドを入力し、プログラムをビルドします。
```bash
$ cd ~/catkin_ws
$ catkin_make
 ```
