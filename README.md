# Delivery Robot Node 

## <u>1. Overview</u>
&nbsp; Delivery Robot Nodeソフトウェアは、複数ロボットによる建物内自動搬送を目的とした2Dナビゲーションの実証実験用に開発しました。ROSのノードとして機能します。事前にROSのnavigation_stack（amcl+move_base）環境を準備し、delivery_robotノードを起動して、所定の書式に則った移動指示TOPICを遠隔監視のノードから配信することで、目的地への移動を行います。移動指示TOPICに複数の目的地を記述すれば、目的地を順に巡ります。<br>

![image](https://github.com/jadsys/DeliveryRobotNode/wiki/images/01.png)

## <u>2. System requirements</u>
### 2.1 Hardware
&nbsp; ロボット：ROS対応台車型ロボット（Turtlebot、メガローバー等）<br>
&nbsp; センサー：2D/3D Lider<br>
&nbsp; ロボット制御用PC：Barebone PC推奨<br>
&nbsp; 遠隔監視用PC	：指定なし<br>

【ハードウェア構成例】<br>
![image](https://github.com/jadsys/DeliveryRobotNode/wiki/images/02.png)


### 2.2 Software
&nbsp; OS：Ubuntu 16.04 LTS <br>
&nbsp; ROS：Kinetic<br>
&nbsp; Localization：amcl（Navigation_Stack）<br>
&nbsp; Navigation：move_base（Navigation_Stack）<br>
&nbsp; Odometory：Wheel＋Lider<br>
 
##<u>3. Software structure</u>
&nbsp; ソフトウェアはROSノードにより構成されます。ノード構成を以下に示します。

![image](https://github.com/jadsys/DeliveryRobotNode/wiki/images/03.png)

&nbsp; 複数ロボットに対応するため、ロボット毎に[Entity_Type]、[Entity_ID]という識別子を付与します。この識別子を使用して、同一ROSネットワーク内で複数ロボットを制御します。<br>
　　・Entity_Type	…ロボット名（例：turtlebot、megarover）<br>
　　・Entity_ID	…ロボット通番（例：turtlebot_01、megarover_02）<br>

　ROS NODE/TOPIC/TF名に識別子が入るよう、各種設定ファイルをご変更下さい。<br>
　　・ROS NODE…例：/[Entity_ID]/move_base、/[Entity_ID]/amcl<br>
　　・ROS TOPIC…例：/[Entity_ID]/cmd_vel、/[Entity_ID]/odom<br>
　　・ROS TF…例：[Entity_ID]_baselink、[Entity_ID]_base_footprint<br>
　
　以下にROS NODE/TOPIC/TFの命名例をrqt_graphとtf_treeにて示します。

![image](https://github.com/jadsys/DeliveryRobotNode/wiki/images/04.png)

![image](https://github.com/jadsys/DeliveryRobotNode/wiki/images/05.png)

## <u>4. Installations</u>
&nbsp;システム構築手順については  "[Installation Manual](InstallationManual.md)"をご参照下さい。 

## <u>5. How to Use</u>
&nbsp;使用方法については  "[How to use](HowToUse.md)"をご参照下さい。  

## <u>6. License</u>

[BSD](LICENSE)

