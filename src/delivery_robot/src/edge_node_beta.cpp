// ros/ros.h　ROSに関する基本的なAPIのためのヘッダ
#include "ros/ros.h"
// comp_tutrial/adder.h　adder.msgから生成されたメッセージを定義しているヘッダ
//#include "comp_tutorial/adder.h"

#include "delivery_robot/r_command.h" // 移動指示メッセージ
#include "delivery_robot/r_result.h"  // 移動指示応答メッセージ(仮)
#include "delivery_robot/r_state.h"   // 状態報告メッセージ
#include "delivery_robot/r_emergency_command.h"  // 緊急停止メッセージ
#include "delivery_robot/r_emergency_result.h"   // 緊急停止応答メッセージ

#include <stdio.h>
#include <time.h>
#include <sys/time.h>



ros::Publisher para_pub;
ros::Publisher para_emg;



std::string iso8601ex(void)
{
    int ch;
    char iso_time[40];
    char time_zone[10];
    char dest[70];
    struct timeval myTime; 
    struct tm *time_st;  
    
    memset(iso_time, 0, sizeof(iso_time));
    memset(time_zone, 0, sizeof(time_zone));
    memset(dest, 0, sizeof(dest));
    
    gettimeofday(&myTime, NULL);
    time_st = localtime(&myTime.tv_sec);
    
    ch = strftime(iso_time, sizeof(iso_time)-1,"%FT%T", time_st);
    ch = strftime(time_zone, sizeof(time_zone)-1,"%z", time_st); 
    
    sprintf(dest, "%s.%03lu%s", iso_time, (myTime.tv_usec+500)/1000, time_zone);
    ROS_INFO("%s", dest);

    std::string time_str = dest;
//    ROS_INFO("%s", time_str.c_str());

    return( time_str );
}

void chatterCallback3(const delivery_robot::r_state msg)
{
//    ROS_INFO("id(%s) type(%s) time(%s) mode(%s) errors[0](%s) errors[1](%s)",msg.id.c_str(), msg.type.c_str(), msg.time.c_str(), msg.mode.c_str(), msg.errors[0].c_str(), msg.errors[1].c_str() );
    ROS_INFO("id(%s) type(%s) time(%s) mode(%s) errors.size()(%d)",msg.id.c_str(), msg.type.c_str(), msg.time.c_str(), msg.mode.c_str(), (int)msg.errors.size());
    for(int i=0; i<msg.errors.size(); i++){
        ROS_INFO("errors[%d](%s)", i, msg.errors[i].c_str());
    }
    ROS_INFO("x(%f) y(%f) z(%f) roll(%f) pitch(%f) yaw(%f) volt(%f)", msg.pose.point.x, msg.pose.point.y, msg.pose.point.z, msg.pose.angle.roll, msg.pose.angle.pitch, msg.pose.angle.yaw, msg.battery.voltage);

    ROS_INFO("current_optional.valid(%s) current(%f)",(msg.battery.current_optional.valid == true)? "true":"false", msg.battery.current_optional.current); 
}

int main(int argc, char **argv)
{
  //Publisherとしての定義
  // n.advertise<comp_tutorial::adder>("para_input", 1000);
  // comp_tutorial::adder型のメッセージをpara_inputというトピックへ配信する
  //"1000"はトピックキューの最大値

    int mode = 0;
    std::string entity_id = "default_id";
    std::string entity_type = "default_type";
    std::string str_mode = "mode_no";

    ROS_INFO("argc=%i" , argc ); 

    if( argc >= 2 ) entity_id = argv[1];
    if( argc >= 3 ) entity_type = argv[2];
    if( argc >= 4 ) mode = atoi(argv[3]);
    if( argc >= 4 ) str_mode = argv[3];

    ROS_INFO("entity_id:%s", entity_id.c_str());
    ROS_INFO("entity_type:%s", entity_type.c_str());
    ROS_INFO("mode:%i", mode ); 

  // 初期化のためのAPI
  ros::init(argc, argv, "edge" + entity_id + str_mode);

  // ノードハンドラの宣言
  ros::NodeHandle n;

#if 0 
  ros::Publisher para_pub = n.advertise<comp_tutorial::adder>("para_input", 1000);

  //1秒間に1つのメッセージをPublishする
  ros::Rate loop_rate(1);

  //comp_tutrial::adder型のオブジェクトを定義
  //adder.msgで定義したa,bはメンバ変数としてアクセスできる
  comp_tutorial::adder msg;

  int count = 0;
  while (ros::ok())//ノードが実行中は基本的にros::ok()=1
  {
    msg.a = count;
    msg.b = count;
    para_pub.publish(msg);//PublishのAPI
    printf("a = %d b = %d \n",msg.a , msg.b );
    ros::spinOnce();
    loop_rate.sleep();
    count++;
  }
#else
//  para_pub = n.advertise<delivery_robot::r_command>("/robot_bridge/turtlebot3_waffle_01/cmd", 100, true);
//  para_emg = n.advertise<delivery_robot::r_emergency_command>("/robot_bridge/turtlebot3_waffle_01/emg", 100, true);

    para_pub = n.advertise<delivery_robot::r_command>("/robot_bridge/"+ entity_id +"/cmd", 100, true);
    para_emg = n.advertise<delivery_robot::r_emergency_command>("/robot_bridge/"+ entity_id +"/emg", 100, true);

// tst
    std::string tst = "/robot_bridge/"+ entity_id +"/cmd";
    ROS_INFO("tst:%s", tst.c_str()); 

// tst

    //delivery_robot::r_req型のオブジェクトを定義
    delivery_robot::r_command msg;
    delivery_robot::r_emergency_command emg_msg; // 緊急停止コマンド

    msg.id = entity_id;
    msg.type = entity_type;
    msg.time = iso8601ex();

    if(mode == 999 || mode == 888 || mode == 777){
        // 緊急停止コマンド
        emg_msg.id   = msg.id;
        emg_msg.type = msg.type;
        emg_msg.time = msg.time;
        if(mode == 999){
           emg_msg.emergency_cmd = "stop";
        }else
        if(mode == 888){
           emg_msg.emergency_cmd = "suspend";
        }else{
           emg_msg.emergency_cmd = "resume";
        }
    }

#if 1 // 0830_3 社内テスト
int wIdx=0;
  switch(mode){

  case 1:// navi
    msg.cmd = "navi";

    msg.waypoints.resize(0);
    break;

  case 11:// navi   
    msg.cmd = "navi";

    msg.waypoints.resize(4);
    msg.waypoints[wIdx].point.x = 0.00;         // 真ん中
    msg.waypoints[wIdx].point.y = 0.00;
    msg.waypoints[wIdx].point.z = 0.0;
    msg.waypoints[wIdx].angle_optional.valid = false;
    msg.waypoints[wIdx].angle_optional.angle.yaw = 0.00;
    wIdx++;

    msg.waypoints[wIdx].point.x = -0.35;        // カラーbox前
    msg.waypoints[wIdx].point.y = 1.19;
    msg.waypoints[wIdx].point.z = 0.0;
    msg.waypoints[wIdx].angle_optional.valid = true;
    msg.waypoints[wIdx].angle_optional.angle.yaw = 3.14;
    wIdx++;

    msg.waypoints[wIdx].point.x = 0.00;         // 真ん中
    msg.waypoints[wIdx].point.y = 0.00;
    msg.waypoints[wIdx].point.z = 0.0;
    msg.waypoints[wIdx].angle_optional.valid = false;
    msg.waypoints[wIdx].angle_optional.angle.yaw = 0.00;
    wIdx++;

    msg.waypoints[wIdx].point.x = -0.29;        // 開始位置
    msg.waypoints[wIdx].point.y = -1.53;
    msg.waypoints[wIdx].point.z = 0.0;
    msg.waypoints[wIdx].angle_optional.valid = true;
    msg.waypoints[wIdx].angle_optional.angle.yaw = 1.57;
    wIdx++;


    break;

  case 12:// navi
    msg.cmd = "navi";

    msg.waypoints.resize(2);
    msg.waypoints[wIdx].point.x = 0.00;     // 真ん中
    msg.waypoints[wIdx].point.y = 0.00;
    msg.waypoints[wIdx].point.z = 0.0;
    msg.waypoints[wIdx].angle_optional.valid = true;
    msg.waypoints[wIdx].angle_optional.angle.yaw = 0.01;
    wIdx++;
    msg.waypoints[wIdx].point.x = -0.29;    // 開始位置
    msg.waypoints[wIdx].point.y = -1.53;
    msg.waypoints[wIdx].point.z = 0.0;
    msg.waypoints[wIdx].angle_optional.valid = true;
    msg.waypoints[wIdx].angle_optional.angle.yaw = 1.57;
    wIdx++;
    break;

  case 13:// navi
    msg.cmd = "navi";

    msg.waypoints.resize(4);

    msg.waypoints[wIdx].point.x = 5.389;       // 棚前
    msg.waypoints[wIdx].point.y = 0.217;
    msg.waypoints[wIdx].point.z = 0.0;
    msg.waypoints[wIdx].angle_optional.valid = true;
    msg.waypoints[wIdx].angle_optional.angle.yaw = -1.57; // 北向き
    wIdx++;

    msg.waypoints[wIdx].point.x = 0.00;     // 真ん中
    msg.waypoints[wIdx].point.y = 0.00;
    msg.waypoints[wIdx].point.z = 0.0;
    msg.waypoints[wIdx].angle_optional.valid = false;
    msg.waypoints[wIdx].angle_optional.angle.yaw = 0.00;
    wIdx++;

    msg.waypoints[wIdx].point.x = -0.28;        // 本棚前
    msg.waypoints[wIdx].point.y = 2.39;
    msg.waypoints[wIdx].point.z = 0.0;
    msg.waypoints[wIdx].angle_optional.valid = true;
    msg.waypoints[wIdx].angle_optional.angle.yaw = 3.14;
    wIdx++;

    msg.waypoints[wIdx].point.x = 0.00;     // 真ん中
    msg.waypoints[wIdx].point.y = 0.00;
    msg.waypoints[wIdx].point.z = 0.0;
    msg.waypoints[wIdx].angle_optional.valid = true;
    msg.waypoints[wIdx].angle_optional.angle.yaw = 0.00;    // 西向き
    wIdx++;

    break;

  case 14:// navi
    msg.cmd = "navi";

    msg.waypoints.resize(5);

    msg.waypoints[wIdx].point.x = -0.29;    // 開始位置
    msg.waypoints[wIdx].point.y = -1.53;
    msg.waypoints[wIdx].point.z = 0.0;
    msg.waypoints[wIdx].angle_optional.valid = true;
    msg.waypoints[wIdx].angle_optional.angle.yaw = -1.57;
    wIdx++;
#if 0
    msg.waypoints[wIdx].point.x = 0.00;     // 真ん中
    msg.waypoints[wIdx].point.y = 0.00;
    msg.waypoints[wIdx].point.z = 0.0;
    msg.waypoints[wIdx].angle_optional.valid = false;
    msg.waypoints[wIdx].angle_optional.angle.yaw = 0.00;
    wIdx++;
#endif
    msg.waypoints[wIdx].point.x = -0.28;        // 本棚前
    msg.waypoints[wIdx].point.y = 2.39;
    msg.waypoints[wIdx].point.z = 0.0;
    msg.waypoints[wIdx].angle_optional.valid = true;
    msg.waypoints[wIdx].angle_optional.angle.yaw = 1.57;
    wIdx++;

    msg.waypoints[wIdx].point.x = 0.00;     // 真ん中
    msg.waypoints[wIdx].point.y = 0.00;
    msg.waypoints[wIdx].point.z = 0.0;
    msg.waypoints[wIdx].angle_optional.valid = false;
    msg.waypoints[wIdx].angle_optional.angle.yaw = 0.00;    // 西向き
    wIdx++;

    msg.waypoints[wIdx].point.x = 5.389;       // 棚前
    msg.waypoints[wIdx].point.y = 0.217;
    msg.waypoints[wIdx].point.z = 0.0;
    msg.waypoints[wIdx].angle_optional.valid = true;
    msg.waypoints[wIdx].angle_optional.angle.yaw = 0.00; // 西向き
    wIdx++;

    msg.waypoints[wIdx].point.x = 0.00;     // 真ん中
    msg.waypoints[wIdx].point.y = 0.00;
    msg.waypoints[wIdx].point.z = 0.0;
    msg.waypoints[wIdx].angle_optional.valid = true;
    msg.waypoints[wIdx].angle_optional.angle.yaw = 0.00;    // 西向き
    wIdx++;

    break;

  case 113:// navi   Lictia
    msg.cmd = "navi";

    msg.waypoints.resize(13);
    msg.waypoints[wIdx].point.x = -0.70;    // 中央経由地
    msg.waypoints[wIdx].point.y = -1.99;
    msg.waypoints[wIdx].point.z = 0.0;
    msg.waypoints[wIdx].angle_optional.valid = false;
    msg.waypoints[wIdx].angle_optional.angle.yaw = 0.00;
    wIdx++;

    msg.waypoints[wIdx].point.x = 3.71;    // 事務室
    msg.waypoints[wIdx].point.y = -1.99;
    msg.waypoints[wIdx].point.z = 0.0;
    msg.waypoints[wIdx].angle_optional.valid = true;
    msg.waypoints[wIdx].angle_optional.angle.yaw = 1.57;
    wIdx++;

    msg.waypoints[wIdx].point.x = -0.70;    // 中央経由地
    msg.waypoints[wIdx].point.y = -1.99;
    msg.waypoints[wIdx].point.z = 0.0;
    msg.waypoints[wIdx].angle_optional.valid = false;
    msg.waypoints[wIdx].angle_optional.angle.yaw = 0.00;
    wIdx++;

    msg.waypoints[wIdx].point.x = -1.66;    // カフェ前
    msg.waypoints[wIdx].point.y = -6.68;
    msg.waypoints[wIdx].point.z = 0.0;
    msg.waypoints[wIdx].angle_optional.valid = false;
    msg.waypoints[wIdx].angle_optional.angle.yaw = 0.0;
    wIdx++;

    msg.waypoints[wIdx].point.x = 1.55;    // 個室前経由
    msg.waypoints[wIdx].point.y = -7.78;
    msg.waypoints[wIdx].point.z = 0.0;
    msg.waypoints[wIdx].angle_optional.valid = false;
    msg.waypoints[wIdx].angle_optional.angle.yaw = 0.0;
    wIdx++;

    msg.waypoints[wIdx].point.x = 1.51;    // 個室
    msg.waypoints[wIdx].point.y = -9.70;
    msg.waypoints[wIdx].point.z = 0.0;
    msg.waypoints[wIdx].angle_optional.valid = true;
    msg.waypoints[wIdx].angle_optional.angle.yaw = -1.58;
    wIdx++;

    msg.waypoints[wIdx].point.x = 1.55;    // 個室前経由
    msg.waypoints[wIdx].point.y = -7.78;
    msg.waypoints[wIdx].point.z = 0.0;
    msg.waypoints[wIdx].angle_optional.valid = false;
    msg.waypoints[wIdx].angle_optional.angle.yaw = 0.0;
    wIdx++;

#if 1 // 会議室1
    msg.waypoints[wIdx].point.x = 10.60;    // 会議室1
    msg.waypoints[wIdx].point.y = -8.00;
    msg.waypoints[wIdx].point.z = 0.0;
    msg.waypoints[wIdx].angle_optional.valid = false;
    msg.waypoints[wIdx].angle_optional.angle.yaw = 0.00;
    wIdx++;

    msg.waypoints[wIdx].point.x = 10.63;    // 会議室1   中
    msg.waypoints[wIdx].point.y = -9.08;
    msg.waypoints[wIdx].point.z = 0.0;
    msg.waypoints[wIdx].angle_optional.valid = true;
    msg.waypoints[wIdx].angle_optional.angle.yaw = -1.57;
    wIdx++;

    msg.waypoints[wIdx].point.x = 10.60;    // 会議室1
    msg.waypoints[wIdx].point.y = -8.00;
    msg.waypoints[wIdx].point.z = 0.0;
    msg.waypoints[wIdx].angle_optional.valid = false;
    msg.waypoints[wIdx].angle_optional.angle.yaw = 0.00;
    wIdx++;
#endif

    msg.waypoints[wIdx].point.x = 1.55;    // 個室前経由
    msg.waypoints[wIdx].point.y = -7.78;
    msg.waypoints[wIdx].point.z = 0.0;
    msg.waypoints[wIdx].angle_optional.valid = false;
    msg.waypoints[wIdx].angle_optional.angle.yaw = 0.0;
    wIdx++;

    msg.waypoints[wIdx].point.x = -1.66;    // カフェ前
    msg.waypoints[wIdx].point.y = -6.68;
    msg.waypoints[wIdx].point.z = 0.0;
    msg.waypoints[wIdx].angle_optional.valid = false;
    msg.waypoints[wIdx].angle_optional.angle.yaw = 0.0;
    wIdx++;

    msg.waypoints[wIdx].point.x = 0.00;    // 受付前
    msg.waypoints[wIdx].point.y = 0.00;
    msg.waypoints[wIdx].point.z = 0.0;
    msg.waypoints[wIdx].angle_optional.valid = true;
    msg.waypoints[wIdx].angle_optional.angle.yaw = 0.0;
    wIdx++;

    break;

  case 114:// reflesh   Lictia
    msg.cmd = "refresh";

    msg.waypoints.resize(2);
    msg.waypoints[wIdx].point.x = -0.70;    // 中央経由地
    msg.waypoints[wIdx].point.y = -1.99;
    msg.waypoints[wIdx].point.z = 0.0;
    msg.waypoints[wIdx].angle_optional.valid = false;
    msg.waypoints[wIdx].angle_optional.angle.yaw = 0.00;
    wIdx++;

    msg.waypoints[wIdx].point.x = 0.00;    // 受付前
    msg.waypoints[wIdx].point.y = 0.00;
    msg.waypoints[wIdx].point.z = 0.0;
    msg.waypoints[wIdx].angle_optional.valid = true;
    msg.waypoints[wIdx].angle_optional.angle.yaw = 0.00;
    wIdx++;

    break; 

  case 115:// navi   Lictia 待機場所1
    msg.cmd = "navi";

    msg.waypoints.resize(1);
    msg.waypoints[wIdx].point.x = -0.40;    // 待機場所
    msg.waypoints[wIdx].point.y = 1.50;
    msg.waypoints[wIdx].point.z = 0.0;
    msg.waypoints[wIdx].angle_optional.valid = true;
    msg.waypoints[wIdx].angle_optional.angle.yaw = -1.57;
    wIdx++;
    break; 

  case 116:// navi   Lictia 待機場所2
    msg.cmd = "navi";

    msg.waypoints.resize(1);
    msg.waypoints[wIdx].point.x = -1.50;    // 待機場所2
    msg.waypoints[wIdx].point.y = 1.50;
    msg.waypoints[wIdx].point.z = 0.0;
    msg.waypoints[wIdx].angle_optional.valid = true;
    msg.waypoints[wIdx].angle_optional.angle.yaw = -1.57;
    wIdx++;
    break; 


  case 117:// refresh   Lictia 待機場所1
    msg.cmd = "refresh";

    msg.waypoints.resize(1);
    msg.waypoints[wIdx].point.x = -0.40;    // 待機場所
    msg.waypoints[wIdx].point.y = 1.50;
    msg.waypoints[wIdx].point.z = 0.0;
    msg.waypoints[wIdx].angle_optional.valid = true;
    msg.waypoints[wIdx].angle_optional.angle.yaw = -1.57;
    wIdx++;
    break; 

  case 118:// refresh   Lictia 待機場所2
    msg.cmd = "refresh";

    msg.waypoints.resize(1);
    msg.waypoints[wIdx].point.x = -1.50;    // 待機場所2
    msg.waypoints[wIdx].point.y = 1.50;
    msg.waypoints[wIdx].point.z = 0.0;
    msg.waypoints[wIdx].angle_optional.valid = true;
    msg.waypoints[wIdx].angle_optional.angle.yaw = -1.57;
    wIdx++;
    break; 

  case 119:// navi   Lictia wayポイント一周
    msg.cmd = "navi";

    msg.waypoints.resize(23);
    msg.waypoints[wIdx].point.x = 0.0;    // LA 事務室 Token取得ポイント
    msg.waypoints[wIdx].point.y = -1.50;
    msg.waypoints[wIdx].point.z = 0.0;
    msg.waypoints[wIdx].angle_optional.valid = true;
    msg.waypoints[wIdx].angle_optional.angle.yaw = -0.46;
    wIdx++;

    msg.waypoints[wIdx].point.x = 0.50;    // R  中央経由地点2
    msg.waypoints[wIdx].point.y = -2.00;
    msg.waypoints[wIdx].point.z = 0.0;
    msg.waypoints[wIdx].angle_optional.valid = false;
    msg.waypoints[wIdx].angle_optional.angle.yaw = 0.0;
    wIdx++;

    msg.waypoints[wIdx].point.x = 3.71;   // L  事務室
    msg.waypoints[wIdx].point.y = -1.99;
    msg.waypoints[wIdx].point.z = 0.0;
    msg.waypoints[wIdx].angle_optional.valid = true;
    msg.waypoints[wIdx].angle_optional.angle.yaw = -1.57;
    wIdx++;

    msg.waypoints[wIdx].point.x = 0.0;   // RA  事務室 Token解放ポイント
    msg.waypoints[wIdx].point.y = -2.50;
    msg.waypoints[wIdx].point.z = 0.0;
    msg.waypoints[wIdx].angle_optional.valid = true;
    msg.waypoints[wIdx].angle_optional.angle.yaw = -1.80;
    wIdx++;

    msg.waypoints[wIdx].point.x = -0.94;   // S  カフエ前2
    msg.waypoints[wIdx].point.y = -5.90;
    msg.waypoints[wIdx].point.z = 0.0;
    msg.waypoints[wIdx].angle_optional.valid = false;
    msg.waypoints[wIdx].angle_optional.angle.yaw = 0.0;
    wIdx++;

    msg.waypoints[wIdx].point.x = 0.70		;   // LB  個室 Token取得ポイント
    msg.waypoints[wIdx].point.y = -7.40;
    msg.waypoints[wIdx].point.z = 0.0;
    msg.waypoints[wIdx].angle_optional.valid = true;
    msg.waypoints[wIdx].angle_optional.angle.yaw = -0.15;
    wIdx++;

    msg.waypoints[wIdx].point.x = 1.50	;   // T  個室前経由地2
    msg.waypoints[wIdx].point.y = -7.50;
    msg.waypoints[wIdx].point.z = 0.0;
    msg.waypoints[wIdx].angle_optional.valid = false;
    msg.waypoints[wIdx].angle_optional.angle.yaw = 0.0;
    wIdx++;

    msg.waypoints[wIdx].point.x = 1.51	;   // G  個室
    msg.waypoints[wIdx].point.y = -9.70;
    msg.waypoints[wIdx].point.z = 0.0;
    msg.waypoints[wIdx].angle_optional.valid = true;
    msg.waypoints[wIdx].angle_optional.angle.yaw = -1.57;
    wIdx++;

    msg.waypoints[wIdx].point.x = 1.50	;   // T  個室前経由地2
    msg.waypoints[wIdx].point.y = -7.50;
    msg.waypoints[wIdx].point.z = 0.0;
    msg.waypoints[wIdx].angle_optional.valid = false;
    msg.waypoints[wIdx].angle_optional.angle.yaw = 0.0;
    wIdx++;

    msg.waypoints[wIdx].point.x = 2.30;   // RB  個室 Token 解除ポイント
    msg.waypoints[wIdx].point.y = -7.50;
    msg.waypoints[wIdx].point.z = 0.0;
    msg.waypoints[wIdx].angle_optional.valid = true;
    msg.waypoints[wIdx].angle_optional.angle.yaw = 0.05;
    wIdx++;

    msg.waypoints[wIdx].point.x = 4.90;   // LC  会議室 Token  取得ポイント
    msg.waypoints[wIdx].point.y = -7.30;
    msg.waypoints[wIdx].point.z = 0.0;
    msg.waypoints[wIdx].angle_optional.valid = true;
    msg.waypoints[wIdx].angle_optional.angle.yaw = 0.00;
    wIdx++;

    msg.waypoints[wIdx].point.x = 5.30;   // U  一時、待機場所(目的地-制限エリア)
    msg.waypoints[wIdx].point.y = -7.15;
    msg.waypoints[wIdx].point.z = 0.0;
    msg.waypoints[wIdx].angle_optional.valid = true;
    msg.waypoints[wIdx].angle_optional.angle.yaw = -1.57;
    wIdx++;

#if 1   // 会議室3
    msg.waypoints[wIdx].point.x = 6.06;    // 会議室3
    msg.waypoints[wIdx].point.y = -8.00;
    msg.waypoints[wIdx].point.z = 0.0;
    msg.waypoints[wIdx].angle_optional.valid = false;
    msg.waypoints[wIdx].angle_optional.angle.yaw = 0.00;
    wIdx++;

    msg.waypoints[wIdx].point.x = 5.96;    // 会議室3   中
    msg.waypoints[wIdx].point.y = -9.09;
    msg.waypoints[wIdx].point.z = 0.0;
    msg.waypoints[wIdx].angle_optional.valid = true;
    msg.waypoints[wIdx].angle_optional.angle.yaw = -1.57;
    wIdx++;

    msg.waypoints[wIdx].point.x = 6.06;    // 会議室3
    msg.waypoints[wIdx].point.y = -8.00;
    msg.waypoints[wIdx].point.z = 0.0;
    msg.waypoints[wIdx].angle_optional.valid = false;
    msg.waypoints[wIdx].angle_optional.angle.yaw = 0.00;
    wIdx++;
#endif

#if 1 // 会議室1
    msg.waypoints[wIdx].point.x = 10.60;    // 会議室1
    msg.waypoints[wIdx].point.y = -8.00;
    msg.waypoints[wIdx].point.z = 0.0;
    msg.waypoints[wIdx].angle_optional.valid = false;
    msg.waypoints[wIdx].angle_optional.angle.yaw = 0.00;
    wIdx++;

    msg.waypoints[wIdx].point.x = 10.73;    // 会議室1   中
    msg.waypoints[wIdx].point.y = -9.08;
    msg.waypoints[wIdx].point.z = 0.0;
    msg.waypoints[wIdx].angle_optional.valid = true;
    msg.waypoints[wIdx].angle_optional.angle.yaw = -1.57;
    wIdx++;

    msg.waypoints[wIdx].point.x = 10.60;    // 会議室1
    msg.waypoints[wIdx].point.y = -8.00;
    msg.waypoints[wIdx].point.z = 0.0;
    msg.waypoints[wIdx].angle_optional.valid = false;
    msg.waypoints[wIdx].angle_optional.angle.yaw = 0.00;
    wIdx++;
#endif

    msg.waypoints[wIdx].point.x = 4.0;   // RC  会議室 Token  解放ポイント
    msg.waypoints[wIdx].point.y = -8.00;
    msg.waypoints[wIdx].point.z = 0.0;
    msg.waypoints[wIdx].angle_optional.valid = true;
    msg.waypoints[wIdx].angle_optional.angle.yaw = 3.14;
    wIdx++;

    msg.waypoints[wIdx].point.x = 0.20;   // F  個室前経由地
    msg.waypoints[wIdx].point.y = -8.10;
    msg.waypoints[wIdx].point.z = 0.0;
    msg.waypoints[wIdx].angle_optional.valid = false;
    msg.waypoints[wIdx].angle_optional.angle.yaw = 0.0;
    wIdx++;

    msg.waypoints[wIdx].point.x = -1.66;   // E  カフェ前
    msg.waypoints[wIdx].point.y = -6.68;
    msg.waypoints[wIdx].point.z = 0.0;
    msg.waypoints[wIdx].angle_optional.valid = false;
    msg.waypoints[wIdx].angle_optional.angle.yaw = 0.0;
    wIdx++;

    msg.waypoints[wIdx].point.x = -1.00;   // B  中央経由地点
    msg.waypoints[wIdx].point.y = -2.50;
    msg.waypoints[wIdx].point.z = 0.0;
    msg.waypoints[wIdx].angle_optional.valid = false;
    msg.waypoints[wIdx].angle_optional.angle.yaw = 0.0;
    wIdx++;

    msg.waypoints[wIdx].point.x = -0.40;   // Q  待機場所
    msg.waypoints[wIdx].point.y = 1.50;
    msg.waypoints[wIdx].point.z = 0.0;
    msg.waypoints[wIdx].angle_optional.valid = true;
    msg.waypoints[wIdx].angle_optional.angle.yaw = -1.57;
    wIdx++;

    break; 


case 120:// navi   Lictia wayポイント
    msg.cmd = "navi";

    msg.waypoints.resize(4);
    msg.waypoints[wIdx].point.x = 0.50;    // R  中央経由地点2
    msg.waypoints[wIdx].point.y = -2.00;
    msg.waypoints[wIdx].point.z = 0.0;
    msg.waypoints[wIdx].angle_optional.valid = false;
    msg.waypoints[wIdx].angle_optional.angle.yaw = 0.0;
    wIdx++;

    msg.waypoints[wIdx].point.x = -0.94;   // S  カフエ前2
    msg.waypoints[wIdx].point.y = -5.90;
    msg.waypoints[wIdx].point.z = 0.0;
    msg.waypoints[wIdx].angle_optional.valid = false;
    msg.waypoints[wIdx].angle_optional.angle.yaw = 0.0;
    wIdx++;

    msg.waypoints[wIdx].point.x = -1.00;   // B  中央経由地点
    msg.waypoints[wIdx].point.y = -2.50;
    msg.waypoints[wIdx].point.z = 0.0;
    msg.waypoints[wIdx].angle_optional.valid = false;
    msg.waypoints[wIdx].angle_optional.angle.yaw = 0.0;
    wIdx++;

    msg.waypoints[wIdx].point.x = -0.40;   // Q  待機場所
    msg.waypoints[wIdx].point.y = 1.50;
    msg.waypoints[wIdx].point.z = 0.0;
    msg.waypoints[wIdx].angle_optional.valid = true;
    msg.waypoints[wIdx].angle_optional.angle.yaw = -1.57;
    wIdx++;

    break; 


  case 100:// navi
    msg.cmd = "navi";

    msg.waypoints.resize(1);
    msg.waypoints[wIdx].point.x = 0.00;
    msg.waypoints[wIdx].point.y = 0.00;
    msg.waypoints[wIdx].point.z = 0.0;
    wIdx++;
    break; 

  case 2:// refresh
    msg.cmd = "refresh";

    msg.waypoints.resize(1);// 開始位置
    msg.waypoints[wIdx].point.x = 0.00;     // 真ん中
    msg.waypoints[wIdx].point.y = 0.00;
    msg.waypoints[wIdx].point.z = 0.0;
    msg.waypoints[wIdx].angle_optional.valid = true;
    msg.waypoints[wIdx].angle_optional.angle.yaw = 0.00;
    wIdx++;

    break; 

  case 22:// refresh
    msg.cmd = "refresh";

    msg.waypoints.resize(1);
    msg.waypoints[wIdx].point.x = -0.29;
    msg.waypoints[wIdx].point.y = -1.53;
    msg.waypoints[wIdx].point.z = 0.0;
    msg.waypoints[wIdx].angle_optional.valid = true;
    msg.waypoints[wIdx].angle_optional.angle.yaw = 3.14;
    wIdx++;
    break; 

  default:// standby
    msg.cmd = "standby";

    msg.waypoints.resize(0);
    break;
  }
#endif

    if(mode == 999 || mode == 888 || mode == 777){
        // 緊急停止     サスペンド      レジューム
        ROS_INFO("emergency_command id(%s) type(%s) time(%s) cmd(%s)", emg_msg.id.c_str(), emg_msg.type.c_str(), emg_msg.time.c_str(), emg_msg.emergency_cmd.c_str() );
        para_emg.publish(emg_msg);
    }else{
        //
        ROS_INFO("id(%s) type(%s) time(%s) cmd(%s)",msg.id.c_str(), msg.type.c_str(), msg.time.c_str(), msg.cmd.c_str() );
        int num=msg.waypoints.size(); 
        for(int i=0;i<num;i++){
            ROS_INFO("[%i]: x(%f) y(%f) z(%f)", i, msg.waypoints[i].point.x, msg.waypoints[i].point.y, msg.waypoints[i].point.z);
        }
        para_pub.publish(msg);//PublishのAPI
    }

    ros::Rate rate(30);
    int cnt = 30;
    while( (cnt--) > 0 )//ノードが実行中は基本的にros::ok()=1
    {
      rate.sleep();   // スリープ
      ros::spinOnce();// コールバック処理
    }

    ros::Subscriber sub2 = n.subscribe("/robot_bridge/"+ entity_id +"/state", 100, chatterCallback3);

    ros::Rate loop_rate(1);
    int count = 0;
    while (ros::ok())//ノードが実行中は基本的にros::ok()=1
    {
      loop_rate.sleep();// スリープ
      ros::spinOnce();// コールバック処理
      printf("count = %d\n",count);
      count++;
    }

#endif
  return 0;
}




