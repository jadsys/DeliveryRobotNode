#include <iostream>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_listener.h>

#include <ros/console.h> // ログデバッグ出力用

#ifndef M_PI
#define M_PI 3.14159265358979             // 円周率
#endif
#define DEG2RAD(x) ((x)*M_PI/180)  // 度からラジアン
#define RAD2DEG(x) ((x)*180/M_PI)  // ラジアンから度

// ID,TYPE
#define     DEFAULT_ROBOT_ID    "turtlebot_01"
#define     DEFAULT_ROBOT_TYPE  "turtlebot"

#define     DEFAULT_VELOCITY_TOPIC  "cmd_vel"

class RobotDriver
{
private:
  //! The node handle we'll be using        // 使用するノードハンドル
  ros::NodeHandle nh_;
  //! We will be publishing to the "cmd_vel" topic to issue commands    //コマンドを発行するために "cmd_vel"トピックに公開します。
  ros::Publisher cmd_vel_pub_;
  //! We will be listening ot TF transforms as well //  TF変換も聴きます
  tf::TransformListener listener_;

  bool move_forward_state_;
  bool move_turn_state_;

  std::string entityId_; // ロボットのユニークID
  std::string velocityTopic_; // velocityトピック名


public:
  //! ROS node initialization       //  ROSノードの初期化
  RobotDriver(ros::NodeHandle &nh)
  {
    nh_ = nh;

    move_forward_state_ = false;
    move_turn_state_ = false;

  }

  RobotDriver(ros::NodeHandle &nh, ros::NodeHandle &privateNode)
  {
    nh_ = nh;

    move_forward_state_ = false;
    move_turn_state_ = false;

    if (privateNode.getParam("entity_id", entityId_)){
        ROS_INFO("RobotDriver entity_id:%s", entityId_.c_str());
    }else{
        entityId_ = DEFAULT_ROBOT_ID;
    }

    if (privateNode.getParam("velocity_topic", velocityTopic_)){
        ROS_INFO("RobotDriver velocity_topic:%s", velocityTopic_.c_str());
    }else{
        velocityTopic_ = DEFAULT_VELOCITY_TOPIC;
    }

    //set up the publisher for the cmd_vel topic    //cmd_velトピックの発行者を設定します。
    cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>( entityId_ + "/" + velocityTopic_, 1);
  }

  //! Drive forward a specified distance based on odometry information  // 走行距離情報に基づいて指定された距離を前進する
  // 並進   ※バックは不可 
  // distance： >0  走行距離
  bool driveForwardOdom(double distance)
  {
    //wait for the listener to get the first message    // リスナーが最初のメッセージを受け取るのを待つ

    try{
        ROS_DEBUG("--- waitForTransform --- Before  ");
        ros::Time now = ros::Time::now();
        listener_.waitForTransform(entityId_ + "_base_link", entityId_ + "_odom",     // base_linkから見たodomがくるまで待つ
                                now, ros::Duration(3.0));   // タイムアウト3秒
        ROS_DEBUG("--- waitForTransform --- After");
    }
    catch (tf::TransformException ex)
    {
      ROS_INFO("---  catch msg  --- waitForTransform");

      ROS_ERROR("%s",ex.what());
      return(false);
    }

    //we will record transforms here    // ここで変換を記録します
    tf::StampedTransform start_transform;       // 開始位置
    tf::StampedTransform current_transform;     // 現在位置
    //record the starting transform from the odometry to the base frame     // オドメトリからベースフレームへの開始変換を記録する
    ROS_DEBUG("--- lookupTransform --- Before");

    try{
        listener_.lookupTransform( entityId_ + "_base_link", entityId_ + "_odom",              // 座標の読み出し
                                  ros::Time(0), start_transform);
    }
    catch (tf::TransformException ex)
    {
      ROS_INFO("---  catch msg --- lookupTransform");

      ROS_ERROR("%s",ex.what());
      return(false);
    }

    ROS_DEBUG("--- lookupTransform --- After");

    //we will be sending commands of type "twist"   //  "twist"タイプのコマンドを送信します
    geometry_msgs::Twist base_cmd;
    //the command will be to go forward at 0.25 m/s     // コマンドは0.25m/sで前進することです（直進速度）
    base_cmd.linear.y = base_cmd.angular.z = 0;
    base_cmd.linear.x = 0.25;

    ros::Rate rate(50.0);   //  50Hz（1秒間に50回、この場合0.02秒間隔）
    bool done = false;

      ROS_DEBUG("start");
      ROS_DEBUG("x(%f) y(%f) z(%f) w(%f)",start_transform.getOrigin().x(),start_transform.getOrigin().y(),
      start_transform.getOrigin().z(), start_transform.getOrigin().w());
      ROS_DEBUG("Advance"); // 前進

    while (!done && nh_.ok())   //doneまで、またはCtrl+Cが押されるまで
    {
      //send the drive command      //  ドライブコマンドを送信する
      cmd_vel_pub_.publish(base_cmd);   // パブリッシュ
      rate.sleep();                     // 指定Hz間隔にするようにスリープ
      //get the current transform   //  現在の変換を取得する
      try
      {
        listener_.lookupTransform(entityId_ + "_base_link", entityId_ + "_odom",      // 座標の読み出し
                                  ros::Time(0), current_transform);     // ros::Time(0) = 最新の情報を読み出す
      }
      catch (tf::TransformException ex)
      {
        ROS_ERROR("%s",ex.what());
        break;
      }
      //see how far we've traveled  // 走行距離を計算
      tf::Transform relative_transform = 
        start_transform.inverse() * current_transform;              // inverse()=この変換の逆を返します。

      double dist_moved = relative_transform.getOrigin().length();  // getOrigin()=Vector3型を返す  length()=ベクトルの長さを返します。
//      ROS_DEBUG("x(%f) y(%f) z(%f) w(%f)",current_transform.getOrigin().x(),current_transform.getOrigin().y(),
//      current_transform.getOrigin().z(), current_transform.getOrigin().w());
      ROS_DEBUG("distance = %f dist_moved= %f",distance, dist_moved );

      if(dist_moved > distance) done = true;
    }

    base_cmd.linear.x = base_cmd.linear.y = base_cmd.angular.z = 0;
    cmd_vel_pub_.publish(base_cmd);

    if (done) return true;
    return false;
  }


  //  bool clockwise：true=右回転   false=左回転
  //  double radians：回転角度(ラジアン)    DEG2RAD(x) x=角度
  bool turnOdom(bool clockwise, double radians, double angularSpeed = 0.4)
  {
    double radians_before = radians;

    //wait for the listener to get the first message    // リスナーが最初のメッセージを受け取るのを待つ
    ros::Time now = ros::Time::now();
    try{
        listener_.waitForTransform( entityId_ + "_base_link", entityId_ + "_odom",    // base_linkから見たodomがくるまで待つ
                               now, ros::Duration(3.0));   // タイムアウト1秒
    }
    catch (tf::TransformException ex)
    {
      ROS_INFO("---  catch msg  --- waitForTransform");

      ROS_ERROR("%s",ex.what());
      return(false);
    }

    //we will record transforms here        // ここで変換を記録します
    tf::StampedTransform start_transform;
    tf::StampedTransform current_transform;



    double yaw_before = 0;
    double yaw_current = 0;
    double yaw_total = 0;

    //record the starting transform from the odometry to the base frame // オドメトリからベースフレームへの開始変換を記録する

    try{
        listener_.lookupTransform(entityId_ + "_base_link", entityId_ + "_odom",  // 座標の読み出し
                              ros::Time(0), start_transform);
    }
    catch (tf::TransformException ex)
    {
      ROS_INFO("---  catch msg --- lookupTransform");

      ROS_ERROR("%s",ex.what());
      return(false);
    }

    //we will be sending commands of type "twist"   // "twist"タイプのコマンドを送信します
    geometry_msgs::Twist base_cmd;
    //the command will be to turn at 0.75 rad/s     // コマンドは0.4 rad / sで回転するようになります（回転速度）
    base_cmd.linear.x = base_cmd.linear.y = 0.0;
    base_cmd.angular.z = angularSpeed;
    if (clockwise) base_cmd.angular.z = -base_cmd.angular.z; // 回転方向を決める（＋は左回転、－は右回転）

    double turn_threshold = fabs(base_cmd.angular.z) / 5;// 回転量のしきい値 0.2s時の最大回転rad

    //the axis we want to be rotating by            //  回転させたい軸
    tf::Vector3 desired_turn_axis(0,0,1);           //  z軸を指定
    if (!clockwise) desired_turn_axis = -desired_turn_axis;

    ros::Rate rate(50.0);   //  50Hz（1秒間に50回、この場合0.02秒間隔）
    bool done = false;

    double angle_turned_before = 0;
    double angle_turned_total = 0;

    while (!done && nh_.ok())   //doneまで、またはCtrl+Cが押されるまで
    {
      //send the drive command      // ドライブコマンドを送信する
      cmd_vel_pub_.publish(base_cmd);   // パブリッシュ
      rate.sleep();                     // 指定Hz間隔にするようにスリープ
      //get the current transform   //  現在の変換を取得する
      try
      {
        listener_.lookupTransform(entityId_ + "_base_link", entityId_ + "_odom",  // 座標の読み出し
                                  ros::Time(0), current_transform);

        tf::Matrix3x3 m = current_transform.getBasis();
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);

        yaw_current = yaw; // 現在のyawを取得   

      }
      catch (tf::TransformException ex)
      {
        ROS_ERROR("%s",ex.what());
        break;
      }

      if ( fabs(yaw_current-yaw_before) < 1.0e-2) continue;       // fabs=絶対値

      double turn_val;

      if( clockwise ){
          // 右回転     
          if(yaw_current >= yaw_before){
             turn_val = yaw_current - yaw_before;
             ROS_DEBUG(">= yaw_before(%f) yaw_current(%f) yaw_total(%f)+turn_val(%f)",yaw_before, yaw_current, yaw_total, turn_val);
          }else{  
             turn_val = (M_PI - yaw_before) + (M_PI + yaw_current);
             ROS_DEBUG("< yaw_before(%f) yaw_current(%f) yaw_total(%f)+turn_val(%f)",yaw_before, yaw_current, yaw_total, turn_val);
          }
      }else{
          // 左回転           
          if(yaw_before >= yaw_current){
             turn_val = yaw_before - yaw_current;       
             ROS_DEBUG(">= yaw_before(%f) yaw_current(%f) yaw_total(%f)+turn_val(%f)",yaw_before, yaw_current, yaw_total, turn_val);
          }else{  
             turn_val = (M_PI - yaw_current) + (M_PI + yaw_before);
             ROS_DEBUG("< yaw_before(%f) yaw_current(%f) yaw_total(%f)+turn_val(%f)",yaw_before, yaw_current, yaw_total, turn_val);
          }
      }
      if(turn_val <= turn_threshold){
          yaw_total += turn_val; // 加算
      }else{
          ROS_WARN("turn_threshold(%f) turn_val(%f)",turn_threshold, turn_val);
      }

      yaw_before = yaw_current; // 前回値保持

      if (yaw_total > radians) done = true;

    }

    base_cmd.linear.x = base_cmd.linear.y = base_cmd.angular.z = 0;
    cmd_vel_pub_.publish(base_cmd);

    if (done) return true;
    return false;
  }

  bool moveForwardState()
  { 
      return(move_forward_state_);
  }
  bool moveTurnState()
  { 
      return(move_turn_state_);
  }

  bool stopOdom()
  { 
      geometry_msgs::Twist base_cmd;
      base_cmd.linear.y = base_cmd.angular.z = 0;
      base_cmd.linear.x = 0.0;
      cmd_vel_pub_.publish(base_cmd);
      move_forward_state_ = false;
      move_turn_state_ = false;
  }
  bool moveForward(double linearSpeed)
  { 
      geometry_msgs::Twist base_cmd;
      base_cmd.linear.y = base_cmd.angular.z = 0;
      base_cmd.linear.x = linearSpeed;
      cmd_vel_pub_.publish(base_cmd);

      move_forward_state_ = true;
  }
  bool moveTurn(bool clockwise, double angularSpeed)
  { 
    geometry_msgs::Twist base_cmd;
    base_cmd.linear.x = base_cmd.linear.y = 0.0;
    base_cmd.angular.z = angularSpeed;
    if (clockwise) base_cmd.angular.z = -base_cmd.angular.z; // 回転方向を決める（＋は左回転、－は右回転）
    cmd_vel_pub_.publish(base_cmd);
    move_turn_state_ = true;
  }
};





