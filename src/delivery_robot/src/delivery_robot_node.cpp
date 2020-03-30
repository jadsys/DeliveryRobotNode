/*
    ロボットノード

トピック名称 メモ 
    ENTITY_IDは、ロボットごとに定めるユニークな文字列
> 　①ロボットへの移動指示
> 　　/robot_bridge/$(arg ENTITY_ID)/cmd
> 　②移動指示受信結果
> 　　/robot_bridge/$(arg ENTITY_ID)/cmdexe
> 　③ロボットの状態報告
> 　　/robot_bridge/$(arg ENTITY_ID)/state
> 　④ロボットへの緊急停止
> 　　/robot_bridge/$(arg ENTITY_ID)/emg
> 　⑤緊急停止受信結果
> 　　/robot_bridge/$(arg ENTITY_ID)/emgexe

*/
#include <ros/ros.h>
#include <actionlib_msgs/GoalID.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>// 初期位置トピックの型 https://demura.net/lecture/14011.html
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/BatteryState.h>//バッテリーステータス     TB3
#include <std_msgs/Int16MultiArray.h>//バッテリーステータス     メガローバ
#include <RobotDriver.cpp> // ロボット制御
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib_msgs/GoalStatusArray.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>

#include "delivery_robot/r_command.h" // 移動指示メッセージ
#include "delivery_robot/r_result.h"  // 移動指示応答メッセージ
#include "delivery_robot/r_state.h"   // 状態報告メッセージ
#include "delivery_robot/r_emergency_command.h"  // 緊急停止メッセージ
#include "delivery_robot/r_emergency_result.h"   // 緊急停止応答メッセージ

//  MODE種別
#define     MODE_STANDBY        "standby"
#define     MODE_NAVI           "navi"
#define     MODE_SUSPEND        "suspend"
#define     MODE_ERROR          "error"
//  COMMAND種別
#define     CMD_NAVI            "navi"
#define     CMD_REFRESH         "refresh"
#define     CMD_STANDBY         "standby"
//  RESULT種別
#define     RESULT_ACK          "ack"
#define     RESULT_IGNORE       "ignore"
#define     RESULT_ERROR        "error"
// ID,TYPE
#define     DEFAULT_ROBOT_ID    "turtlebot_01"
#define     DEFAULT_ROBOT_TYPE  "turtlebot"
// エマージェンシーコマンド
#define     EMERGENCY_STOP      "stop"
#define     EMERGENCY_SUSPEND   "suspend"
#define     EMERGENCY_RESUME    "resume"
// move base ステータスインデックス
#define     MOVEBASE_INIT       0
#define     MOVE_BASE_PENDING   0 
#define     MOVE_BASE_SUCCEEDED 3 
#define     MOVE_BASE_ABORTED   4 

// メガローバ 電圧係数
#define     MR_VOLTAGE_FACTOR   137.8

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;


    void waypointCopy(const delivery_robot::r_pose_optional waypointOrg, delivery_robot::r_pose_optional &waypoint)
    {
        waypoint.point.x = waypointOrg.point.x;
        waypoint.point.y = waypointOrg.point.y;
        waypoint.point.z = waypointOrg.point.z;

        waypoint.angle_optional.valid       = waypointOrg.angle_optional.valid; 
        waypoint.angle_optional.angle.roll  = waypointOrg.angle_optional.angle.roll; 
        waypoint.angle_optional.angle.pitch = waypointOrg.angle_optional.angle.pitch;
        waypoint.angle_optional.angle.yaw   = waypointOrg.angle_optional.angle.yaw;
        return;
    }

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
        //  ROS_INFO("%s", dest);

        std::string time_str = dest;
        //  ROS_INFO("%s", time_str.c_str());

        return( time_str );
    }

    void geometry_quat_to_rpy(double& roll, double& pitch, double& yaw, geometry_msgs::Quaternion geometry_quat)
    {
        tf::Quaternion quat;
        quaternionMsgToTF(geometry_quat, quat);
        tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);  //rpy are Pass by Reference
    }

    geometry_msgs::Quaternion GetQuaternionMsg(double roll, double pitch, double yaw )
    {
        geometry_msgs::Quaternion q;
        tf::Quaternion quat =tf::createQuaternionFromRPY(roll,pitch,yaw);
        quaternionTFToMsg( quat, q);
        return(q); 
    }

class RobotNode
{
private:
    // パブ
    ros::Publisher pub_cancel;
    ros::Publisher pub_goal;
    ros::Publisher pub_initial;
    ros::Publisher pub_answer;
    ros::Publisher pub_emergency_ans;
    tf::TransformListener tfl;
    // サブ
    ros::Subscriber sub_move_base_status;
    ros::Subscriber sub_command_recv;
    ros::Subscriber sub_battery_state_recv;
    ros::Subscriber sub_amclpose_recv;
    ros::Subscriber sub_emergency_recv;

    geometry_msgs::PoseWithCovarianceStamped initial_pose;  // 初期位置
    delivery_robot::r_pose_optional _current_destination;   // 現在の目的地(角度情報あり)

    std::list<delivery_robot::r_pose_optional> _destinations;// 目的値リスト
    ros::Timer timer;
    ros::WallTimer goal_timer;

    bool navi_flg;  // 自動走行中かを判定
    bool calibration_flg;  // キャリブレーション中かを判定
    bool _turn_busy_flg;  // 旋回中かを判定

    int move_base_sts;  // movebaseがゴールに着いたかを受信する

    RobotDriver *_driver;

    ros::Publisher pub_robot_sts;
    float volt_sts;
    std::string mode_status; // 現在のmode保持
    std::string _entityId; // ロボットのユニークID
    std::string _entity_type; // ロボットの種別の識別子

    double g_covariance[36];    // 共分散値

    bool _navi_node;//自作ナビノードを使用するかどうか
    double _wp_sleep_time;// wayポイント停止時間
    double _goal_tolerance_range;// goalポイント許容範囲
    bool _goal_allowable_flg;// goalポイント許容範囲判定フラグ
    double _goal_allowable_range;// ゴール地点到達時のタイムアウトタイマー開始半径
    double _goal_allowable_time;// ゴール地点到達時のタイムアウトタイム
    double _goal_allowable_angle;   // ゴール地点到達時のangle許容角度

public:
    //------------------------------------------------------------------------------
    //  コンストラクタ
    //------------------------------------------------------------------------------
    RobotNode(RobotDriver& driver)
    {
        _driver = &driver;

        volt_sts = -FLT_MAX;

        mode_status = MODE_STANDBY;
        navi_flg = false;
        calibration_flg = true;
        move_base_sts = MOVE_BASE_PENDING;
        _goal_allowable_flg = false;
        _turn_busy_flg = false;

        // 共分散値
        memset( &g_covariance, 0, sizeof(g_covariance)); 
        g_covariance[0] = 0.25;
        g_covariance[7] = 0.25;
        g_covariance[35] = 0.06853891945200942;
    }
    ~RobotNode()
    {

    }

    //------------------------------------------------------------------------------
    //  初期設定
    //------------------------------------------------------------------------------
    bool setup(ros::NodeHandle &node, ros::NodeHandle &privateNode)
    {
        if (privateNode.getParam("entity_id", _entityId)){
            ROS_INFO("entity_id (%s)", _entityId.c_str());
        }else{
            _entityId = DEFAULT_ROBOT_ID;
        }

        if (privateNode.getParam("entity_type", _entity_type)){
            ROS_INFO("entity_type (%s)", _entity_type.c_str());
        }else{
            _entity_type = DEFAULT_ROBOT_TYPE;
        }

        if (privateNode.getParam("navi_node", _navi_node)){
            ;
        }else{
            _navi_node = false;
        }

        if( _navi_node==true ){
            ROS_INFO("_navi_node==true");
        }else{
            ROS_INFO("_navi_node==false");
        }          

        if (privateNode.getParam("wp_sleep_time", _wp_sleep_time)){
            ROS_INFO("wp_sleep_time (%lf)", _wp_sleep_time);
        }else{
            _wp_sleep_time = 5;//(sec)
        }

        if (privateNode.getParam("goal_tolerance_range", _goal_tolerance_range)){
            ROS_INFO("goal_tolerance_range (%lf)", _goal_tolerance_range);
        }else{
            _goal_tolerance_range = 0.05;//(m)
        }

        if (privateNode.getParam("goal_allowable_range", _goal_allowable_range)){
            ROS_INFO("goal_allowable_range (%lf)", _goal_allowable_range);
        }else{
            _goal_allowable_range = 0.20;//(m)
        }

        if (privateNode.getParam("goal_allowable_time", _goal_allowable_time)){
            ROS_INFO("goal_allowable_time (%lf)", _goal_allowable_time);
        }else{
            _goal_allowable_time = 4;//(sec)
        }

        if (privateNode.getParam("goal_allowable_angle", _goal_allowable_angle)){
            ROS_INFO("goal_allowable_angle (%lf)", _goal_allowable_angle);
        }else{
            _goal_allowable_angle = 0.0349066;  // 2度
        }

        // 初期位置
        pub_initial = node.advertise<geometry_msgs::PoseWithCovarianceStamped>(
                    "/" + _entityId + "/initialpose", 5, true);       // true：ラッチ[オプション] 最後に発行されたメッセージが保存される

        // --- パブ ---
        // cancel                                     
        pub_cancel = node.advertise<actionlib_msgs::GoalID>("/" + _entityId + "/move_base/cancel", 5, true);
        // goal
        pub_goal = node.advertise<geometry_msgs::PoseStamped>("/" + _entityId + "/move_base_simple/goal", 5, false);
        // ロボットステータス
        pub_robot_sts = node.advertise<delivery_robot::r_state>("/state", 100, true);
        timer = node.createTimer(ros::Duration(1.0), &RobotNode::robotStatusSend, this);// 1Hz
        // answer
        pub_answer = node.advertise<delivery_robot::r_result>("/cmdexe", 100, false);
        //  緊急停止応答
        pub_emergency_ans = node.advertise<delivery_robot::r_emergency_result>("/emgexe", 100, false);

        // --- サブ ---
        // move_baseステータス
        sub_move_base_status = node.subscribe("/" + _entityId + "/move_base/status", 10, &RobotNode::movebaseStatusRecv, this);
        // 移動指示受信
        sub_command_recv = node.subscribe("/cmd", 10, &RobotNode::commandRecv, this);
        // バッテリーステータス受信
        if( _entity_type == DEFAULT_ROBOT_TYPE ){
            sub_battery_state_recv = node.subscribe("/" + _entityId + "/battery_state", 100, &RobotNode::batteryStateRecv, this);
        }else{
            sub_battery_state_recv = node.subscribe("/" + _entityId + "/rover_sensor", 100, &RobotNode::batteryStateRecvMr, this);
        }
        // amcl_pose受信
        sub_amclpose_recv = node.subscribe("/" + _entityId + "/amcl_pose", 10, &RobotNode::amclPoseRecv, this);
        // 緊急停止受信
        sub_emergency_recv = node.subscribe("/emg", 10, &RobotNode::emergencyRecv, this);

        // ゴール地点到達時のタイムアウトタイマー
        goal_timer = node.createWallTimer(ros::WallDuration(_goal_allowable_time), &RobotNode::goal_allowable_time, this, true, false);

        return(true);
    }

    //--------------------------------------------------------------------------
    //  緊急停止    受信
    //--------------------------------------------------------------------------
    void emergencyRecv(const delivery_robot::r_emergency_command& msg)
    {
        ROS_INFO("emergencyRecv id(%s) type(%s) time(%s) cmd(%s)",msg.id.c_str(), msg.type.c_str(), msg.time.c_str(), msg.emergency_cmd.c_str() );

        // コマンド取得 
        std::string emg_cmd = msg.emergency_cmd; // 受信したCMD

        std::vector<std::string> err_list;

        if(emg_cmd == EMERGENCY_STOP){
            if(calibration_flg == true){
                // キャリブレーション中
                err_list.push_back("during calibration");
            }else{
                movebaseCancel();   // 走行中断
                _driver->stopOdom();// いったん停止
                removeAllGoals();   // goal全削除
                mode_status = MODE_STANDBY;  // mode standbyセット
                navi_flg = false;
            }
        }else
        if( emg_cmd == EMERGENCY_SUSPEND){
            if(mode_status == MODE_NAVI){  // navi中
                movebaseCancel();   // 走行中断
                _driver->stopOdom();// いったん停止
                _destinations.push_front(_current_destination); // 現在の目的値を保持
                goal_timer.stop();
                mode_status = MODE_SUSPEND;
            }else{
                // ナビ走行中ではない
                err_list.push_back("Not navigating");
            }
        }else
        if(emg_cmd == EMERGENCY_RESUME){
            if(mode_status == MODE_SUSPEND){  // サスペンド中
                mode_status = MODE_NAVI;
                goalSend();
            }else{
                // サスペンド中ではない
                err_list.push_back("Not suspended");
            }
        }

        emergencyAnswer( msg, err_list);
    }

    //--------------------------------------------------------------------------
    //  緊急停止    応答
    //--------------------------------------------------------------------------
    void emergencyAnswer(const delivery_robot::r_emergency_command& cmd_msg, std::vector<std::string>& err_list)
    {
        delivery_robot::r_emergency_result ans_msg;

        ans_msg.id = _entityId;
        ans_msg.type = _entity_type;
        ans_msg.time =  iso8601ex();
        ans_msg.received_time = cmd_msg.time ;
        ans_msg.received_emergency_cmd = cmd_msg.emergency_cmd ;

        int err_cnt = err_list.size();
        if( err_cnt == 0 ){
            // 正常応答
            ans_msg.result = RESULT_ACK;
        }else{
            // 異常応答
            ans_msg.result = RESULT_ERROR;
            ans_msg.errors.resize(err_cnt);
            for( int i=0; i<err_cnt; i++){
                ans_msg.errors[i] = err_list[i];
            }
        }
        ROS_INFO("emergencyAns id(%s) type(%s) time(%s) rtime(%s) rcmd(%s) result(%s)", ans_msg.id.c_str(), ans_msg.type.c_str(), ans_msg.time.c_str(), ans_msg.received_time.c_str(), ans_msg.received_emergency_cmd.c_str(), ans_msg.result.c_str());

        pub_emergency_ans.publish(ans_msg);
    }

    //------------------------------------------------------------------------------
    //  移動指示受信
    //------------------------------------------------------------------------------
    void commandRecv(const delivery_robot::r_command msg)
    {
        ROS_INFO("commandRecv id(%s) type(%s) time(%s) cmd(%s)",msg.id.c_str(), msg.type.c_str(), msg.time.c_str(), msg.cmd.c_str() );
        std::vector<std::string> err_list;

        // コマンド取得 
        std::string cmd_status = msg.cmd; // 受信したCMD

        if( mode_status == MODE_STANDBY && cmd_status == CMD_NAVI){ // 待機中のnavi受信

            removeAllGoals();  // goal全削除
            // 目的地
            for(int i=0; i<msg.waypoints.size(); i++){
                _destinations.push_back( msg.waypoints[i]);
            }

            if(calibration_flg == true){
                // キャリブレーション中
                err_list.push_back("during calibration");
                commandAnswer( msg, RESULT_ERROR, err_list);
            }else
            // ナビ（自動走行）
            if( msg.waypoints.size() >= 1 ){
                // navi開始
                mode_status = MODE_NAVI;  // mode naviセット
                navi_flg = true;
                commandAnswer( msg, RESULT_ACK, err_list);
            }else{
                // wayポイントなし
                err_list.push_back("There is no way point");// wayポイントがありません
                commandAnswer( msg, RESULT_ERROR, err_list);
            }
        }else
        if( (mode_status == MODE_NAVI || mode_status == MODE_SUSPEND) && cmd_status == CMD_REFRESH){ // 移動中のRefresh受信
            movebaseCancel();   // 走行中断
            removeAllGoals();  // goal全削除

            // 目的地
            for(int i=0; i<msg.waypoints.size(); i++){
                _destinations.push_back( msg.waypoints[i]);
            }

            if( msg.waypoints.size() >= 1 ){
                mode_status = MODE_NAVI; // サスペンド中、NAVI状態に復帰させる
                // navi継続
                commandAnswer( msg, RESULT_ACK, err_list);
                goalSend();
            }else{
                // naviなし
                mode_status = MODE_STANDBY;  // mode standbyセット
                navi_flg = false;
                err_list.push_back("There is no way point");// wayポイントがありません
                commandAnswer( msg, RESULT_ERROR, err_list);
            }
        }else
        if( (mode_status == MODE_NAVI || mode_status == MODE_SUSPEND) && cmd_status == CMD_STANDBY){ // 移動中のstandby受信
            movebaseCancel();   // 走行中断
            removeAllGoals();  // goal全削除
            mode_status = MODE_STANDBY;  // mode standbyセット
            navi_flg = false;
            commandAnswer( msg, RESULT_ACK, err_list);
        }else{
            // コマンド無視
            ROS_INFO("command recv ignore MODE:%s CMD:%s", mode_status.c_str(), cmd_status.c_str());
            commandAnswer( msg, RESULT_IGNORE, err_list);
        }
    }

    //------------------------------------------------------------------------------
    //  移動指示    結果応答
    //------------------------------------------------------------------------------
    void commandAnswer(const delivery_robot::r_command& cmd_msg, std::string result_kind, std::vector<std::string>& err_list)
    {
        delivery_robot::r_result ans_msg;

        ans_msg.id = _entityId;
        ans_msg.type = _entity_type;
        ans_msg.time =  iso8601ex();
        ans_msg.received_time = cmd_msg.time;
        ans_msg.received_cmd = cmd_msg.cmd;
        ans_msg.received_waypoints.resize(cmd_msg.waypoints.size());
        for(int i=0;i<ans_msg.received_waypoints.size();i++){
            waypointCopy( cmd_msg.waypoints[i], ans_msg.received_waypoints[i] );
        }

        ans_msg.result = result_kind;

        // エラーメッセージ
        int err_cnt = err_list.size();
        ans_msg.errors.resize(err_cnt);
        for( int i=0; i<err_cnt; i++){
            ans_msg.errors[i] = err_list[i];
        }

        // パブ 
        pub_answer.publish(ans_msg);
    }

    //------------------------------------------------------------------------------
    //  バッテリステータス受信  30Hz
    //------------------------------------------------------------------------------
    void batteryStateRecv(const sensor_msgs::BatteryState msg)
    {
        volt_sts = msg.voltage;
    }

    //------------------------------------------------------------------------------
    //  バッテリステータス受信  メガローバ  90Hz
    //------------------------------------------------------------------------------
    void batteryStateRecvMr(const std_msgs::Int16MultiArray msg)
    {
        if(msg.data.size() >= 10){
            volt_sts = msg.data[9] / MR_VOLTAGE_FACTOR;
        }
    }

    //------------------------------------------------------------------------------
    //  amcl_pose受信
    //------------------------------------------------------------------------------
    void amclPoseRecv(const geometry_msgs::PoseWithCovarianceStamped msg)
    {
        try
        {
            memcpy( &g_covariance, &msg.pose.covariance, sizeof(g_covariance));//共分散
        }
        catch(const std::exception &e)
        {
            ROS_WARN("amcl_pose conversion errer[%s]", e.what());
        }
    }

    void robotStatusErrCheck(std::vector<std::string>& sts_err_list)
    {
        if( MOVE_BASE_ABORTED == move_base_sts ){
            sts_err_list.push_back("I can not reach the goal");// 到達できません
        } 
    }
    //--------------------------------------------------------------------------
    //  ロボットステータス送信
    //--------------------------------------------------------------------------
    void robotStatusSend(const ros::TimerEvent&)
    {
        double x, y, z, roll, pitch, yaw;
        std::vector<std::string> sts_err_list;
        try
        {
            // エラーチェック
            robotStatusErrCheck(sts_err_list);
            int err_cnt = sts_err_list.size();
            // 座標取得
            tf::StampedTransform trans;
            ros::Time now = ros::Time::now();
            tfl.waitForTransform("map", _entityId + "_base_link",    
                    now, ros::Duration(3.0));
            tfl.lookupTransform("map", _entityId + "_base_link",   // mapからbase_link、(world座標のロボットの位置)
                    ros::Time(0), trans);
            x = trans.getOrigin().x();                // X座標
            y = trans.getOrigin().y();                // Y座標
            z = 0.0;                                  // Z座標(※ 0固定)
            tf::Matrix3x3 m = trans.getBasis();
            m.getRPY(roll, pitch, yaw);

            // --- ロボットステータス ---
            delivery_robot::r_state msg;

            msg.id = _entityId;
            msg.type = _entity_type;
            msg.time = iso8601ex();
            if(err_cnt <= 0){ 
                msg.mode = mode_status;
            }else{
                msg.mode = MODE_ERROR;// エラー応答
                msg.errors.resize(err_cnt);
                for( int i=0; i<err_cnt; i++){
                    msg.errors[i] = sts_err_list[i];// エラーメッセージ
                }
            } 
            // 現在の位置・向き
            msg.pose.point.x = x;
            msg.pose.point.y = y;
            msg.pose.point.z = z;
            msg.pose.angle.roll = roll;
            msg.pose.angle.pitch = pitch;
            msg.pose.angle.yaw = yaw;

            //  現在の目的値
            if( navi_flg == true ){
                memcpy( &msg.destination, &_current_destination, sizeof(msg.destination));
            }

            memcpy( &msg.covariance, &g_covariance, sizeof(msg.covariance));// 共分散

            // バッテリー情報
            msg.battery.voltage = volt_sts;
            msg.battery.current_optional.valid = false;
            msg.battery.current_optional.current = 0.0;

            // パブ
            pub_robot_sts.publish(msg);
        }
        catch(tf::TransformException &e)
        {
            ROS_WARN("robot status send err[%s]", e.what());
        }
    }
    //--------------------------------------------------------------------------
    //  初期位置送信
    //--------------------------------------------------------------------------
    bool initialPoseSend(const float x, const float y, const float yaw)
    {
        initial_pose.header.frame_id = "map";
        initial_pose.pose.pose.position.x = x;
        initial_pose.pose.pose.position.y = y;
        initial_pose.pose.pose.orientation = GetQuaternionMsg( 0, 0, yaw);

        memset(&initial_pose.pose.covariance[0], 0, sizeof(initial_pose.pose.covariance));//共分散行列（散らばり具合を表す指標）
        initial_pose.pose.covariance[0] = 0.25;
        initial_pose.pose.covariance[7] = 0.25;
        initial_pose.pose.covariance[35] = 0.06853891945200942;
        pub_initial.publish(initial_pose);
    }

    //--------------------------------------------------------------------------
    //  目的値全削除
    //--------------------------------------------------------------------------
    void removeAllGoals()
    {
        while ( _destinations.size() >= 1) _destinations.pop_front();
        _destinations.clear();
    }

    //--------------------------------------------------------------------------
    //  movebase cancel 送信
    //--------------------------------------------------------------------------
    void movebaseCancel()
    {
        if(_navi_node == false){
             // アクションクライアント  
            _driver->stopOdom();  // いったん停止
            //tell the action client that we want to spin a thread by default
            MoveBaseClient ac( _entityId + "/move_base", true);
            
            //wait for the action server to come up
            while(!ac.waitForServer(ros::Duration(5.0))){
                ROS_INFO("Waiting for the move_base action server to come up");     // move_baseアクションサーバが起動するのを待っています
            }

            ac.cancelAllGoals(); 
            return;

        }else{
            // 現在地点を目標地点とする
            float x, y, z, yaw;
            tf::StampedTransform trans;
            tfl.waitForTransform("map", _entityId + "_base_link",    
                     ros::Time(0), ros::Duration(3.0));
            tfl.lookupTransform("map", _entityId + "_base_link",   // mapからbase_link、(world座標のロボットの位置)
                     ros::Time(0), trans);
            x = trans.getOrigin().x();                // Ｘ座標
            y = trans.getOrigin().y();                // Ｙ座標
            z = trans.getOrigin().z();                // Z座標
            yaw = tf::getYaw(trans.getRotation());    // 四元数からyaw角を取得

            geometry_msgs::PoseStamped stop_goal;

            stop_goal.header.frame_id = "map";
            stop_goal.pose.position.x = x;
            stop_goal.pose.position.y = y;
            stop_goal.pose.orientation = GetQuaternionMsg( 0, 0, yaw);
            pub_goal.publish(stop_goal);
        }
    } 

    //--------------------------------------------------------------------------
    //  move base ステータス受信
    //--------------------------------------------------------------------------
    void movebaseStatusRecv(const actionlib_msgs::GoalStatusArray::ConstPtr &status)
    {
        int status_id = -1;
        //uint8 PENDING         = 0  // まだアクションサーバによって処理されていない
        //uint8 ACTIVE          = 1  // ゴールは現在アクションサーバによって処理されている
        //uint8 PREEMPTED       = 2  // 実行開始後、キャンセル要求を受け取りました
        //uint8 SUCCEEDED       = 3  // 目標は、アクション・サーバ（終端状態）で成功裏に達成した
        //uint8 ABORTED         = 4  // 目標は、アクション・サーバによるによって実行中に中止されました
        //uint8 REJECTED        = 5  // 目標が処理できずにアクションサーバーによって拒否されました。目標が到達不能または無効だったため
        //uint8 PREEMPTING      = 6  // 目標は実行開始後、キャンセル要求を受け取り、実行は完了していません
        //uint8 RECALLING       = 7  // ゴールは実行を開始する前にキャンセル要求を受け取りました
        //uint8 RECALLED        = 8  // ゴールは実行を開始する前にキャンセル要求を受け取って＃正常にキャンセルされました
        //uint8 LOST            = 9  // アクションクライアントは、ゴールがLOSTであると判断できます。

        if (!status->status_list.empty()){
            actionlib_msgs::GoalStatus goalStatus = status->status_list[0];
            status_id = goalStatus.status;

            move_base_sts = status_id;
        }

        if(status_id==1){
            //移動中
        }else
        if((status_id==3)||(status_id==0)){
            //ゴールに到達・もしくはゴールに到達して待機中。
            ROS_INFO("movebase GOAL(%d)", status_id);
        }else
        if( status_id==MOVE_BASE_ABORTED ){ // アボート
            ROS_INFO("movebase ABORTED!!");


            if( mode_status == MODE_NAVI ){ // ナビ走行中であれば走行停止
                movebaseCancel();   // 走行中断
                removeAllGoals();  // goal全削除
                mode_status = MODE_STANDBY;  // mode standbyセット
                navi_flg = false;
            }
        }
    }

    //--------------------------------------------------------------------------
    //  旋回角度と向きを取得する
    //--------------------------------------------------------------------------
    bool getTurnAngle(const double rad_base, const double rad , double &turn_rad)
    {
        turn_rad = 0;
        bool blsts = true;
        double current_rad = rad_base + M_PI;
        double goal_rad    = rad      + M_PI;

        if( current_rad >= goal_rad ){
            if( (current_rad - goal_rad) <= M_PI ){
                blsts = true;   // 右旋回
                turn_rad = current_rad - goal_rad;
            }else{
                blsts = false;  // 左旋回
                turn_rad = 2*M_PI - (current_rad - goal_rad);
            }
        }else{
            if( (goal_rad - current_rad) <= M_PI ){
                turn_rad = goal_rad - current_rad;
                blsts = false;  // 左旋回
            }else{
                turn_rad = 2*M_PI - (goal_rad - current_rad);
                blsts = true;   // 右旋回
            }
        }

        return( blsts );
    }

    //--------------------------------------------------------------------------
    //  wayポイント方向に向きを変える
    //--------------------------------------------------------------------------
    double turnTowardsGoal()
    {
        double x, y, yaw;
        try
        {
            // 現在のロボットの向きを取得
            tf::StampedTransform trans;
            tfl.waitForTransform("map", _entityId + "_base_link", ros::Time(0), ros::Duration(0.5));
            tfl.lookupTransform("map", _entityId + "_base_link", ros::Time(0), trans);// mapから見たbase_link、(world座標のロボットの位置)
            x = trans.getOrigin().x();                // Ｘ座標
            y = trans.getOrigin().y();                // Ｙ座標
            yaw = tf::getYaw(trans.getRotation());    // 四元数からyaw角を取得
        }
        catch(tf::TransformException &e)
        {
            ROS_WARN("turnTowardsGoal() err(%s)", e.what());
            return(0.0);
        }

        // goalに向かう向きを取得    
        double yaw_way = atan2((double)(_current_destination.point.y - y), (double)(_current_destination.point.x - x));

        // 旋回角度を取得
        double turn_rad = 0; //旋回角度
        bool turn_direction = getTurnAngle( yaw, yaw_way, turn_rad);

        // 旋回させる
        ros::Rate rate(30.0);   // 30Hz処理

        for(;;)
        {
            // 旋回させる
            _driver->moveTurn(turn_direction, 0.4);  
 
            rate.sleep();
            ros::spinOnce();

            try
            {
                // 現在のロボットの向きを取得
                tf::StampedTransform trans;
                tfl.waitForTransform("map", _entityId + "_base_link", ros::Time(0), ros::Duration(0.5));
                tfl.lookupTransform("map", _entityId + "_base_link", ros::Time(0), trans);// mapから見たbase_link、(world座標のロボットの位置)
                x = trans.getOrigin().x();                // Ｘ座標
                y = trans.getOrigin().y();                // Ｙ座標
                yaw = tf::getYaw(trans.getRotation());    // 四元数からyaw角を取得
            }
            catch(tf::TransformException &e)
            {
                ROS_WARN("turnTowardsGoal() err(%s)", e.what());
                continue;
            }

            // goalに向かう向きを取得    
            yaw_way = atan2((double)(_current_destination.point.y - y), (double)(_current_destination.point.x - x));

            // 旋回角度を取得
            turn_rad = 0; //旋回角度
            double turn_direction_now = getTurnAngle( yaw, yaw_way, turn_rad);

            if(turn_direction_now != turn_direction){
                ROS_INFO("turn_direction reverse");
                turn_direction = turn_direction_now;
            } 

            if(RAD2DEG(turn_rad) <= 3.0){
                break;
            }

            if(navi_flg == false || mode_status == MODE_SUSPEND){
                break;
            } 
        } 

        _driver->stopOdom();  // いったん停止

        return( yaw_way );
    }

    //--------------------------------------------------------------------------
    //  現在の座標,向きを取得する
    //--------------------------------------------------------------------------
    bool currentCoordinates( double& cur_x, double& cur_y, double& cur_yaw )
    {
        bool ret_sts = true;
        try
        {
            // 現在のロボットの向きを取得
            tf::StampedTransform trans;
            tfl.waitForTransform("map", _entityId + "_base_link", ros::Time(0), ros::Duration(0.5));
            tfl.lookupTransform("map", _entityId + "_base_link", ros::Time(0), trans);// mapから見たbase_link、(world座標のロボットの位置)
            cur_x = trans.getOrigin().x();                // Ｘ座標
            cur_y = trans.getOrigin().y();                // Ｙ座標
            cur_yaw = tf::getYaw(trans.getRotation());    // 四元数からyaw角を取得
        }
        catch(tf::TransformException &ex)
        {
            ROS_WARN("currentCoordinates() err(%s)", ex.what());
            ret_sts = true;
        }

        return( ret_sts );
    }

    //--------------------------------------------------------------------------
    //  狙った方向に向きを変える
    //--------------------------------------------------------------------------
    bool turnAngle( double angle_yaw, double allowable_angle )
    {
        double cur_x, cur_y, cur_yaw;
        double turn_rad = 0; //旋回角度
        int err_cnt=0;
        bool ret_sts = true;

        bool loop_end = false;
        ros::Rate rate(30.0);   // 30Hz処理

        while(!loop_end)
        {
            rate.sleep();
            ros::spinOnce();

            if( true == currentCoordinates( cur_x, cur_y, cur_yaw )){   // 現在の向きを取得
                bool turn_direction = getTurnAngle( cur_yaw, angle_yaw, turn_rad ); // 角度のずれを取得
                ROS_INFO( "turn_rad(%f) allowable_angle(%f)", turn_rad, allowable_angle);
                if( turn_rad > allowable_angle ){   // 許容範囲よりずれているか
                    // 旋回させる
                    _driver->moveTurn(turn_direction, 0.4);
                }else{
                    loop_end = true;
                }
            }else{
                err_cnt++;
                if( err_cnt >= 10 ){
                    ret_sts = false;
                    loop_end = true;
                }
            }
        }

        _driver->stopOdom();  // いったん停止

        return( ret_sts );
    }

    //------------------------------------------------------------------------------
    //  ゴール地点到達時のタイマータイムアウト
    //------------------------------------------------------------------------------
    void goal_allowable_time(const ros::WallTimerEvent&) 
    {
        ROS_INFO("!!!!goal_allowable_time Out!!!!");
        _goal_allowable_flg = true;
        movebaseCancel();//停止させる
    }

    //------------------------------------------------------------------------------
    //  Sleep処理
    //------------------------------------------------------------------------------
    void sleepFunc(double sleep_time)
    {
        double sleepTotal = 0;
        
        ros::Rate rate(20.0);   // 20Hz処理

        while(ros::ok())
        {
            rate.sleep();
            ros::spinOnce();
            sleepTotal+=0.05; 
            if(sleepTotal >= sleep_time){
                break;
            }
        }
    } 

    //--------------------------------------------------------------------------
    //  PoseStampedメッセージ作成
    //--------------------------------------------------------------------------
    geometry_msgs::PoseStamped makePoseStamped( double x, double y, double yaw)
    {
        geometry_msgs::PoseStamped  posePoseStamped;

        posePoseStamped.header.frame_id = "map";
        posePoseStamped.pose.position.x = x;
        posePoseStamped.pose.position.y = y;
        posePoseStamped.pose.orientation = GetQuaternionMsg( 0, 0, yaw);

        return( posePoseStamped );
    }

    //--------------------------------------------------------------------------
    //  wayポイント送信
    //--------------------------------------------------------------------------
    bool goalSend()
    {
        bool retSts = false;  

        goal_timer.stop(); 
        _goal_allowable_flg = false;

        if(_destinations.size() == 0) return false;

        _current_destination = _destinations.front();
        _destinations.pop_front();

        if(_turn_busy_flg == false){

            _turn_busy_flg = true; 
            // goalの方向を向ける
            double yaw = turnTowardsGoal();
            // wayポイント到着時の向き指定ありか
            if( _current_destination.angle_optional.valid == true ){
                yaw = _current_destination.angle_optional.angle.yaw;
            }

            geometry_msgs::PoseStamped way_goal = makePoseStamped( _current_destination.point.x, _current_destination.point.y, yaw);

            // パブリッシュ
            if(navi_flg == true && mode_status == MODE_NAVI){
                pub_goal.publish(way_goal);
                ROS_INFO("Applying goal x:%0.3f y:%0.3f yaw:%0.3f",
                    way_goal.pose.position.x,
                    way_goal.pose.position.y,
                    tf::getYaw(way_goal.pose.orientation));
                retSts = true; 
            }else{
                retSts = false;
            } 
            _turn_busy_flg = false; 
        } 

        return(retSts);
    }

    //--------------------------------------------------------------------------
    //  １回転
    //--------------------------------------------------------------------------
    void turn360(double turn_speed) 
    {
        double yaw;

        try
        {
            // 現在のロボットの向きを取得
            tf::StampedTransform trans;
            tfl.waitForTransform(_entityId + "_base_link", _entityId + "_odom", ros::Time(0), ros::Duration(0.5));
            tfl.lookupTransform(_entityId + "_base_link", _entityId + "_odom", ros::Time(0), trans);// base_linkから見たodom、(ロボットの向き)
            yaw = tf::getYaw(trans.getRotation());    // 四元数からyaw角を取得
        }
        catch(tf::TransformException &e)
        {
            ROS_ERROR("turn360() err(%s)", e.what());
            calibration_flg = false;
            return;
        }

        double turn_threshold = turn_speed / 5;// 回転量のしきい値 0.2s時の最大回転rad

        double yaw_before = 0;
        double yaw_current = yaw;
        double yaw_total = 0;
        double turn_val;

        ROS_INFO("turn360 (%f rad/s) start ---------->",turn_speed);

        ros::Rate rate(50.0);

        while(ros::ok())
        {
            _driver->moveTurn(true, turn_speed); // 旋回させる(右回転)

            rate.sleep();
            ros::spinOnce();
 
            try
            {
                // 現在のロボットの向きを取得
                tf::StampedTransform trans;
                ros::Time now = ros::Time::now();
                tfl.waitForTransform(_entityId + "_base_link", _entityId + "_odom", now, ros::Duration(1.0));
                tfl.lookupTransform(_entityId + "_base_link", _entityId + "_odom", now, trans);// base_linkodom見たbase_link、(ロボットの向き)
                yaw_current = tf::getYaw(trans.getRotation());    // 四元数からyaw角を取得
            }
            catch(tf::TransformException &e)
            {
                ROS_ERROR("turn360() err(%s)", e.what());
                return;
            }

            if ( fabs(yaw_current-yaw_before) < 1.0e-2) continue;// 小さすぎる値はスルー // fabs=絶対値

            // 右回転     
            if(yaw_current >= yaw_before){
               turn_val = yaw_current - yaw_before;
                ROS_DEBUG(">= yaw_before(%f) yaw_current(%f) yaw_total(%f)+turn_val(%f)",yaw_before, yaw_current, yaw_total, turn_val);
            }else{  
               turn_val = (M_PI - yaw_before) + (M_PI + yaw_current);
               ROS_DEBUG("< yaw_before(%f) yaw_current(%f) yaw_total(%f)+turn_val(%f)",yaw_before, yaw_current, yaw_total, turn_val);
            }

            if(turn_val <= turn_threshold){
                yaw_total += turn_val; // 加算
            }else{
                //ROS_WARN("turn_threshold(%f) turn_val(%f)",turn_threshold, turn_val);
            }

            yaw_before = yaw_current; // 前回値保持

            if (yaw_total >= DEG2RAD(360)){
                break;
            }
        }

        _driver->stopOdom();  // いったん停止
        calibration_flg = false;

        ROS_INFO("turn360 end <-------------");
    }

    //--------------------------------------------------------------------------
    //  メインループ
    //--------------------------------------------------------------------------
    void mainloop()
    {
        ROS_INFO("Standby...");

        ros::Rate rate(10.0);   // 10Hz処理

        while(ros::ok())
        {
            rate.sleep();
            ros::spinOnce();

            if( mode_status == MODE_NAVI && navi_flg == true ){
                if(!goalSend() && mode_status == MODE_NAVI) //  wayポイント送信
                {
                    ROS_ERROR("No goal specified");
                    navi_flg = false;
                    continue;   // 移動指示を待つ
                }
                ROS_INFO("Navi Start");
            }else{
                continue;   // 移動指示待ち
            }

            while(ros::ok())
            {
                rate.sleep();
                ros::spinOnce();

                float x, y, yaw;
                try
                {
                    tf::StampedTransform trans;
                    tfl.waitForTransform("map", _entityId + "_base_link",
                            ros::Time(0), ros::Duration(0.5));
                    tfl.lookupTransform("map", _entityId + "_base_link",   // mapから見たbase_link、(world座標のロボットの位置)
                            ros::Time(0), trans);
                    x = trans.getOrigin().x();                // Ｘ座標
                    y = trans.getOrigin().y();                // Ｙ座標
                    yaw = tf::getYaw(trans.getRotation());    // 四元数からyaw角を取得
                }
                catch(tf::TransformException &e)
                {
                    ROS_WARN("%s", e.what());
                    continue;
                }

                ROS_INFO("hypotf(%f)", hypotf(x - _current_destination.point.x, y - _current_destination.point.y) );

                if(mode_status == MODE_SUSPEND){
                    ROS_INFO("Suspend..."); // サスペンド中
                    continue;
                }

                if(hypotf(x - _current_destination.point.x, y - _current_destination.point.y) <= _goal_allowable_range){  //  目的値まで近づいたらタイマー開始する
                    if(navi_flg == true){
                        ROS_INFO("Goal Timer Start");
                        goal_timer.start(); //タイマースタート
                    }
                }

                if( ((move_base_sts==MOVE_BASE_SUCCEEDED || move_base_sts==MOVE_BASE_PENDING) //ゴールに到達・もしくはゴールに到達して待機中。
                     && hypotf(x - _current_destination.point.x, y - _current_destination.point.y) <= _goal_tolerance_range) //  目的値までの許容範囲以内
                || _goal_allowable_flg == true ) {  // ゴール到達時タイムアウト

                    ROS_INFO("Goal Timer Stop");
                    goal_timer.stop();
                    ROS_INFO("move_base_sts(%d)",move_base_sts);

                    // 角度判定
                    if( _current_destination.angle_optional.valid == true ){ // 角度指定あり
                        double goal_angle_yaw = _current_destination.angle_optional.angle.yaw;
                        turnAngle( goal_angle_yaw, _goal_allowable_angle );
                    }

                    ROS_INFO("way point Goal");

                    if( _current_destination.angle_optional.valid == true && _destinations.size() >= 1 ){
                        // ちょっと止まる
                        sleepFunc(_wp_sleep_time);
                    }

                    bool goal_snd_sts = goalSend();

                    if(mode_status == MODE_NAVI){
                        if(goal_snd_sts == false){
                            // ナビ中の次のwayポイントなし
                            ROS_INFO("Finished");        // wayポイントが無ければ終了
                            mode_status = MODE_STANDBY;  // mode standbyセット
                            navi_flg = false;
                        }else{
                            // wayポイント送信
                            ROS_INFO("Next goal applied");
                        }
                    }
                }

                if(navi_flg == false){
                    ROS_INFO("navi end. Standby... ");
                    break;  // 自動走行終了
                }
            }
        }
    }
};
/******************************************************************************/
/* main                                                                       */
/******************************************************************************/
int main(int argc, char *argv[])
{
    ros::init(argc, argv, "delivery_robot_node");

    ros::NodeHandle node;
    ros::NodeHandle privateNode("~");

    // 初期位置
    std::vector<double> init_pose;
    if (privateNode.getParam("initial_pose", init_pose)){
        ROS_INFO("initial_pose(%lf,%lf,%lf)", init_pose[0], init_pose[1], init_pose[2]);
    }else{
        ROS_ERROR("not initial pose");
        return(-1);
    }
    // # キャリブレーション 回転速度
    double turn_speed;
    if (privateNode.getParam("initial_turn_speed", turn_speed)){
        ROS_INFO("initial_turn_speed (%lf)", turn_speed);
    }else{
        turn_speed = 0.4;// rad
    }

    RobotDriver driver(node, privateNode);

    RobotNode robot_node(driver);
    robot_node.setup(node, privateNode); 

    // 初期位置送信 
    robot_node.initialPoseSend(init_pose[0], init_pose[1], init_pose[2]);//initialPose送信

    //1回転
    robot_node.turn360(turn_speed);

    ROS_INFO("stanby");

    // メインループ
    robot_node.mainloop();
}
