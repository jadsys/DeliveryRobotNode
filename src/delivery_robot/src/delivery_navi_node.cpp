/*
    ナビゲーションパッケージを利用して、複数の目的地に順番に移動する
*/
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <RobotDriver.cpp>
#include <sensor_msgs/LaserScan.h>

#ifndef M_PI
#define M_PI 3.14159265358979             // 円周率
#endif
#define DEG2RAD(x) ((x)*M_PI/180)  // 度からラジアン
#define RAD2DEG(x) ((x)*180/M_PI)  // ラジアンから度

    double GetMsgYaw(const geometry_msgs::Quaternion &q)
    {
        double roll, pitch, yaw;

        tf::Quaternion quat(q.x,q.y,q.z,q.w);
        tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
        return(yaw);
    }

class robot_navi_node
{
private:
    ros::Subscriber sub_scan; // /scan用 サブスクライバ
    ros::Subscriber sub_goal; // /goal用 サブスクライバ
    tf::TransformListener tfl;

    geometry_msgs::PoseStamped current_goal;                // 目的地
    std::list<geometry_msgs::PoseStamped> goals;

    sensor_msgs::LaserScan latest_scan; // /scanトピック受信値を保存        型確認コマンド：rostopic info /scan

    // /scanトピックを受信するコールバック関数
    void cb_scan(const sensor_msgs::LaserScan::ConstPtr &msg)
    {
        // 受け取ったメッセージをコピーしておく
        latest_scan = *msg;
    }

    // /goalトピックを受信するコールバック関数
    void cb_goal(const geometry_msgs::PoseStamped::ConstPtr &msg)
    {
        // 受け取ったメッセージをコピーしておく
        ROS_INFO("Get goal point!!");
        current_goal = *msg;
    }

    double _wall_judgment_width;        // 壁判定   幅
    double _wall_judgment_distance;     // 壁判定   正面

    double _linear_speed_long;  // 走行速度(遠い)
    double _linear_speed_short; // 走行速度(近い)
    double _linear_speed_near;  // 走行速度(ゴール付近)

    double _distance_short;  // 走行速度切り替え距離(近い)
    double _distance_near;   // 走行速度切り替え距離(ゴール付近))
    double _distance_goal;   // ゴール判定距離
    double _allowable_angle; // ゴール判定距離

public:
    robot_navi_node()
    {
        ros::NodeHandle nh("~");//プライベートパラメータを読みだすためのノードハンドル

        current_goal.pose.position.x = FLT_MAX;
        current_goal.pose.position.y = FLT_MAX;

        //  -----   パブリッシャ    ------

        //  -----   サブスクライバ  ------
        //      (scanを受信する)
        sub_scan = nh.subscribe("/scan", 5,                         // "/scan"：トピック名      5：キューサイズ
                                &robot_navi_node::cb_scan, this);   //  ～cb_scan：コールバック関数
        //      (goalを受信する)
        sub_goal = nh.subscribe("/move_base_simple/goal", 5,        // "/goal"：トピック名      5：キューサイズ
                                &robot_navi_node::cb_goal, this);   //  ～cb_goal：コールバック関数
    }

    //------------------------------------------------------------------------------
    //  初期設定
    //------------------------------------------------------------------------------
    bool setup(ros::NodeHandle &node, ros::NodeHandle &privateNode)
    {
        if (privateNode.getParam("wall_judgment_width", _wall_judgment_width)){
            ROS_INFO("wall_judgment_width (%f)", _wall_judgment_width);
        }else{
            _wall_judgment_width = 0.30;
        }

        if (privateNode.getParam("wall_judgment_distance", _wall_judgment_distance)){
            ROS_INFO("wall_judgment_distance (%f)", _wall_judgment_distance);
        }else{
            _wall_judgment_distance = 0.60;
        }

        if (privateNode.getParam("linear_speed_long", _linear_speed_long)){
            ROS_INFO("linear_speed_long (%f)", _linear_speed_long);
        }else{
            _linear_speed_long = 0.20;
        }

        if (privateNode.getParam("linear_speed_short", _linear_speed_short)){
            ROS_INFO("linear_speed_short (%f)", _linear_speed_short);
        }else{
            _linear_speed_short = 0.10;
        }

        if (privateNode.getParam("linear_speed_near", _linear_speed_near)){
            ROS_INFO("linear_speed_near (%f)", _linear_speed_near);
        }else{
            _linear_speed_near = 0.05;
        }

        if (privateNode.getParam("distance_short", _distance_short)){
            ROS_INFO("distance_short (%f)", _distance_short);
        }else{
            _distance_short = 0.50;
        }

        if (privateNode.getParam("distance_near", _distance_near)){
            ROS_INFO("distance_near (%f)", _distance_near);
        }else{
            _distance_near = 0.30;
        }

        if (privateNode.getParam("distance_goal", _distance_goal)){
            ROS_INFO("distance_goal (%f)", _distance_goal);
        }else{
            _distance_goal = 0.05;
        }

        if (privateNode.getParam("allowable_angle", _allowable_angle)){
            ROS_INFO("allowable_angle (%f)", _allowable_angle);
        }else{
            _allowable_angle = 0.05;
        }
    }

    //--------------------------------------------------------------------------
    // 壁判定
    //--------------------------------------------------------------------------
    bool wall_scan()
    {
        bool blret = false;

        if(latest_scan.ranges.size() > 0)
        {
            // LaserScanメッセージをすでに受け取っている場合

            float hx = 0, hy = 0;
            int hnum = 0;

            // theta-range 座標系から x-y 座標系に変換
            for(unsigned int i = 0; i < latest_scan.ranges.size(); i ++)
            {
                if(!(latest_scan.ranges[i] < latest_scan.range_min ||
                     latest_scan.ranges[i] > latest_scan.range_max ||
                     std::isnan(latest_scan.ranges[i])))                // 数値が NaN であるか判定する。
                {
                    // 距離値がエラーでない場合

                    float theta = latest_scan.angle_min + (latest_scan.angle_increment * i);       // angle_increment：測定間の角距離[rad]
                    // x-y 座標系に変換
                    float x = latest_scan.ranges[i] * cosf(theta);
                    float y = latest_scan.ranges[i] * sinf(theta);

                    if(fabs(y) <= (_wall_judgment_width/2) && x >= 0.05 && x <= _wall_judgment_distance){   // 幅 30cm 正面35cm以内
                        blret = true;
                        ROS_INFO("x=%f y=%f", x, y);
                    }
                }
            }
        }

        return( blret );
    }

    bool rad_func(const double rad_base, const double rad , double &turn_rad)
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
    //  メインループ
    //--------------------------------------------------------------------------
    void mainloop(RobotDriver& driver)
    {
        ROS_INFO("navi node start");

        ros::Rate rate(10.0);   // 10Hz処理
        while(ros::ok())
        {
            rate.sleep();
            ros::spinOnce();

            if(current_goal.pose.position.x == FLT_MAX && current_goal.pose.position.y == FLT_MAX){
                continue;   // ゴール未定
            }

            double x, y, yaw;
            try
            {
                tf::StampedTransform trans;
                tfl.waitForTransform("map", "base_link",    
                        ros::Time(0), ros::Duration(0.5));
                tfl.lookupTransform("map", "base_link",   // mapから見たbase_link、(world座標のロボットの位置)
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

            double yaw_goal = GetMsgYaw(current_goal.pose.orientation); //  ゴールの向きを取得(値未使用)

            //------------------------------------------------------------------
            //  ゴール判定
            //------------------------------------------------------------------
            if(hypotf(x - current_goal.pose.position.x,                         //  hypotf:目的値までの距離を取得
                        y - current_goal.pose.position.y) <= _distance_goal)    //  目的値までの距離が判定距離より近い
            {
                ROS_INFO("GOAL Stop !!");

                // ゴール
                current_goal.pose.position.x = FLT_MAX;
                current_goal.pose.position.y = FLT_MAX;
                driver.stopOdom();  // いったん停止

            }else
            {
                double goal_range = hypot(x - current_goal.pose.position.x, y - current_goal.pose.position.y);
                double turn_rad = 0; //旋回角度

                double yaw_way = atan2((double)(current_goal.pose.position.y - y), (double)(current_goal.pose.position.x - x)); // goalの方向
                double turn = rad_func( yaw, yaw_way, turn_rad);
                ROS_INFO("goal_range=%f yaw=%f yaw_goal=%f yaw_way=%f turn_rad=%f", goal_range,  yaw, yaw_goal, yaw_way, turn_rad);

                if( wall_scan() == true ){
                    //----------------------------------------------------------
                    //  障害物検知
                    //----------------------------------------------------------
                    driver.stopOdom();  // いったん停止
                    ROS_INFO("wall !!  move pause");
                }else
                if( fabs(turn_rad) > _allowable_angle){
                    //----------------------------------------------------------
                    //  進行方向にずれが生じた場合、一時停止後、進行方向を向く
                    //----------------------------------------------------------
                    driver.stopOdom();  // いったん停止
                    if(turn == true ){ 
                       ROS_INFO("right turn_rad(%f)",RAD2DEG(turn_rad));
                    }else{
                       ROS_INFO("left  turn_rad(%f)",RAD2DEG(turn_rad));
                    }
                    driver.turnOdom(turn, turn_rad);  // 旋回動作
                }else{
                    //----------------------------------------------------------
                    // 直進させる
                    //----------------------------------------------------------
                    double linearSpeed = 0;
                    if( goal_range > _distance_short ){
                        linearSpeed = _linear_speed_long;
                    }else
                    if( goal_range > _distance_near  ){
                        linearSpeed = _linear_speed_short;
                    }else{
                        linearSpeed = _linear_speed_near;
                    }

                    ROS_INFO("move forward  linearSpeed=%f", linearSpeed);
                    driver.moveForward(linearSpeed);  // 前進
                }
            }
        }

    }
};

//------------------------------------------------------------------------------
//  NAVIノード
//------------------------------------------------------------------------------
int main(int argc, char *argv[])
{
    ros::init(argc, argv, "delivery_navi_node");

    robot_navi_node robot_navi;

    ros::NodeHandle nh;
    ros::NodeHandle privateNode("~");

    RobotDriver driver(nh);

    robot_navi.setup( nh, privateNode);

    robot_navi.mainloop(driver);

}

