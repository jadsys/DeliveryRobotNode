/*
    �i�r�Q�[�V�����p�b�P�[�W�𗘗p���āA�����̖ړI�n�ɏ��ԂɈړ�����
*/
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <RobotDriver.cpp>
#include <sensor_msgs/LaserScan.h>

#ifndef M_PI
#define M_PI 3.14159265358979             // �~����
#endif
#define DEG2RAD(x) ((x)*M_PI/180)  // �x���烉�W�A��
#define RAD2DEG(x) ((x)*180/M_PI)  // ���W�A������x

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
    ros::Subscriber sub_scan; // /scan�p �T�u�X�N���C�o
    ros::Subscriber sub_goal; // /goal�p �T�u�X�N���C�o
    tf::TransformListener tfl;

    geometry_msgs::PoseStamped current_goal;                // �ړI�n
    std::list<geometry_msgs::PoseStamped> goals;

    sensor_msgs::LaserScan latest_scan; // /scan�g�s�b�N��M�l��ۑ�        �^�m�F�R�}���h�Frostopic info /scan

    // /scan�g�s�b�N����M����R�[���o�b�N�֐�
    void cb_scan(const sensor_msgs::LaserScan::ConstPtr &msg)
    {
        // �󂯎�������b�Z�[�W���R�s�[���Ă���
        latest_scan = *msg;
    }

    // /goal�g�s�b�N����M����R�[���o�b�N�֐�
    void cb_goal(const geometry_msgs::PoseStamped::ConstPtr &msg)
    {
        // �󂯎�������b�Z�[�W���R�s�[���Ă���
        ROS_INFO("Get goal point!!");
        current_goal = *msg;
    }

    double _wall_judgment_width;        // �ǔ���   ��
    double _wall_judgment_distance;     // �ǔ���   ����

    double _linear_speed_long;  // ���s���x(����)
    double _linear_speed_short; // ���s���x(�߂�)
    double _linear_speed_near;  // ���s���x(�S�[���t��)

    double _distance_short;  // ���s���x�؂�ւ�����(�߂�)
    double _distance_near;   // ���s���x�؂�ւ�����(�S�[���t��))
    double _distance_goal;   // �S�[�����苗��
    double _allowable_angle; // �S�[�����苗��

public:
    robot_navi_node()
    {
        ros::NodeHandle nh("~");//�v���C�x�[�g�p�����[�^��ǂ݂������߂̃m�[�h�n���h��

        current_goal.pose.position.x = FLT_MAX;
        current_goal.pose.position.y = FLT_MAX;

        //  -----   �p�u���b�V��    ------

        //  -----   �T�u�X�N���C�o  ------
        //      (scan����M����)
        sub_scan = nh.subscribe("/scan", 5,                         // "/scan"�F�g�s�b�N��      5�F�L���[�T�C�Y
                                &robot_navi_node::cb_scan, this);   //  �`cb_scan�F�R�[���o�b�N�֐�
        //      (goal����M����)
        sub_goal = nh.subscribe("/move_base_simple/goal", 5,        // "/goal"�F�g�s�b�N��      5�F�L���[�T�C�Y
                                &robot_navi_node::cb_goal, this);   //  �`cb_goal�F�R�[���o�b�N�֐�
    }

    //------------------------------------------------------------------------------
    //  �����ݒ�
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
    // �ǔ���
    //--------------------------------------------------------------------------
    bool wall_scan()
    {
        bool blret = false;

        if(latest_scan.ranges.size() > 0)
        {
            // LaserScan���b�Z�[�W�����łɎ󂯎���Ă���ꍇ

            float hx = 0, hy = 0;
            int hnum = 0;

            // theta-range ���W�n���� x-y ���W�n�ɕϊ�
            for(unsigned int i = 0; i < latest_scan.ranges.size(); i ++)
            {
                if(!(latest_scan.ranges[i] < latest_scan.range_min ||
                     latest_scan.ranges[i] > latest_scan.range_max ||
                     std::isnan(latest_scan.ranges[i])))                // ���l�� NaN �ł��邩���肷��B
                {
                    // �����l���G���[�łȂ��ꍇ

                    float theta = latest_scan.angle_min + (latest_scan.angle_increment * i);       // angle_increment�F����Ԃ̊p����[rad]
                    // x-y ���W�n�ɕϊ�
                    float x = latest_scan.ranges[i] * cosf(theta);
                    float y = latest_scan.ranges[i] * sinf(theta);

                    if(fabs(y) <= (_wall_judgment_width/2) && x >= 0.05 && x <= _wall_judgment_distance){   // �� 30cm ����35cm�ȓ�
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
                blsts = true;   // �E����
                turn_rad = current_rad - goal_rad;
            }else{
                blsts = false;  // ������
                turn_rad = 2*M_PI - (current_rad - goal_rad);
            }
        }else{
            if( (goal_rad - current_rad) <= M_PI ){
                turn_rad = goal_rad - current_rad;
                blsts = false;  // ������
            }else{
                turn_rad = 2*M_PI - (goal_rad - current_rad);
                blsts = true;   // �E����
            }
        }

        return( blsts );
    }

    //--------------------------------------------------------------------------
    //  ���C�����[�v
    //--------------------------------------------------------------------------
    void mainloop(RobotDriver& driver)
    {
        ROS_INFO("navi node start");

        ros::Rate rate(10.0);   // 10Hz����
        while(ros::ok())
        {
            rate.sleep();
            ros::spinOnce();

            if(current_goal.pose.position.x == FLT_MAX && current_goal.pose.position.y == FLT_MAX){
                continue;   // �S�[������
            }

            double x, y, yaw;
            try
            {
                tf::StampedTransform trans;
                tfl.waitForTransform("map", "base_link",    
                        ros::Time(0), ros::Duration(0.5));
                tfl.lookupTransform("map", "base_link",   // map���猩��base_link�A(world���W�̃��{�b�g�̈ʒu)
                        ros::Time(0), trans);
                x = trans.getOrigin().x();                // �w���W
                y = trans.getOrigin().y();                // �x���W
                yaw = tf::getYaw(trans.getRotation());    // �l��������yaw�p���擾
            }
            catch(tf::TransformException &e)
            {
                ROS_WARN("%s", e.what());
                continue;
            }

            double yaw_goal = GetMsgYaw(current_goal.pose.orientation); //  �S�[���̌������擾(�l���g�p)

            //------------------------------------------------------------------
            //  �S�[������
            //------------------------------------------------------------------
            if(hypotf(x - current_goal.pose.position.x,                         //  hypotf:�ړI�l�܂ł̋������擾
                        y - current_goal.pose.position.y) <= _distance_goal)    //  �ړI�l�܂ł̋��������苗�����߂�
            {
                ROS_INFO("GOAL Stop !!");

                // �S�[��
                current_goal.pose.position.x = FLT_MAX;
                current_goal.pose.position.y = FLT_MAX;
                driver.stopOdom();  // ���������~

            }else
            {
                double goal_range = hypot(x - current_goal.pose.position.x, y - current_goal.pose.position.y);
                double turn_rad = 0; //����p�x

                double yaw_way = atan2((double)(current_goal.pose.position.y - y), (double)(current_goal.pose.position.x - x)); // goal�̕���
                double turn = rad_func( yaw, yaw_way, turn_rad);
                ROS_INFO("goal_range=%f yaw=%f yaw_goal=%f yaw_way=%f turn_rad=%f", goal_range,  yaw, yaw_goal, yaw_way, turn_rad);

                if( wall_scan() == true ){
                    //----------------------------------------------------------
                    //  ��Q�����m
                    //----------------------------------------------------------
                    driver.stopOdom();  // ���������~
                    ROS_INFO("wall !!  move pause");
                }else
                if( fabs(turn_rad) > _allowable_angle){
                    //----------------------------------------------------------
                    //  �i�s�����ɂ��ꂪ�������ꍇ�A�ꎞ��~��A�i�s����������
                    //----------------------------------------------------------
                    driver.stopOdom();  // ���������~
                    if(turn == true ){ 
                       ROS_INFO("right turn_rad(%f)",RAD2DEG(turn_rad));
                    }else{
                       ROS_INFO("left  turn_rad(%f)",RAD2DEG(turn_rad));
                    }
                    driver.turnOdom(turn, turn_rad);  // ���񓮍�
                }else{
                    //----------------------------------------------------------
                    // ���i������
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
                    driver.moveForward(linearSpeed);  // �O�i
                }
            }
        }

    }
};

//------------------------------------------------------------------------------
//  NAVI�m�[�h
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

