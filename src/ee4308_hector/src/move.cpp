#include <ros/ros.h>
#include <stdio.h>
#include <cmath>
#include <limits>
#include <errno.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Empty.h>
#include <hector_uav_msgs/EnableMotors.h>
#include <std_msgs/Bool.h>
#include <opencv2/core/core.hpp>
#include <fstream>
#include <signal.h>
#include "common.hpp"
#define NaN std::numeric_limits<double>::quiet_NaN()

ros::ServiceClient en_mtrs;
void disable_motors(int sig)
{
    ROS_INFO(" HMOVE : Disabling motors...");
    hector_uav_msgs::EnableMotors en_mtrs_srv;
    en_mtrs_srv.request.enable = false;
    en_mtrs.call(en_mtrs_srv); 
}

double target_x = NaN, target_y = NaN, target_z = NaN;
void cbTarget(const geometry_msgs::PointStamped::ConstPtr &msg)
{
    target_x = msg->point.x;
    target_y = msg->point.y;
    target_z = msg->point.z;
}

double x = NaN, y = NaN, z = NaN, a = NaN;
void cbPose(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg)
{
    auto &p = msg->pose.pose.position;
    x = p.x;
    y = p.y;
    z = p.z;

    // euler yaw (ang_rbt) from quaternion <-- stolen from wikipedia
    auto &q = msg->pose.pose.orientation; // reference is always faster than copying. but changing it means changing the referenced object.
    double siny_cosp = 2 * (q.w * q.z + q.x * q.y);
    double cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
    a = atan2(siny_cosp, cosy_cosp);
}

bool rotate = false;
void cbRotate(const std_msgs::Bool::ConstPtr &msg)
{
    rotate = msg->data;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "turtle_move");
    ros::NodeHandle nh;

    // --------- parse parameters ----------
    bool enable_move;
    if (!nh.param("enable_move", enable_move, true))
        ROS_WARN(" HMOVE : Param enable_move not found, set to true");
    if (!enable_move)
        return 0;
    bool verbose;
    if (!nh.param("verbose_move", verbose, false))
        ROS_WARN(" HMOVE : Param verbose_move not found, set to false");
    double Kp_lin;
    if (!nh.param("Kp_lin", Kp_lin, 1.0))
        ROS_WARN(" HMOVE : Param Kp_lin not found, set to 1.0");
    double Ki_lin;
    if (!nh.param("Ki_lin", Ki_lin, 0.0))
        ROS_WARN(" HMOVE : Param Ki_lin not found, set to 0");
    double Kd_lin;
    if (!nh.param("Kd_lin", Kd_lin, 0.0))
        ROS_WARN(" HMOVE : Param Kd_lin not found, set to 0");
    double Kp_z;
    if (!nh.param("Kp_z", Kp_z, 1.0))
        ROS_WARN(" HMOVE : Param Kp_z not found, set to 1.0");
    double Ki_z;
    if (!nh.param("Ki_z", Ki_z, 0.0))
        ROS_WARN(" HMOVE : Param Ki_lin not found, set to 0");
    double Kd_z;
    if (!nh.param("Kd_z", Kd_z, 0.0))
        ROS_WARN(" HMOVE : Param Kd_lin not found, set to 0");
    double yaw_rate;
    if (!nh.param("yaw_rate", yaw_rate, 0.5))
        ROS_WARN(" HMOVE : Param yaw_rate not found, set to 0.5");
    double max_lin_vel;
    if (!nh.param("max_lin_vel", max_lin_vel, 2.0))
        ROS_WARN(" HMOVE : Param max_lin_vel not found, set to 2");
    double max_z_vel;
    if (!nh.param("max_z_vel", max_z_vel, 0.5))
        ROS_WARN(" HMOVE : Param max_z_vel not found, set to 0.5");
    double move_iter_rate;
    if (!nh.param("move_iter_rate", move_iter_rate, 25.0))
        ROS_WARN(" HMOVE : Param move_iter_rate not found, set to 25");
    
    // --------- Enable Motors ----------
    ROS_INFO(" HMOVE : Enabling motors...");
    en_mtrs = nh.serviceClient<hector_uav_msgs::EnableMotors>("enable_motors");
    hector_uav_msgs::EnableMotors en_mtrs_srv;
    en_mtrs_srv.request.enable = true;
    if (en_mtrs.call(en_mtrs_srv))
        ROS_INFO(" HMOVE : Motors enabled!");
    else
        ROS_WARN(" HMOVE : Cannot enable motors!");
    signal(SIGINT, disable_motors);

    // --------- Subscribers ----------
    ros::Subscriber sub_target = nh.subscribe("target", 1, &cbTarget);
    ros::Subscriber sub_pose = nh.subscribe("pose", 1, &cbPose);
    ros::Subscriber sub_rotate = nh.subscribe("rotate", 1, &cbRotate);

    // --------- Publishers ----------
    ros::Publisher pub_cmd = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1, true);
    geometry_msgs::Twist msg_cmd; // all properties are initialised to zero.

    // --------- Wait for Topics ----------
    ROS_INFO(" HMOVE : Waiting for topics");
    while (ros::ok() && nh.param("run", true) && (std::isnan(target_x) || std::isnan(x))) // not dependent on main.cpp, but on motion.cpp
        ros::spinOnce(); // update the topics

    // --------- Begin Controller ----------
    ROS_INFO(" HMOVE : ===== BEGIN =====");
    ros::Rate rate(move_iter_rate); // same as publishing rate of pose topic
    double cmd_lin_vel_x, cmd_lin_vel_y, cmd_lin_vel_z, cmd_lin_vel_a;
    double dt;
    double prev_time = ros::Time::now().toSec();

    ////////////////// DECLARE VARIABLES HERE //////////////////
    double pos_error_x = x - target_x;
    double pos_error_y = y - target_y;
    double pos_error_z = z - target_z;
    double prev_pos_error_x = pos_error_x;
    double prev_pos_error_y = pos_error_y;
    double prev_pos_error_z = pos_error_z;
    double summation_pos_x = pos_error_x;
    double summation_pos_y = pos_error_y;
    double summation_pos_z = pos_error_z;

    double P_constant_x = 0;
    double P_constant_y = 0;
    double P_constant_z = 0;
    double D_constant_x = 0;
    double D_constant_y = 0;
    double D_constant_z = 0;
    double I_constant_x = 0;
    double I_constant_y = 0;
    double I_constant_z = 0;
    double output_lin_x = 0;
    double output_lin_y = 0;
    double output_lin_z = 0;

    double lin_acc_x = 0;
    double lin_acc_y = 0;
    double lin_acc_z = 0;

    // main loop
    while (ros::ok() && nh.param("run", true))
    {
        // update all topics
        ros::spinOnce();

        dt = ros::Time::now().toSec() - prev_time;
        if (dt == 0) // ros doesn't tick the time fast enough
            continue;
        prev_time += dt;

        //// PID controller here ////
        // X-PID //
        pos_error_x = target_x - x;
        summation_pos_x += pos_error_x * dt;

        P_constant_x = Kp_lin * pos_error_x;
        I_constant_x = Ki_lin * summation_pos_x;
        D_constant_x = Kd_lin * ((pos_error_x - prev_pos_error_x) / dt);

        output_lin_x = P_constant_x + D_constant_x + I_constant_x;
        prev_pos_error_x = pos_error_x;

        // Y-PID //
        pos_error_y = target_y - y;
        summation_pos_y += pos_error_y * dt;

        P_constant_y = Kp_lin * pos_error_y;
        I_constant_y = Ki_lin * summation_pos_y;
        D_constant_y = Kd_lin * ((pos_error_y - prev_pos_error_y) / dt);

        output_lin_y = P_constant_y + D_constant_y + I_constant_y;
        prev_pos_error_y = pos_error_y;

        // Z-PID //
        pos_error_z = target_z - z;
        summation_pos_z += pos_error_z * dt;

        P_constant_z = Kp_lin * pos_error_z;
        I_constant_z = Ki_lin * summation_pos_z;
        D_constant_z = Kd_lin * ((pos_error_z - prev_pos_error_z) / dt);

        output_lin_z = P_constant_z + D_constant_z + I_constant_z;
        prev_pos_error_z = pos_error_z;

        lin_acc_x = (output_lin_x - cmd_lin_vel_x) / dt;
        cmd_lin_vel_x = sat(output_lin_x + (lin_acc_x * dt), max_lin_vel);

        lin_acc_y = (output_lin_y - cmd_lin_vel_y) / dt;
        cmd_lin_vel_y = sat(output_lin_y + (lin_acc_y * dt), max_lin_vel);

        lin_acc_z = (output_lin_z - cmd_lin_vel_z) / dt;
        cmd_lin_vel_z = sat(output_lin_z + (lin_acc_z * dt), max_lin_vel);

        ROS_INFO(" HMOVE : The speeds are (%6.3f, %6.3f, %6.3f", cmd_lin_vel_x, cmd_lin_vel_y, cmd_lin_vel_z);


        // publish speeds
        msg_cmd.linear.x = cmd_lin_vel_x;
        msg_cmd.linear.y = cmd_lin_vel_y;
        msg_cmd.linear.z = cmd_lin_vel_z;
        msg_cmd.angular.z = cmd_lin_vel_a;
        pub_cmd.publish(msg_cmd);

        //// IMPLEMENT /////
        
        // verbose
        if (verbose)
        {
            // ROS_INFO(" HMOVE : Target(%6.3f, %6.3f, %6.3f) FV(%6.3f) VX(%6.3f) VY(%6.3f) VZ(%7.3f)", target_x, target_y, target_z, cmd_lin_vel, cmd_lin_vel_x, cmd_lin_vel_y, cmd_lin_vel_z);
        }

        // wait for rate
        rate.sleep();
    }

    // attempt to stop the motors (does not work if ros wants to shutdown)
    msg_cmd.linear.x = 0;
    msg_cmd.linear.y = 0;
    msg_cmd.linear.z = 0;
    msg_cmd.angular.z = 0;
    pub_cmd.publish(msg_cmd);

    // disable motors
    ROS_INFO(" HMOVE : Motors Disabled");
    en_mtrs_srv.request.enable = false;
    en_mtrs.call(en_mtrs_srv);

    ROS_INFO(" HMOVE : ===== END =====");

    return 0;
}