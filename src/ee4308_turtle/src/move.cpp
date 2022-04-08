#include <ros/ros.h>
#include <stdio.h>
#include <cmath>
#include <errno.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Empty.h>
#include "common.hpp"
#include <std_msgs/Float32.h>
#include <fstream>

bool target_changed = false;
Position target;
void cbTarget(const geometry_msgs::PointStamped::ConstPtr &msg)
{
    target.x = msg->point.x;
    target.y = msg->point.y;
}

Position pos_rbt(0, 0);
double ang_rbt = 10; // set to 10, because ang_rbt is between -pi and pi, and integer for correct comparison while waiting for motion to load
void cbPose(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    auto &p = msg->pose.position;
    pos_rbt.x = p.x;
    pos_rbt.y = p.y;

    // euler yaw (ang_rbt) from quaternion <-- stolen from wikipedia
    auto &q = msg->pose.orientation; // reference is always faster than copying. but changing it means changing the referenced object.
    double siny_cosp = 2 * (q.w * q.z + q.x * q.y);
    double cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
    ang_rbt = atan2(siny_cosp, cosy_cosp);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "turtle_move");
    ros::NodeHandle nh;
    // for recording data
    std::ofstream data_file;
    data_file.open("/home/pgokul/team01/data/move_data.txt");  

    // Get ROS Parameters
    bool enable_move;
    if (!nh.param("enable_move", enable_move, true))
        ROS_WARN(" TMOVE : Param enable_move not found, set to true");
    bool verbose;
    if (!nh.param("verbose_move", verbose, false))
        ROS_WARN(" TMOVE : Param verbose_move not found, set to false");
    double Kp_lin;
    if (!nh.param("Kp_lin", Kp_lin, 1.0))
        ROS_WARN(" TMOVE : Param Kp_lin not found, set to 1.0");
    double Ki_lin;
    if (!nh.param("Ki_lin", Ki_lin, 0.0))
        ROS_WARN(" TMOVE : Param Ki_lin not found, set to 0");
    double Kd_lin;
    if (!nh.param("Kd_lin", Kd_lin, 0.0))
        ROS_WARN(" TMOVE : Param Kd_lin not found, set to 0");
    double max_lin_vel;
    if (!nh.param("max_lin_vel", max_lin_vel, 0.22))
        ROS_WARN(" TMOVE : Param max_lin_vel not found, set to 0.22");
    double max_lin_acc;
    if (!nh.param("max_lin_acc", max_lin_acc, 1.0))
        ROS_WARN(" TMOVE : Param max_lin_acc not found, set to 1");
    double Kp_ang;
    if (!nh.param("Kp_ang", Kp_ang, 1.0))
        ROS_WARN(" TMOVE : Param Kp_ang not found, set to 1.0");
    double Ki_ang;
    if (!nh.param("Ki_ang", Ki_ang, 0.0))
        ROS_WARN(" TMOVE : Param Ki_ang not found, set to 0");
    double Kd_ang;
    if (!nh.param("Kd_ang", Kd_ang, 0.0))
        ROS_WARN(" TMOVE : Param Kd_ang not found, set to 0");
    double max_ang_vel;
    if (!nh.param("max_ang_vel", max_ang_vel, 2.84))
        ROS_WARN(" TMOVE : Param max_ang_vel not found, set to 2.84");
    double max_ang_acc;
    if (!nh.param("max_ang_acc", max_ang_acc, 4.0))
        ROS_WARN(" TMOVE : Param max_ang_acc not found, set to 4");
    double move_iter_rate;
    if (!nh.param("move_iter_rate", move_iter_rate, 25.0))
        ROS_WARN(" TMOVE : Param move_iter_rate not found, set to 25");

    // Subscribers
    ros::Subscriber sub_target = nh.subscribe("target", 1, &cbTarget);
    ros::Subscriber sub_pose = nh.subscribe("pose", 1, &cbPose);

    // Publishers
    ros::Publisher pub_cmd = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1, true);
    ros::Publisher pos_error_pub = nh.advertise<std_msgs::Float32>("error_pos", 1);
    ros::Publisher ang_error_pub = nh.advertise<std_msgs::Float32>("error_ang", 1);

    // prepare published messages
    geometry_msgs::Twist msg_cmd; // all properties are initialised to zero.

    // Setup rate
    ros::Rate rate(move_iter_rate); // same as publishing rate of pose topic

    // wait for other nodes to load
    ROS_INFO(" TMOVE : Waiting for topics");
    while (ros::ok() && nh.param("run", true) && ang_rbt == 10) // not dependent on main.cpp, but on motion.cpp
    {
        rate.sleep();
        ros::spinOnce(); //update the topics
    }

    // Setup variables
    double cmd_lin_vel = 0, cmd_ang_vel = 0;
    double dt;
    double prev_time = ros::Time::now().toSec();

    ////////////////// DECLARE VARIABLES HERE //////////////////
    double error_pos = dist_euc(pos_rbt, target);
    double error_pos_prev = error_pos;
    double I_lin = 0;
    double D_lin = 0;
    
    double error_ang = limit_angle(heading(pos_rbt, target) - ang_rbt);
    double error_ang_prev = error_ang;
	double I_ang = 0;
	double D_ang = 0;

    double control_sig_lin;
    double control_sig_ang;

    double estimated_lin_acc;
    double estimated_ang_acc;
    double sat_lin_acc;
    double sat_ang_acc;
    bool reverse = false;
    double error_ang_tmp;

    //for rqt plot
    std_msgs::Float32 error_pos_data;
    std_msgs::Float32 error_ang_data;

    //for data collection
    double time_start = ros::Time::now().toSec();

    ROS_INFO(" TMOVE : ===== BEGIN =====");

    // main loop
    if (enable_move)
    {
        while (ros::ok() && nh.param("run", true))
        {
            // update all topics
            ros::spinOnce();

            dt = ros::Time::now().toSec() - prev_time;
            if (dt == 0) // ros doesn't tick the time fast enough
                continue;
            prev_time += dt;

            ////////////////// MOTION CONTROLLER HERE //////////////////
            // update previous errors
            error_pos_prev = error_pos;
            error_ang_prev = error_ang;

            // calculate new errors
            error_pos = dist_euc(pos_rbt, target);
            error_ang = limit_angle(heading(pos_rbt, target) - ang_rbt);

            /*// calculate integral terms
            I_lin += error_pos * dt;
            I_ang += error_ang * dt;

            //calculate derivative terms
            D_lin = (error_pos - error_pos_prev) / dt;
            D_ang = (error_ang - error_ang_prev) / dt;

            //calculate control signals 
            control_sig_lin = Kp_lin * error_pos + Kd_lin * D_lin + Ki_lin * I_lin;
            control_sig_ang = Kp_ang * error_ang + Kd_ang * D_ang + Ki_ang * I_ang;

            //coupling angular error with linear velocity
            if (error_ang < -M_PI / 2 || error_ang > M_PI / 2)
            {
                control_sig_lin = 0;
            }
            else 
            {
                control_sig_lin = control_sig_lin * pow(cos(error_ang), 3);
            }*/

            if (error_ang < -M_PI / 2 || error_ang > M_PI / 2)
            {
                reverse = true;
                if (error_ang < M_PI / 2)
                {
                    error_ang_tmp = error_ang + M_PI;
                }
                else 
                {
                    error_ang_tmp = error_ang - M_PI;
                }
            }
            else 
            {
                reverse = false;
            }

            if (reverse)
            {
                // calculate integral terms               
                I_ang += error_ang_tmp * dt;
            }
            else 
            {
                // calculate integral terms
                I_ang += error_ang * dt;
            }
            I_lin += error_pos * dt;

            //calculate derivative terms
            D_lin = (error_pos - error_pos_prev) / dt;
            D_ang = (error_ang - error_ang_prev) / dt;


            //calculate control signals 
            control_sig_lin = Kp_lin * error_pos + Kd_lin * D_lin + Ki_lin * I_lin;
            if (reverse)
            {
                control_sig_ang = Kp_ang * error_ang_tmp + Kd_ang * D_ang + Ki_ang * I_ang;
            }
            else 
            {
                control_sig_ang = Kp_ang * error_ang + Kd_ang * D_ang + Ki_ang * I_ang;
            }
            
            control_sig_lin = control_sig_lin * pow(cos(error_ang), 5);


            //constraining velocities
            estimated_lin_acc = (control_sig_lin - cmd_lin_vel) / dt;
            estimated_ang_acc = (control_sig_ang - cmd_ang_vel) / dt;

            sat_lin_acc = sat(estimated_lin_acc, max_lin_acc);
            sat_ang_acc = sat(estimated_ang_acc, max_ang_acc);

            cmd_lin_vel = sat(cmd_lin_vel + sat_lin_acc * dt, max_lin_vel);
            cmd_ang_vel = sat(cmd_ang_vel + sat_ang_acc * dt, max_ang_vel);

            // publish speeds
            msg_cmd.linear.x = cmd_lin_vel;
            msg_cmd.angular.z = cmd_ang_vel;
            if (abs(target.x) < 1e-5 && abs(target.y) < 1e-5) {
                msg_cmd.linear.x = 0;
                msg_cmd.angular.z = 0;
            }
            pub_cmd.publish(msg_cmd);

            // publish errors
            error_pos_data.data = error_pos;
            error_ang_data.data = error_ang;

            pos_error_pub.publish(error_pos_data);
            ang_error_pub.publish(error_ang_data);

            // write data to file 
            double time_diff = ros::Time::now().toSec() - time_start;
            data_file << time_diff << "\t" << error_pos << "\t" << heading(pos_rbt, target) << "\t" << ang_rbt << "\t" << error_ang << "\t" << target.x << "\t" << target.y << "\t" << std::endl;

            // verbose
            if (verbose)
            {
                ROS_INFO(" TMOVE :  FV(%6.3f) AV(%6.3f)", cmd_lin_vel, cmd_ang_vel);
            }

            // wait for rate
            rate.sleep();
        }
    }

    // attempt to stop the motors (does not work if ros wants to shutdown)
    msg_cmd.linear.x = 0;
    msg_cmd.angular.z = 0;
    pub_cmd.publish(msg_cmd);

    ROS_INFO(" TMOVE : ===== END =====");
    // close file
    data_file.close();
    return 0;
}
