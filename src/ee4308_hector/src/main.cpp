#include <ros/ros.h>
#include <stdio.h>
#include <stdlib.h>
#include <cmath>
#include <errno.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Bool.h>
#include <opencv2/core/core.hpp>
#includ "trajectory.hpp"
#include "common.hpp"
#define NaN std::numeric_limits<double>::quiet_NaN()

enum HectorState
{
    TAKEOFF,
    LAND,
    TURTLE,
    START,
    GOAL
};
std::string to_string(HectorState state)
{
    switch (state)
    {
    case TAKEOFF:
        return "TAKEOFF";
    case LAND:
        return "LAND";
    case TURTLE:
        return "TURTLE";
    case START:
        return "START";
    case GOAL:
        return "GOAL";
    default:
        return "??";
    }
}

bool verbose;
double initial_x, initial_y, initial_z;
double x = NaN, y = NaN, z = NaN, a = NaN;
void cbHPose(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg)
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
double turtle_x = NaN, turtle_y = NaN;
void cbTPose(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    auto &p = msg->pose.position;
    turtle_x = p.x;
    turtle_y = p.y;
}
double vx = NaN, vy = NaN, vz = NaN, va = NaN;
void cbHVel(const geometry_msgs::Twist::ConstPtr &msg)
{
    vx = msg->linear.x;
    vy = msg->linear.y;
    vz = msg->linear.z;
    va = msg->angular.z;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "hector_main");
    ros::NodeHandle nh;

    // Make sure motion and move can run (fail safe)
    nh.setParam("run", true); // turns off other nodes

    double main_iter_rate;
    if (!nh.param("main_iter_rate", main_iter_rate, 25.0))
        ROS_WARN(" HMAIN : Param main_iter_rate not found, set to 25");
    if (!nh.param("initial_x", initial_x, 0.0))
        ROS_WARN(" HMAIN : Param initial_x not found, set initial_x to 0.0");
    if (!nh.param("initial_y", initial_y, 0.0))
        ROS_WARN(" HMAIN : Param initial_y not found, set initial_y to 0.0");
    if (!nh.param("initial_z", initial_z, 0.178))
        ROS_WARN(" HMAIN : Param initial_z not found, set initial_z to 0.178");
    double height;
    if (!nh.param("height", height, 2.0))
        ROS_WARN(" HMAIN : Param initial_z not found, set to 5");
    double look_ahead;
    if (!nh.param("look_ahead", look_ahead, 1.0))
        ROS_WARN(" HMAIN : Param look_ahead not found, set to 1");
    double close_enough;
    if (!nh.param("close_enough", close_enough, 0.1))
        ROS_WARN(" HMAIN : Param close_enough not found, set to 0.1");
    double average_speed;
    if (!nh.param("average_speed", average_speed, 2.0))
        ROS_WARN(" HMAIN : Param average_speed not found, set to 2.0");
    if (!nh.param("verbose_main", verbose, true))
        ROS_WARN(" HMAIN : Param verbose_main not found, set to false");
    // get the final goal position of turtle
    std::string goal_str;
    double goal_x = NaN, goal_y = NaN;
    if (nh.param("/turtle/goals", goal_str, std::to_string(initial_x) + "," + std::to_string(initial_y))) // set to initial hector positions
    {
        char goal_str_tok[goal_str.length() + 1];
        strcpy(goal_str_tok, goal_str.c_str()); // to tokenise --> convert to c string (char*) first
        char *tok = strtok(goal_str_tok, " ,");
        try
        {
            while (tok != nullptr)
            {
                goal_x = strtod(tok, nullptr);
                goal_y = strtod(strtok(nullptr, " ,"), nullptr);
                tok = strtok(nullptr, " ,");
            }
            ROS_INFO(" HMAIN : Last Turtle Goal is (%lf, %lf)", goal_x, goal_y);
        }
        catch (...)
        {
            ROS_ERROR(" HMAIN : Invalid Goals: %s", goal_str.c_str());
            ros::shutdown();
            return 1;
        }
    }
    else
        ROS_WARN(" HMAIN : Param goal not found, set to %s", goal_str.c_str());

    // --------- Subscribers ----------
    ros::Subscriber sub_hpose = nh.subscribe("pose", 1, &cbHPose);
    ros::Subscriber sub_tpose = nh.subscribe("/turtle/pose", 1, &cbTPose);
    ros::Subscriber sub_hvel = nh.subscribe("velocity", 1, &cbHVel);

    // --------- Publishers ----------
    ros::Publisher pub_target = nh.advertise<geometry_msgs::PointStamped>("target", 1, true);
    geometry_msgs::PointStamped msg_target;
    msg_target.header.frame_id = "world";  // for rviz
    ros::Publisher pub_rotate = nh.advertise<std_msgs::Bool>("rotate", 1, true);
    std_msgs::Bool msg_rotate;
    ros::Publisher pub_traj = nh.advertise<nav_msgs::Path>("trajectory", 1, true);
    nav_msgs::Path msg_traj;
    msg_traj.header.frame_id = "world";
    // state = TAKEOFF;

    // --------- Wait for Topics ----------
    while (ros::ok() && nh.param("run", true) && (std::isnan(x) || std::isnan(turtle_x) || std::isnan(vx))) // not dependent on main.cpp, but on motion.cpp
        ros::spinOnce();                                                                                    // update the topics

    // --------- Main loop ----------
    ROS_INFO(" HMAIN : ===== BEGIN =====");
    HectorState state = TAKEOFF;
    ros::Rate rate(main_iter_rate);
    double prev_time = ros::Time::now().toSec();
    double prev_turtle_x = turtle_x;
    double prev_turtle_y = turtle_y;
    double dx = 0;
    double dy = 0;
    double dt;
    double distance_hector_turtle = 0;
    double distance_hector_goal = 0;
    double distance_hector_start = 0;
    double target_x, target_y;
    std::vector<Position> trajectory;
    Position pos_target;
    int t=0;

    while (ros::ok() && nh.param("run", true))
    {
        // get topics
        ros::spinOnce();

        // ROS_INFO(" HMAIN : Drone's state is %d", state==TAKEOFF);

        dt = ros::Time::now().toSec() - prev_time;
        if (dt == 0) // ros doesn't tick the time fast enough
            continue;
        prev_time += dt;

        //// IMPLEMENT ////
        if (state == TAKEOFF)
        {
            msg_target.point.x = initial_x;
            msg_target.point.y = initial_y;
            msg_target.point.z = initial_z + 2;
            pub_target.publish(msg_target);
            ROS_INFO(" HMAIN : Drone's state is TAKEOFF");
            // Disable Rotate
            msg_rotate.data = true;
            pub_rotate.publish(msg_rotate);
            if(z >= 1.8 and z <= 2.2){
                state = TURTLE;
            } 
        }
        else if (state == TURTLE)
        {
            if(trajectory.empty())
            {
                dx = (turtle_x - prev_turtle_x) / dt;
                dy = (turtle_y - prev_turtle_y) / dt;
                target_x = turtle_x + dx * look_ahead;
                target_y = turtle_y + dy * look_ahead;
                prev_turtle_x = turtle_x;
                prev_turtle_y = turtle_y;
                trajectory = generate_trajectory(//To be implemented
                    initial_x, initial_y, initial_z,
                    goal_x, goal_y, height,
                    look_ahead, close_enough, average_speed,
                    dx, dy, dt);
                pos_target = trajectory[t];
            }
            else if(dist_euc(x,y,trajectory[t].x,trajectory[t].y) < close_enough)
            {
                t++;
                if(t >= trajectory.size())
                {
                    trajectory.clear();
                    t = 0;
                }
                else
                {   
                    pos_target = trajectory[t];
                    msg_target.point.x = pos_target.x
                    msg_target.point.y = pos_target.y
                    pub_target.publish(msg_target);
                }
                
            }
            distance_hector_turtle = pow((x - turtle_x),2) + pow((y - turtle_y),2);
            distance_hector_turtle = sqrt(distance_hector_turtle);
            ROS_INFO(" HMAIN : Drone's state is TURTLE");
            if (distance_hector_turtle <= close_enough){
                state = GOAL;
                trajectory.clear();
                t = 0;
            }  
        }
        else if (state == START)
        {
            if(trajectory.empty())
            {
                target_x = initial_x;
                target_y = initial_y;
                trajectory = generate_trajectory(//To be implemented
                    initial_x, initial_y, initial_z,
                    goal_x, goal_y, height,
                    look_ahead, close_enough, average_speed,
                    dx, dy, dt);
                pos_target = trajectory[t];
            }
            msg_target.point.x = initial_x;
            msg_target.point.y = initial_y;
            pub_target.publish(msg_target);

            distance_hector_start = pow((x - initial_x),2) + pow((y - initial_y),2);
            distance_hector_start = sqrt(distance_hector_start);
            ROS_INFO(" HMAIN : Drone's state is START");
            if (!nh.param("/turtle/run", false) and distance_hector_start <= close_enough)
            { // when the turtle reaches the final goal
                state = LAND;
            }
            else if (distance_hector_start <= close_enough)
            {
                state = TURTLE; // SHOULD BE TURTLE;
            }
        }
        else if (state == GOAL)
        {
            msg_target.point.x = goal_x;
            msg_target.point.y = goal_y;
            pub_target.publish(msg_target);

            distance_hector_goal = pow((x - goal_x),2) + pow((y - goal_y),2);
            distance_hector_goal = sqrt(distance_hector_goal);
            ROS_INFO(" HMAIN : Drone's state is GOAL");
            if(distance_hector_goal <= close_enough){
                state = START;
            }           
        }
        else if (state == LAND)
        {
            msg_target.point.x = initial_x;
            msg_target.point.y = initial_y;
            msg_target.point.z = initial_z;
            pub_target.publish(msg_target);
            ROS_INFO(" HMAIN : Drone's state is LAND");
            if(z <= 0.378){
                // Disable Rotate
                msg_rotate.data = false;
                pub_rotate.publish(msg_rotate);
                break;
            }
        }

        if (verbose)
            ROS_INFO_STREAM(" HMAIN : " << to_string(state));

        rate.sleep();
    }

    nh.setParam("run", false); // turns off other nodes
    ROS_INFO(" HMAIN : ===== END =====");
    return 0;
}
