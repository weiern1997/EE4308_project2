#include <ros/ros.h>
#include <stdio.h>
#include <cmath>
#include <limits>
#include <errno.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h> // publish to pose topic
#include <geometry_msgs/Vector3Stamped.h>            // subscribe to magnetic topic
#include <sensor_msgs/Imu.h>                         // subscribe to imu topic
#include <sensor_msgs/NavSatFix.h>                   // subscribe to GPS
#include <hector_uav_msgs/Altimeter.h>               // subscribe to barometer
#include <sensor_msgs/Range.h>                       // subscribe to sonar
#include <nav_msgs/Odometry.h>                       // subscribe to ground truth topic
#include <std_srvs/Empty.h>                          // Service to calrbrate motors
#include <opencv2/core/core.hpp>
#include "common.hpp"

#define NaN std::numeric_limits<double>::quiet_NaN()

// global parameters to be read from ROS PARAMs
bool verbose, use_ground_truth, enable_baro, enable_magnet, enable_sonar, enable_gps;

// others
bool ready = false; // signal to topics to begin

// --------- PREDICTION WITH IMU ----------
const double G = 9.8;
double prev_imu_t = 0;
cv::Matx21d X = {0, 0}, Y = {0, 0}; // see intellisense. This is equivalent to cv::Matx<double, 2, 1>
cv::Matx21d A = {0, 0};
cv::Matx21d Z = {0, 0};
cv::Matx22d P_x = cv::Matx22d::ones(), P_y = cv::Matx22d::zeros();
cv::Matx22d P_a = cv::Matx22d::ones();
cv::Matx22d P_z = cv::Matx22d::ones();
double ua = NaN, ux = NaN, uy = NaN, uz = NaN;
double qa, qx, qy, qz;
// see https://docs.opencv.org/3.4/de/de1/classcv_1_1Matx.html
void cbImu(const sensor_msgs::Imu::ConstPtr &msg)
{
    if (!ready)
    {
        prev_imu_t = msg->header.stamp.toSec();
        return;
    }

    // calculate time
    double imu_t = msg->header.stamp.toSec();
    double imu_dt = imu_t - prev_imu_t;
    prev_imu_t = imu_t;

    // read inputs
    ua = msg->angular_velocity.z;
    ux = msg->linear_acceleration.x;
    uy = msg->linear_acceleration.y;
    uz = msg->linear_acceleration.z;
    
    //// IMPLEMENT IMU ////
    cv::Matx22d F_x = {1, imu_dt, 0, 1};
    cv::Matx22d W_x = {-0.5 * pow(imu_dt, 2) * cos(A(0)), 0.5 *  pow(imu_dt, 2) * sin(A(0)),
                        -imu_dt * cos(A(0)), imu_dt * sin(A(0))};
    cv::Matx21d U_x = {ux, uy};
    X = F_x * X + W_x * U_x;
    cv::Matx22d Q_x = {qx * qx, 0, 0, qy * qy};
    P_x = F_x * P_x * F_x.t() + W_x * Q_x * W_x.t();

    cv::Matx22d F_y = {1, imu_dt, 0, 1};
    cv::Matx22d W_y = {-0.5 * pow(imu_dt, 2) * sin(A(0)), -0.5 *  pow(imu_dt, 2) * cos(A(0)),
                        -imu_dt * sin(A(0)), -imu_dt * cos(A(0))};
    cv::Matx21d U_y = {ux, uy};
    Y = F_y * Y + W_y * U_y;
    cv::Matx22d Q_y = {qx * qx, 0, 0, qy * qy}; //check
    P_y = F_y * P_y * F_y.t() + W_y * Q_y * W_y.t();

    cv::Matx22d F_z = {1, imu_dt, 0, 1};
    cv::Matx22d W_z = {0.5 * pow(imu_dt, 2), -0.5 *  pow(imu_dt, 2),
                        imu_dt, -imu_dt};
    cv::Matx21d U_z = {uz, G};
    Z = F_z * Z + W_z * U_z;
    cv::Matx22d Q_z = {qz * qz}; //check
    P_z = F_z * P_z * F_z.t() + W_z * Q_z * W_z.t();

    cv::Matx22d F_phi = {1, 0, 0, 1};
    cv::Matx22d W_phi= {imu_dt, 0,
                        0, 1};
    cv::Matx21d U_phi = {A(1), A(1)};
    A = F_phi * A + W_phi * U_phi;
    cv::Matx22d Q_phi = {qa * qa}; //check
    P_a = F_phi * P_a * F_phi.t() + W_phi * Q_phi * W_phi.t();
}

// --------- GPS ----------
// https://docs.ros.org/en/api/sensor_msgs/html/msg/NavSatFix.html
cv::Matx31d GPS = {NaN, NaN, NaN};
cv::Matx31d initial_pos = {NaN, NaN, NaN}; // written below in main. no further action needed.
cv::Matx31d initial_ECEF = {NaN, NaN, NaN}; 
const double DEG2RAD = M_PI / 180;
const double RAD_POLAR = 6356752.3;
const double RAD_EQUATOR = 6378137;
const double e_sq = 1 - (RAD_POLAR * RAD_POLAR) / (RAD_EQUATOR * RAD_EQUATOR);
double r_gps_x, r_gps_y, r_gps_z;
void cbGps(const sensor_msgs::NavSatFix::ConstPtr &msg)
{
    if (!ready)
        return;
    cv::Matx31d ECEF = {NaN, NaN, NaN};
    cv::Matx33d R_en = {NaN, NaN, NaN, NaN, NaN, NaN, NaN, NaN, NaN};
    // IMPLEMENT GPS /////
    double lat = msg->latitude;
    double lon = msg->longitude;
    double alt = msg->altitude;

    lat = lat * DEG2RAD;
    lon = lon * DEG2RAD;
    
    double N_lat = RAD_EQUATOR / sqrt(1 - e_sq * sin(lat) * sin(lat));
    ECEF(0) = (N_lat + alt) * cos(lat) * cos(lon);
    ECEF(1) = (N_lat + alt) * cos(lat) * sin(lon);
    ECEF(2) = (N_lat * (1 - e_sq) + alt) * sin(lat);

    R_en(0, 0) = -sin(lat)*cos(lon);
    R_en(0, 1) = -sin(lon);
    R_en(0, 2) = -cos(lat)*cos(lon);
    R_en(1, 0) = -sin(lat)*sin(lon);
    R_en(1, 1) = cos(lon);
    R_en(1, 2) = -cos(lat)*sin(lon);
    R_en(2, 0) = cos(lat);
    R_en(2, 1) = 0;
    R_en(2, 2) = -sin(lat);
    
    // for initial message -- you may need this:
    if (std::isnan(initial_ECEF(0)))
    {   // calculates initial ECEF and returns
        initial_ECEF = ECEF;
        return;
    }
    cv::Matx31d NED = R_en.t() * (ECEF - initial_ECEF);
    cv::Matx33d R = {1,0,0,0,-1,0,0,0,-1};
    GPS = R * NED + initial_pos;
    cv::Matx12d H_gps = {1,0};
    cv::Mat1d V = {1};
    
    cv::Mat1d M_x = H_gps * P_x * H_gps.t() + V *r_gps_x* V.t();
    cv::Matx21d K_x = P_x * H_gps.t() * (1 / M_x(0));
    X = X + K_x * (GPS(0) - X(0));
    P_x = P_x - K_x * H_gps * P_x;

    cv::Mat1d M_y = H_gps * P_y * H_gps.t() + V *r_gps_y* V.t();
    cv::Matx21d K_y = P_y * H_gps.t() * (1 / M_y(0));
    Y = Y + K_y * (GPS(1) - Y(0));
    P_y = P_y - K_y * H_gps * P_y;

    cv::Mat1d M_z = H_gps * P_z * H_gps.t() + V *r_gps_z* V.t();
    cv::Matx21d K_z = P_z * H_gps.t() * (1 / M_z(0));
    Z = Z + K_z * (GPS(2) - Z(0));
    P_z = P_z - K_z * H_gps * P_z;
}

// --------- Magnetic ----------
double a_mgn = NaN;
double r_mgn_a;

void cbMagnet(const geometry_msgs::Vector3Stamped::ConstPtr &msg)
{
    if (!ready)
        return;
    
    //// IMPLEMENT GPS ////
    double mx = msg->vector.x;
    double my = msg->vector.y;

    a_mgn = - atan2(my, mx);
    r_mgn_a = r_mgn_a;//from hector.yaml
    

    cv::Mat1d Y = {a_mgn};
    // double Y = a_mgn;
    cv::Mat1d h_X = {A(0)};
    // // double h_X = A(0);
    cv::Matx12d H = {1, 0};
    cv::Mat1d V = {1};
    // // double V = 1;
    cv::Mat1d R = {r_mgn_a};
    // // double R = r_mgn_a;

    cv::Mat1d M = (H * P_a * H.t() + V * R * V);
    cv::Matx21d K = P_a * H.t() * (1/ M(0));
    // double K = P_a * H.t() * (H * P_a * H.t() + V * R * V).inv();
    A = A + K * (Y(0)- h_X(0));
    P_a = P_a - (K * H * P_a);
}

// --------- Baro ----------
double z_bar = NaN;
double r_bar_z;
void cbBaro(const hector_uav_msgs::Altimeter::ConstPtr &msg)
{
    if (!ready)
        return;

    //// IMPLEMENT BARO ////
    // z_bar = msg->altitude;

}

// --------- Sonar ----------
double z_snr = NaN;
double r_snr_z;
void cbSonar(const sensor_msgs::Range::ConstPtr &msg)
{
    if (!ready)
        return;

    // IMPLEMENT SONAR ////
    z_snr = msg->range;

    cv::Matx21d Z = {z_snr,0};
    cv::Matx12d H = {1,0};
    cv::Matx21d K = P_z * H.t() * (H * P_z * H.t() + r_snr_z).inv();

    Z(0) = Z(0) + K(0,0) * (z_snr - Z(0));
    P_z = P_z - K * H * P_z;
}

// --------- GROUND TRUTH ----------
nav_msgs::Odometry msg_true;
void cbTrue(const nav_msgs::Odometry::ConstPtr &msg)
{
    msg_true = *msg;
}

// --------- MEASUREMENT UPDATE WITH GROUND TRUTH ----------
int main(int argc, char **argv)
{
    ros::init(argc, argv, "hector_motion");
    ros::NodeHandle nh;

    // --------- parse parameters ----------
    double motion_iter_rate;
    if (!nh.param("motion_iter_rate", motion_iter_rate, 50.0))
        ROS_WARN("HMOTION: Param motion_iter_rate not found, set to 50.0");
    if (!nh.param("verbose_motion", verbose, false))
        ROS_WARN("HMOTION: Param verbose_motion not found, set to false");
    if (!nh.param("initial_x", X(0), 0.0))
        ROS_WARN("HMOTION: Param initial_x not found, set initial_x to 0.0");
    if (!nh.param("initial_y", Y(0), 0.0))
        ROS_WARN("HMOTION: Param initial_y not found, set initial_y to 0.0");
    if (!nh.param("initial_z", Z(0), 0.178))
        ROS_WARN("HMOTION: Param initial_z not found, set initial_z to 0.178");
    initial_pos = {X(0), Y(0), Z(0)};
    if (!nh.param("use_ground_truth", use_ground_truth, true))
        ROS_WARN("HMOTION: Param use_ground_truth not found, set use_ground_truth to true");
    if (!nh.param("r_gps_x", r_gps_x, 1.0))
        ROS_WARN("HMOTION: Param r_gps_x not found, set to 1.0");
    if (!nh.param("r_gps_y", r_gps_y, 1.0))
        ROS_WARN("HMOTION: Param r_gps_y not found, set to 1.0");
    if (!nh.param("r_gps_z", r_gps_z, 1.0))
        ROS_WARN("HMOTION: Param r_gps_z not found, set to 1.0");
    if (!nh.param("r_mgn_a", r_mgn_a, 1.0))
        ROS_WARN("HMOTION: Param r_mgn_a not found, set to 1.0");
    if (!nh.param("r_bar_z", r_bar_z, 1.0))
        ROS_WARN("HMOTION: Param r_bar_z not found, set to 1.0");
    if (!nh.param("r_snr_z", r_snr_z, 1.0))
        ROS_WARN("HMOTION: Param r_snr_z not found, set to 1.0");
    if (!nh.param("qa", qa, 1.0))
        ROS_WARN("HMOTION: Param qa not found, set to 1.0");
    if (!nh.param("qx", qx, 1.0))
        ROS_WARN("HMOTION: Param qx not found, set to 1.0");
    if (!nh.param("qy", qy, 1.0))
        ROS_WARN("HMOTION: Param qy not found, set to 1.0");
    if (!nh.param("qz", qz, 1.0))
        ROS_WARN("HMOTION: Param qz not found, set to 1.0");
    if (!nh.param("enable_baro", enable_baro, true))
        ROS_WARN("HMOTION: Param enable_baro not found, set to true");
    if (!nh.param("enable_magnet", enable_magnet, true))
        ROS_WARN("HMOTION: Param enable_magnet not found, set to true");
    if (!nh.param("enable_sonar", enable_sonar, true))
        ROS_WARN("HMOTION: Param enable_sonar not found, set to true");
    if (!nh.param("enable_gps", enable_gps, true))
        ROS_WARN("HMOTION: Param enable_gps not found, set to true");

    // --------- Subscribers ----------
    ros::Subscriber sub_true = nh.subscribe<nav_msgs::Odometry>("ground_truth/state", 1, &cbTrue);
    ros::Subscriber sub_imu = nh.subscribe<sensor_msgs::Imu>("raw_imu", 1, &cbImu);
    ros::Subscriber sub_gps = nh.subscribe<sensor_msgs::NavSatFix>("fix", 1, &cbGps);
    if (!enable_gps)
        sub_gps.shutdown();
    ros::Subscriber sub_magnet = nh.subscribe<geometry_msgs::Vector3Stamped>("magnetic", 1, &cbMagnet);
    if (!enable_magnet)
        sub_magnet.shutdown();
    ros::Subscriber sub_baro = nh.subscribe<hector_uav_msgs::Altimeter>("altimeter", 1, &cbBaro);
    if (!enable_baro)
        sub_baro.shutdown();
    ros::Subscriber sub_sonar = nh.subscribe<sensor_msgs::Range>("sonar_height", 1, &cbSonar);
    if (!enable_sonar)
        sub_sonar.shutdown();

    // --------- Publishers ----------
    ros::Publisher pub_pose = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("pose", 1, true);
    geometry_msgs::PoseWithCovarianceStamped msg_pose;
    msg_pose.header.frame_id = "world";   // for rviz
    msg_pose.pose.pose.orientation.x = 0; // no roll
    msg_pose.pose.pose.orientation.y = 0; // no pitch
    ros::Publisher pub_vel = nh.advertise<geometry_msgs::Twist>("velocity", 1, true); // publish velocity
    geometry_msgs::Twist msg_vel;

    // --------- Wait for Topics ----------
    ROS_INFO("HMOTION: Waiting for topics");
    while (ros::ok() && nh.param("run", true) && ((std::isnan(ux) && msg_true.header.seq == 0))) // wait for imu and truth only
        ros::spinOnce(); // update subscribers

    if (!ros::ok())
    { // ROS shutdown
        ROS_INFO("HMOTION: ===== END =====");
        return 0;
    }

    // --------- Calibrate Gyro service ----------
    ROS_INFO("HMOTION: Calibrating Gyro...");
    ros::ServiceClient calibrate_gyro = nh.serviceClient<std_srvs::Empty>("raw_imu/calibrate");
    std_srvs::Empty calibrate_gyro_srv;
    if (calibrate_gyro.call(calibrate_gyro_srv))
        ROS_INFO("HMOTION: Calibrated Gyro");
    else
        ROS_WARN("HMOTION: Gyro cannot be calibrated!");

    // --------- Main loop ----------

    ros::Rate rate(motion_iter_rate);
    ROS_INFO("HMOTION: ===== BEGIN =====");
    ready = true;
    while (ros::ok() && nh.param("run", true))
    {
        ros::spinOnce(); // update topics

        // Verbose
        if (verbose)
        {
            auto & tp = msg_true.pose.pose.position;
            auto &q = msg_true.pose.pose.orientation;
            double siny_cosp = 2 * (q.w * q.z + q.x * q.y);
            double cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
            ROS_INFO("[HM] ---------X-------Y-------Z-------A------");
            ROS_INFO("[HM]  TRUE(%7.3lf,%7.3lf,%7.3lf,%6.3lf)", tp.x, tp.y, tp.z, atan2(siny_cosp, cosy_cosp));
            ROS_INFO("[HM] STATE(%7.3lf,%7.3lf,%7.3lf,%6.3lf)", X(0), Y(0), Z(0), A(0));
            ROS_INFO("[HM]   GPS(%7.3lf,%7.3lf,%7.3lf, ---- )", GPS(0), GPS(1), GPS(2));
            ROS_INFO("[HM] MAGNT( ----- , ----- , ----- ,%6.3lf)", a_mgn);
            ROS_INFO("[HM]  BARO( ----- , ----- ,%7.3lf, ---- )", z_bar);
            ROS_INFO("[HM] BAROB( ----- , ----- ,%7.3lf, ---- )", Z(3));
            ROS_INFO("[HM] SONAR( ----- , ----- ,%7.3lf, ---- )", z_snr);
        }

        //  Publish pose and vel
        if (use_ground_truth)
        {
            msg_pose.header.stamp = ros::Time::now();
            msg_pose.pose.pose.position = msg_true.pose.pose.position;
            msg_pose.pose.pose.orientation = msg_true.pose.pose.orientation;
            msg_vel = msg_true.twist.twist;
        }
        else
        {
            msg_pose.header.stamp = ros::Time::now();
            msg_pose.pose.pose.position.x = X(0);
            msg_pose.pose.pose.position.y = Y(0);
            msg_pose.pose.pose.position.z = Z(0);
            msg_pose.pose.covariance[0] = P_x(0, 0);  // x cov
            msg_pose.pose.covariance[7] = P_y(0, 0);  // y cov
            msg_pose.pose.covariance[14] = P_z(0, 0); // z cov
            msg_pose.pose.covariance[35] = P_a(0, 0); // a cov
            msg_pose.pose.pose.orientation.w = cos(A(0) / 2);
            msg_pose.pose.pose.orientation.z = sin(A(0) / 2);
            msg_vel.linear.x = X(1);
            msg_vel.linear.y = Y(1);
            msg_vel.linear.z = Z(1);
            msg_vel.angular.z = A(1);
        }
        pub_pose.publish(msg_pose);
        pub_vel.publish(msg_vel);

        rate.sleep();
    }

    ROS_INFO("HMOTION: ===== END =====");
    return 0;
}