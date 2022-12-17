#pragma once

#define TICK_LOG    0

#include <sensor_msgs/Imu.h>
extern sensor_msgs::Imu imu_;
extern int imu_callbacked;
void imuCallBack(const sensor_msgs::Imu imu);

#include <sensor_msgs/BatteryState.h>
extern sensor_msgs::BatteryState battery_;
void batteryCallBack(const sensor_msgs::BatteryState& battery);

#include <sensor_msgs/LaserScan.h>
void lidar2dFrCallBack(const sensor_msgs::LaserScan& scan);
void lidar2dBlCallBack(const sensor_msgs::LaserScan& scan);

#include <sensor_msgs/Range.h>
extern sensor_msgs::Range range_rf_;
extern int range_rf_callbacked;
void rangeRfCallBack(const sensor_msgs::Range& range);

extern sensor_msgs::Range range_rb_;
extern int range_rb_callbacked;
void rangeRbCallBack(const sensor_msgs::Range& range);

extern sensor_msgs::Range range_fr_;
extern int range_fr_callbacked;
void rangeFrCallBack(const sensor_msgs::Range& range);

extern sensor_msgs::Range range_br_;
extern int range_br_callbacked;
void rangeBrCallBack(const sensor_msgs::Range& range);

#include <nav_msgs/Odometry.h>
extern nav_msgs::Odometry odom_;
extern int odom_callbacked;
void odomCallBack(const nav_msgs::Odometry odom);

#include <nav_msgs/Odometry.h>
extern nav_msgs::Odometry lodom_;
extern int lodom_callbacked;
void lodomCallBack(const nav_msgs::Odometry lodom);

#include <geometry_msgs/Twist.h>
// #include <mecanum/WrapTwist.h>
extern geometry_msgs::Twist vel_;
void velCallBack(const geometry_msgs::Twist& vel);

#include <geometry_msgs/Pose2D.h>
extern geometry_msgs::Pose2D pose2d_;
void pose2dCallBack(const geometry_msgs::Pose2D pose2d);
extern unsigned int pose2d_flag;

#include <chatterbox/ChatIn.h>
#include <chatterbox/ChatOut.h>
#include <chatterbox/WhisperOut.h>
extern chatterbox::ChatIn chatIn_;
extern chatterbox::ChatOut chatOut;
extern chatterbox::WhisperOut whisperOut;
extern unsigned int chatIn_flag;
void chatRemoteCallBack(const chatterbox::ChatIn chatIn);
void chatLocalCallBack(const chatterbox::ChatIn chatIn);
void chatWebCallBack(const chatterbox::ChatIn chatIn);
void chatJoystickCallBack(const chatterbox::ChatIn chatIn);

/*
#include <mecanum/Info.h>
extern mecanum::Info mecanum_info_;
void mecanumInfoCallBack(const mecanum::Info mecanum_info);
*/

/*
#include <arduino_safety/Safety.h>
extern arduino_safety::Safety safety_;
void safetyCallBack(const arduino_safety::Safety safety);
*/

#include <geometry_msgs/PoseWithCovarianceStamped.h>
extern geometry_msgs::PoseWithCovarianceStamped pose_;
extern int pose_callbacked;
void poseCallBack(const geometry_msgs::PoseWithCovarianceStamped pose);

#ifdef GAIN_TUNING
#include <dynamic_reconfigure/server.h>
#include <recipelab/GainTuningConfig.h>
void dynConfCallback(const recipelab::GainTuningConfig config, uint32_t level);
#endif