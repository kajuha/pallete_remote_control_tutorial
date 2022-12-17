#include "AGVCallBack.h"
#include "main.h"

sensor_msgs::Imu imu_;
int imu_callbacked;
void imuCallBack(const sensor_msgs::Imu imu) {
    // static double tsnow, tscall, tsdiff;
    // tsnow = ros::Time::now().toSec();
    // tscall = imu.header.stamp.toSec();
    // tsdiff = tsnow - tscall;
    // queueString("[node]: %15s, [msg]: %15s, [tsnow]:%lf, [tscall]:%lf, [tsdiff]:%lf\n", "mw_ahrsv1", "/imu", tsnow, tscall, tsdiff);
    imu_ = imu;
    imu_callbacked = 0xFF;
    # if 0
#define STEP_TIME 1
    static double time_cur = ros::Time::now().toSec();
    static double time_pre = time_cur;

    time_cur = ros::Time::now().toSec();
    if (time_cur - time_pre > STEP_TIME) {
        time_pre = time_cur;
    }
    #endif

    #if TICK_LOG
    static double time_cur = ros::Time::now().toSec();
    static double time_pre = time_cur;
    static uint32_t count = 0;

    time_cur = ros::Time::now().toSec();

    printf("%s count: %05d, time_diff(ms): %lf, ts: %lf\n", __FUNCTION__, count++, time_cur-time_pre, time_cur);
    time_pre = time_cur;
    #endif
}

sensor_msgs::BatteryState battery_;
void batteryCallBack(const sensor_msgs::BatteryState& battery) {
    // static double tsnow, tscall, tsdiff;
    // tsnow = ros::Time::now().toSec();
    // tscall = battery.header.stamp.toSec();
    // tsdiff = tsnow - tscall;
    // queueString("[node]: %15s, [msg]: %15s, [tsnow]:%lf, [tscall]:%lf, [tsdiff]:%lf\n", "mt4n", "battery", tsnow, tscall, tsdiff);
    battery_ = battery;
    # if 0
#define STEP_TIME 1
    static double time_cur = ros::Time::now().toSec();
    static double time_pre = time_cur;

    time_cur = ros::Time::now().toSec();
    if (time_cur - time_pre > STEP_TIME) {
        time_pre = time_cur;
    }
    #endif

    #if TICK_LOG
    static double time_cur = ros::Time::now().toSec();
    static double time_pre = time_cur;
    static uint32_t count = 0;

    time_cur = ros::Time::now().toSec();

    printf("%s count: %05d, time_diff(ms): %lf, ts: %lf\n", __FUNCTION__, count++, time_cur-time_pre, time_cur);
    time_pre = time_cur;
    #endif
}

void lidar2dFrCallBack(const sensor_msgs::LaserScan& scan) {
    // static double tsnow, tscall, tsdiff;
    // tsnow = ros::Time::now().toSec();
    // tscall = scan.header.stamp.toSec();
    // tsdiff = tsnow - tscall;
    // queueString("[node]: %15s, [msg]: %15s, [tsnow]:%lf, [tscall]:%lf, [tsdiff]:%lf\n", "rplidarNode_fr", "scan", tsnow, tscall, tsdiff);
    # if 0
#define STEP_TIME 1
    static double time_cur = ros::Time::now().toSec();
    static double time_pre = time_cur;

    time_cur = ros::Time::now().toSec();
    if (time_cur - time_pre > STEP_TIME) {
        time_pre = time_cur;
    }
    #endif

    #if TICK_LOG
    static double time_cur = ros::Time::now().toSec();
    static double time_pre = time_cur;
    static uint32_t count = 0;

    time_cur = ros::Time::now().toSec();

    printf("%s count: %05d, time_diff(ms): %lf, ts: %lf\n", __FUNCTION__, count++, time_cur-time_pre, time_cur);
    time_pre = time_cur;
    #endif
}

void lidar2dBlCallBack(const sensor_msgs::LaserScan& scan) {
    // static double tsnow, tscall, tsdiff;
    // tsnow = ros::Time::now().toSec();
    // tscall = scan.header.stamp.toSec();
    // tsdiff = tsnow - tscall;
    // queueString("[node]: %15s, [msg]: %15s, [tsnow]:%lf, [tscall]:%lf, [tsdiff]:%lf\n", "rplidarNode_bl", "scan", tsnow, tscall, tsdiff);
    # if 0
#define STEP_TIME 1
    static double time_cur = ros::Time::now().toSec();
    static double time_pre = time_cur;

    time_cur = ros::Time::now().toSec();
    if (time_cur - time_pre > STEP_TIME) {
        time_pre = time_cur;
    }
    #endif

    #if TICK_LOG
    static double time_cur = ros::Time::now().toSec();
    static double time_pre = time_cur;
    static uint32_t count = 0;

    time_cur = ros::Time::now().toSec();

    printf("%s count: %05d, time_diff(ms): %lf, ts: %lf\n", __FUNCTION__, count++, time_cur-time_pre, time_cur);
    time_pre = time_cur;
    #endif
}

sensor_msgs::Range range_rf_;
int range_rf_callbacked = 0;
void rangeRfCallBack(const sensor_msgs::Range& range) {
    // static double tsnow, tscall, tsdiff;
    // tsnow = ros::Time::now().toSec();
    // tscall = range.header.stamp.toSec();
    // tsdiff = tsnow - tscall;
    // queueString("[node]: %15s, [msg]: %15s, [tsnow]:%lf, [tscall]:%lf, [tsdiff]:%lf\n", "tf40_rf", "tf40_rf", tsnow, tscall, tsdiff);
    range_rf_.range = range.range;
    range_rf_callbacked = 1;
    # if 0
#define STEP_TIME 1
    static double time_cur = ros::Time::now().toSec();
    static double time_pre = time_cur;

    time_cur = ros::Time::now().toSec();
    if (time_cur - time_pre > STEP_TIME) {
        time_pre = time_cur;
    }
    #endif

    #if TICK_LOG
    static double time_cur = ros::Time::now().toSec();
    static double time_pre = time_cur;
    static uint32_t count = 0;

    time_cur = ros::Time::now().toSec();

    printf("%s count: %05d, time_diff(ms): %lf, ts: %lf\n", __FUNCTION__, count++, time_cur-time_pre, time_cur);
    time_pre = time_cur;
    #endif
}

sensor_msgs::Range range_rb_;
int range_rb_callbacked = 0;
void rangeRbCallBack(const sensor_msgs::Range& range) {
    // static double tsnow, tscall, tsdiff;
    // tsnow = ros::Time::now().toSec();
    // tscall = range.header.stamp.toSec();
    // tsdiff = tsnow - tscall;
    // queueString("[node]: %15s, [msg]: %15s, [tsnow]:%lf, [tscall]:%lf, [tsdiff]:%lf\n", "tf40_rb", "tf40_rb", tsnow, tscall, tsdiff);
    range_rb_.range = range.range;
    range_rb_callbacked = 1;
    # if 0
#define STEP_TIME 1
    static double time_cur = ros::Time::now().toSec();
    static double time_pre = time_cur;

    time_cur = ros::Time::now().toSec();
    if (time_cur - time_pre > STEP_TIME) {
        time_pre = time_cur;
    }
    #endif

    #if TICK_LOG
    static double time_cur = ros::Time::now().toSec();
    static double time_pre = time_cur;
    static uint32_t count = 0;

    time_cur = ros::Time::now().toSec();

    printf("%s count: %05d, time_diff(ms): %lf, ts: %lf\n", __FUNCTION__, count++, time_cur-time_pre, time_cur);
    time_pre = time_cur;
    #endif
}

sensor_msgs::Range range_fr_;
int range_fr_callbacked = 0;
void rangeFrCallBack(const sensor_msgs::Range& range) {
    // static double tsnow, tscall, tsdiff;
    // tsnow = ros::Time::now().toSec();
    // tscall = range.header.stamp.toSec();
    // tsdiff = tsnow - tscall;
    // queueString("[node]: %15s, [msg]: %15s, [tsnow]:%lf, [tscall]:%lf, [tsdiff]:%lf\n", "tf02_fr", "TF02_fr", tsnow, tscall, tsdiff);
    range_fr_.range = range.range;
    range_fr_callbacked = 1;
    # if 0
#define STEP_TIME 1
    static double time_cur = ros::Time::now().toSec();
    static double time_pre = time_cur;

    time_cur = ros::Time::now().toSec();
    if (time_cur - time_pre > STEP_TIME) {
        time_pre = time_cur;
    }
    #endif

    #if TICK_LOG
    static double time_cur = ros::Time::now().toSec();
    static double time_pre = time_cur;
    static uint32_t count = 0;

    time_cur = ros::Time::now().toSec();

    printf("%s count: %05d, time_diff(ms): %lf, ts: %lf\n", __FUNCTION__, count++, time_cur-time_pre, time_cur);
    time_pre = time_cur;
    #endif
}

sensor_msgs::Range range_br_;
int range_br_callbacked = 0;
void rangeBrCallBack(const sensor_msgs::Range& range) {
    // static double tsnow, tscall, tsdiff;
    // tsnow = ros::Time::now().toSec();
    // tscall = range.header.stamp.toSec();
    // tsdiff = tsnow - tscall;
    // queueString("[node]: %15s, [msg]: %15s, [tsnow]:%lf, [tscall]:%lf, [tsdiff]:%lf\n", "tf02_br", "TF02_br", tsnow, tscall, tsdiff);
    range_br_.range = range.range;
    range_br_callbacked = 1;
    # if 0
#define STEP_TIME 1
    static double time_cur = ros::Time::now().toSec();
    static double time_pre = time_cur;

    time_cur = ros::Time::now().toSec();
    if (time_cur - time_pre > STEP_TIME) {
        time_pre = time_cur;
    }
    #endif

    #if TICK_LOG
    static double time_cur = ros::Time::now().toSec();
    static double time_pre = time_cur;
    static uint32_t count = 0;

    time_cur = ros::Time::now().toSec();

    printf("%s count: %05d, time_diff(ms): %lf, ts: %lf\n", __FUNCTION__, count++, time_cur-time_pre, time_cur);
    time_pre = time_cur;
    #endif
}

nav_msgs::Odometry odom_;
int odom_callbacked = 0;
void odomCallBack(const nav_msgs::Odometry odom) {
    // static double tsnow, tscall, tsdiff;
    // tsnow = ros::Time::now().toSec();
    // tscall = odom.header.stamp.toSec();
    // tsdiff = tsnow - tscall;
    // queueString("[node]: %15s, [msg]: %15s, [tsnow]:%lf, [tscall]:%lf, [tsdiff]:%lf\n", "mecanum", "odom", tsnow, tscall, tsdiff);
    odom_ = odom;
    odom_callbacked = 1;
    # if 0
#define STEP_TIME 1
    static double time_cur = ros::Time::now().toSec();
    static double time_pre = time_cur;

    time_cur = ros::Time::now().toSec();
    if (time_cur - time_pre > STEP_TIME) {
        time_pre = time_cur;
    }
    #endif

    #if TICK_LOG
    static double time_cur = ros::Time::now().toSec();
    static double time_pre = time_cur;
    static uint32_t count = 0;

    time_cur = ros::Time::now().toSec();

    printf("%s count: %05d, time_diff(ms): %lf, ts: %lf\n", __FUNCTION__, count++, time_cur-time_pre, time_cur);
    time_pre = time_cur;
    #endif
}

nav_msgs::Odometry lodom_;
int lodom_callbacked = 0;
void lodomCallBack(const nav_msgs::Odometry lodom) {
    // static double tsnow, tscall, tsdiff;
    // tsnow = ros::Time::now().toSec();
    // tscall = lodom.header.stamp.toSec();
    // tsdiff = tsnow - tscall;
    // queueString("[node]: %15s, [msg]: %15s, [tsnow]:%lf, [tscall]:%lf, [tsdiff]:%lf\n", "mecanum", "lodom", tsnow, tscall, tsdiff);
    lodom_ = lodom;
    lodom_callbacked = 1;
    # if 0
#define STEP_TIME 1
    static double time_cur = ros::Time::now().toSec();
    static double time_pre = time_cur;

    time_cur = ros::Time::now().toSec();
    if (time_cur - time_pre > STEP_TIME) {
        time_pre = time_cur;
    }
    #endif

    #if TICK_LOG
    static double time_cur = ros::Time::now().toSec();
    static double time_pre = time_cur;
    static uint32_t count = 0;

    time_cur = ros::Time::now().toSec();

    printf("%s count: %05d, time_diff(ms): %lf, ts: %lf\n", __FUNCTION__, count++, time_cur-time_pre, time_cur);
    time_pre = time_cur;
    #endif
}

geometry_msgs::Twist vel_;
void velCallBack(const geometry_msgs::Twist& vel) {
    # if 0
#define STEP_TIME 1
    static double time_cur = ros::Time::now().toSec();
    static double time_pre = time_cur;

    time_cur = ros::Time::now().toSec();
    if (time_cur - time_pre > STEP_TIME) {
        time_pre = time_cur;
    }
    #endif

    #if TICK_LOG
    static double time_cur = ros::Time::now().toSec();
    static double time_pre = time_cur;
    static uint32_t count = 0;

    time_cur = ros::Time::now().toSec();

    printf("%s count: %05d, time_diff(ms): %lf, ts: %lf\n", __FUNCTION__, count++, time_cur-time_pre, time_cur);
    time_pre = time_cur;
    #endif
}

geometry_msgs::Pose2D pose2d_;
unsigned int pose2d_flag;
void pose2dCallBack(const geometry_msgs::Pose2D pose2d) {
    pose2d_ = pose2d;

    pose2d_flag = 0xFF;
    # if 0
#define STEP_TIME 1
    static double time_cur = ros::Time::now().toSec();
    static double time_pre = time_cur;

    time_cur = ros::Time::now().toSec();
    if (time_cur - time_pre > STEP_TIME) {
        time_pre = time_cur;
    }
    #endif

    #if TICK_LOG
    static double time_cur = ros::Time::now().toSec();
    static double time_pre = time_cur;
    static uint32_t count = 0;

    time_cur = ros::Time::now().toSec();

    printf("%s count: %05d, time_diff(ms): %lf, ts: %lf\n", __FUNCTION__, count++, time_cur-time_pre, time_cur);
    time_pre = time_cur;
    #endif
}

#include <chatterbox/ChatIn.h>
chatterbox::ChatIn chatIn_;
chatterbox::ChatOut chatOut;
chatterbox::WhisperOut whisperOut;
unsigned int chatIn_flag;
void chatRemoteCallBack(const chatterbox::ChatIn chatIn) {
    chatIn_ = chatIn;
    chatIn_flag = 0xFF;
    # if 0
#define STEP_TIME 1
    static double time_cur = ros::Time::now().toSec();
    static double time_pre = time_cur;

    time_cur = ros::Time::now().toSec();
    if (time_cur - time_pre > STEP_TIME) {
        time_pre = time_cur;
    }
    #endif

    #if TICK_LOG
    static double time_cur = ros::Time::now().toSec();
    static double time_pre = time_cur;
    static uint32_t count = 0;

    time_cur = ros::Time::now().toSec();

    printf("%s count: %05d, time_diff(ms): %lf, ts: %lf\n", __FUNCTION__, count++, time_cur-time_pre, time_cur);
    time_pre = time_cur;
    #endif
}

void chatLocalCallBack(const chatterbox::ChatIn chatIn) {
    chatIn_ = chatIn;
    chatIn_flag = 0xFF;
    # if 0
#define STEP_TIME 1
    static double time_cur = ros::Time::now().toSec();
    static double time_pre = time_cur;

    time_cur = ros::Time::now().toSec();
    if (time_cur - time_pre > STEP_TIME) {
        time_pre = time_cur;
    }
    #endif

    #if TICK_LOG
    static double time_cur = ros::Time::now().toSec();
    static double time_pre = time_cur;
    static uint32_t count = 0;

    time_cur = ros::Time::now().toSec();

    printf("%s count: %05d, time_diff(ms): %lf, ts: %lf\n", __FUNCTION__, count++, time_cur-time_pre, time_cur);
    time_pre = time_cur;
    #endif
}

void chatWebCallBack(const chatterbox::ChatIn chatIn) {
    chatIn_ = chatIn;
    chatIn_flag = 0xFF;
    # if 0
#define STEP_TIME 1
    static double time_cur = ros::Time::now().toSec();
    static double time_pre = time_cur;

    time_cur = ros::Time::now().toSec();
    if (time_cur - time_pre > STEP_TIME) {
        time_pre = time_cur;
    }
    #endif

    #if TICK_LOG
    static double time_cur = ros::Time::now().toSec();
    static double time_pre = time_cur;
    static uint32_t count = 0;

    time_cur = ros::Time::now().toSec();

    printf("%s count: %05d, time_diff(ms): %lf, ts: %lf\n", __FUNCTION__, count++, time_cur-time_pre, time_cur);
    time_pre = time_cur;
    #endif
}

void chatJoystickCallBack(const chatterbox::ChatIn chatIn) {
    chatIn_ = chatIn;
    chatIn_flag = 0xFF;
    # if 0
#define STEP_TIME 1
    static double time_cur = ros::Time::now().toSec();
    static double time_pre = time_cur;

    time_cur = ros::Time::now().toSec();
    if (time_cur - time_pre > STEP_TIME) {
        time_pre = time_cur;
    }
    #endif

    #if TICK_LOG
    static double time_cur = ros::Time::now().toSec();
    static double time_pre = time_cur;
    static uint32_t count = 0;

    time_cur = ros::Time::now().toSec();

    printf("%s count: %05d, time_diff(ms): %lf, ts: %lf\n", __FUNCTION__, count++, time_cur-time_pre, time_cur);
    time_pre = time_cur;
    #endif
}

/*
#include <mecanum/Info.h>
mecanum::Info mecanum_info_;
void mecanumInfoCallBack(const mecanum::Info mecanum_info) {
    mecanum_info_ = mecanum_info;
    # if 0
#define STEP_TIME 1
    static double time_cur = ros::Time::now().toSec();
    static double time_pre = time_cur;

    time_cur = ros::Time::now().toSec();
    if (time_cur - time_pre > STEP_TIME) {
        time_pre = time_cur;
    }
    #endif

    #if TICK_LOG
    static double time_cur = ros::Time::now().toSec();
    static double time_pre = time_cur;
    static uint32_t count = 0;

    time_cur = ros::Time::now().toSec();

    printf("%s count: %05d, time_diff(ms): %lf, ts: %lf\n", __FUNCTION__, count++, time_cur-time_pre, time_cur);
    time_pre = time_cur;
    #endif
}
*/

/*
#include <arduino_safety/Safety.h>
arduino_safety::Safety safety_;
void safetyCallBack(const arduino_safety::Safety safety) {
    safety_ = safety;
    # if 0
#define STEP_TIME 1
    static double time_cur = ros::Time::now().toSec();
    static double time_pre = time_cur;

    time_cur = ros::Time::now().toSec();
    if (time_cur - time_pre > STEP_TIME) {
        time_pre = time_cur;
    }
    #endif

    #if TICK_LOG
    static double time_cur = ros::Time::now().toSec();
    static double time_pre = time_cur;
    static uint32_t count = 0;

    time_cur = ros::Time::now().toSec();

    printf("%s count: %05d, time_diff(ms): %lf, ts: %lf\n", __FUNCTION__, count++, time_cur-time_pre, time_cur);
    time_pre = time_cur;
    #endif
}
*/

#include <geometry_msgs/PoseWithCovarianceStamped.h>
geometry_msgs::PoseWithCovarianceStamped pose_;
int pose_callbacked;
void poseCallBack(const geometry_msgs::PoseWithCovarianceStamped pose) {
    // static double tsnow, tscall, tsdiff;
    // tsnow = ros::Time::now().toSec();
    // tscall = pose.header.stamp.toSec();
    // tsdiff = tsnow - tscall;
    // queueString("[node]: %15s, [msg]: %15s, [tsnow]:%lf, [tscall]:%lf, [tsdiff]:%lf\n", "amcl", "pose", tsnow, tscall, tsdiff);
    pose_ = pose;
    pose_callbacked = 1;
    # if 0
#define STEP_TIME 1
    static double time_cur = ros::Time::now().toSec();
    static double time_pre = time_cur;

    time_cur = ros::Time::now().toSec();
    if (time_cur - time_pre > STEP_TIME) {
        time_pre = time_cur;
    }
    #endif

    #if TICK_LOG
    static double time_cur = ros::Time::now().toSec();
    static double time_pre = time_cur;
    static uint32_t count = 0;

    time_cur = ros::Time::now().toSec();

    printf("%s count: %05d, time_diff(ms): %lf, ts: %lf\n", __FUNCTION__, count++, time_cur-time_pre, time_cur);
    time_pre = time_cur;
    #endif
}

#ifdef GAIN_TUNING
#include <dynamic_reconfigure/server.h>
#include <recipelab/GainTuningConfig.h>
void dynConfCallback(const recipelab::GainTuningConfig config, uint32_t level) {
    LBox.control.pid.x.Kp = config.x_Kp;
    LBox.control.pid.x.Ki = config.x_Ki;
    LBox.control.pid.x.Kd = config.x_Kd;
    LBox.control.pid.x.Kc = config.x_Kc;

    LBox.control.pid.y.Kp = config.y_Kp;
    LBox.control.pid.y.Ki = config.y_Ki;
    LBox.control.pid.y.Kd = config.y_Kd;
    LBox.control.pid.y.Kc = config.y_Kc;

    LBox.control.pid.rz.Kp = config.rz_Kp;
    LBox.control.pid.rz.Ki = config.rz_Ki;
    LBox.control.pid.rz.Kd = config.rz_Kd;
    LBox.control.pid.rz.Kc = config.rz_Kc;
    
    ROS_INFO("Reconfigure request : %lf %lf %lf %lf | %lf %lf %lf %lf | %lf %lf %lf %lf",        
        config.x_Kp,
        config.x_Ki,
        config.x_Kd,
        config.x_Kc,

        config.y_Kp,
        config.y_Ki,
        config.y_Kd,
        config.y_Kc,

        config.rz_Kp,
        config.rz_Ki,
        config.rz_Kd,
        config.rz_Kc
    );
}
#endif