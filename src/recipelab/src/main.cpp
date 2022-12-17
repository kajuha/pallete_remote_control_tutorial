#include <stdio.h>
#include <ros/ros.h>
#include <queue>
#include <boost/thread.hpp>
#include <nav_msgs/Path.h>

// #include <mecanum/MotorPosition.h>

#ifdef GAIN_TUNING
#include <dynamic_reconfigure/server.h>
#include <recipelab/GainTuningConfig.h>
#endif

using namespace std;

#include "AGVCallBack.h"
#include "AGVConfig.h"
#include "main.h"

AGVModels LBox;

PROFILE_JOINT AGVPx;
PROFILE_JOINT AGVPy;
PROFILE_JOINT AGVYaw;

chatterbox::ChatIn RcmAgvDataRX;           // RCM(상위제어기)에서 AGV제어기로 주는 데이터를 저장하는 구조체
chatterbox::ChatOut RcmAgvDataTX;           // AGV제어기에서 RCM(상위제어기)로 보내는 데이터를 저장하는 구조체

void AGVSensorsTopicSubscriberSet(ros::NodeHandle nh);            // AGV 센서 노드 토픽 Subscriber 설정
void AGVSensorsCheckFn(void);                       // AGV 센서 데이터가 들어오는지 확인하는 함수

AGV1AxisSensorBaseOdometry AGVOdometry_TFSensor;    // 1축 Lidar 센서 조합의 AGV Odometry 알고리즘
AGV_ODOMETRY OdomWheelBase;                         // 메카넘휠 베이스 odometry 결과값
AGV_ODOMETRY OdomImuBase;                           // IMU 센서 베이스 odometry 결과값 
AGV_ODOMETRY Odom1AxisLidarBase;                    // 1축 라이다 센서 베이스 odometry 결과값
AGV_ODOMETRY OdomKalmanBase;                        // 칼만필터 베이스 odometry 결과값
AGV_ODOMETRY OdomAmclBase;                         // RPLIDAR AMCL 베이스 odometry 결과값
AGVKamanFilter OdomKalmanFilter;                    // 칼만필터를 통해 메카넘휠 베이스 odometry 기준으로 1축 라이다 센서 베이스 odometry 측정값을 활용하여 출력
double AGVFrontRange;
double AGVRearRange;
double AGVRigntFrontRange;
double AGVRigntRearRange;

void AGVOdomWheelbaseGet(double x, double y, double qx, double qy, double qz, double w, AGV_ODOMETRY *fodom);   // MD1K 제어기에서 들어오는 메카넘휠 기구학 기반 odometry 정보 수신함수
void AGVOdomImubaseGet(double x, double y, double z, double w,AGV_ODOMETRY *fodom);                             // IMU센서 기반 odometry 정보 수신함수
void AGVOdom1AxisLidarbaseGet(AGV_ODOMETRY *fOdom1AxisLidarBase);                                               // 1축 라이다 센서 기반 odometry 정보 수신함수
void AGVOdomAmclbaseGet(double x, double y, double qx, double qy, double qz, double w, AGV_ODOMETRY *fodom);   // 2축 RPLIDAR 두개를 접목한 AMCL 알고리즘을 통해 나온 라이다 기반 오도메트리 정보 수신함수
void AGVOdomPoseInit(void);

void ProfileValInit(void);		                // 경로계획 관련 변수 초기화 함수
void AGVVariableInit(void);		                    // AGV 시스템 관련 변수 초기화 함수                            
#if GAIN_TUNING
void AGVPIDValInit(dynamic_reconfigure::Server<recipelab::GainTuningConfig>* srvGainTuning);
#else
void AGVPIDValInit(void);                           // AGV PID 관련 변수 초기화 함수
#endif
void AGVOdomValInit(void);                          // AGV Odom 관련 변수 초기화 함수
void AGVPIDValReset(void);                          // AGV PID 관련 변수 Reset 함수
void AGVTimeValInit(void);                          // 타임관련 변수 초기화 함수

// AGV 시스템 관련 함수 정의

void WatchDog_Battery_VDC_UVP(AGVModels *fAGV);           // 배터리 전압이 23.6V 이하 under voltage 상태일 경우(AGVModels *fAGV);           // 배터리 전압이 23.6V 이하 under voltage 상태일 경우

void WatchDog_Positon_ERR(AGVModels *fAGV);           // AGV 위치 피드백값 에러 

void WatchDog_VitualWall_Detect_Front(AGVModels *fAGV);
void WatchDog_VitualWall_Detect_Rear(AGVModels *fAGV);
void WatchDog_VitualWall_Detect_RightF(AGVModels *fAGV);
void WatchDog_VitualWall_Detect_RightR(AGVModels *fAGV);

void WatchDog_Agv_Feedback_Error_x(AGVModels *fAGV);
void WatchDog_Agv_Feedback_Error_y(AGVModels *fAGV);
void WatchDog_Agv_Feedback_Error_rz(AGVModels *fAGV);



// 시스템 순서도 흐름도를 기준으로 묶었다.
void FN_SYSTEM_FLOW_START(void);

void FN_AGV_SYSTEM_OUTPUT_PROCESS1(void);                               // 시스템 출력 함수 -- 모터 제어 출력값 twist 값 출력
void FN_AGV_SYSTEM_INPUT_ODOMETRY_GET(void);                            // 시스템 입력 함수  -- 모터드라이버 값이 주변 센서값 수신 
void FN_AGV_POSITIONTABLEMODE(void);                                    // 시스템 포지션 테이블 구동 함수 -- 임시 함수임.                    
void FN_AGV_SYSTEM_INPUT_RCM_GET(void);                                 // 시스템 입력 함수  -- 상위제어기(RCM) 명령 수신 --> RCM에서 들어오는 명령을 AGV 내부 명령으로 맵핑
void FN_AGV_SYSTEM_INPUT_RCM_PROCESS(void);                             // 시스템 입력 함수 처리 -- RCM 입력값 매핑 및 모드별 상태변수 초기화

void FN_AGV_ODOMETRY_CALCULATE(void);  
void FN_AGV_TRAJECTORY_PROFILE_INIT(geometry_msgs::Twist *twist);       // 시스템 경로 프로파일 초기화 -- 모드별 경로 프로파일 변수 초기화             
void FN_AGV_TRAJECTORY_PROFILE(void);                                   // 시스템 경로 프로파일 처리
void FN_AGV_PID_CONTROL(void);
//void FN_AGV_SYSTEM_CONTROL_OUT(geometry_msgs::Twist *twist);
void FN_AGV_SYSTEM_CONTROL_OUT(void);
void FN_AGV_SYSTEM_WATCHDOG(void);
void FN_AGV_SYSTEM_ERROR_DOACTING(geometry_msgs::Twist *twist);
void FN_AGV_SYSTEM_OUTPUT_PRINTF(geometry_msgs::Twist *twist);                                 // 임시 Printf 출력함수

// timer 설정¡
// 정밀 타이머 
double TimeMain_cur;                               // ros::Time::now().toSec() 을 저장하는 변수
AGV_TIMETEMP TimeCheckMainLoop;                    // while문 내 메인 시간 체크
AGV_TIMETEMP TimeCheckPrintfLoop;                  // printf 출력 시간 체크
AGV_TIMETEMP TimeCheckControlLoop;                 // 메인 제어 알고리즘 시간 체크(프로파일,PID)
AGV_TIMETEMP TimeCheckOdomWheelget;                // 메카넘휠 odometry 샘플링 타임.
AGV_TIMETEMP TimeCheckOdomKalmanFilter;            // 칼만필터 샘플링 타임

// 정밀하지 않는 타이머
__uint32_t TimeCount_T1;		                // 메인 타이머 카운트 
__uint32_t TimeCount_T2_kalman;		            // 메인 타이머 카운트
__uint32_t TimeCount_T3_PID;		            // PID 위치 제어기 빠져 나오는 시간 카운트(50msec 단위)
__uint32_t TimeCount_W1;		                // WatchDog_Output_VDC_OVP_Emergency 타이머
__uint32_t TimeCount_W2_Front;		            // WatchDog_ *** 타이머
__uint32_t TimeCount_W2_Rear;		            // WatchDog_ *** 타이머
__uint32_t TimeCount_W2_RightF;		            // WatchDog_ *** 타이머
__uint32_t TimeCount_W2_RightR;		            // WatchDog_ *** 타이머

__uint32_t TimeCount_W3;		                // WatchDog_ 타이머


__uint32_t TimeCount_W4;		                // WatchDog_ *** 타이머
__uint32_t TimeCount_W5;		                // WatchDog_ *** 타이머

// 임시 변수 
__uint32_t bufcount11=200;

queue<std::string> que_string;

void queueString(std::string format, std::string node, std::string msg, double tsnow, double tscall, double tsdiff) {
#define CHAR_SIZE   512
    static char ch[CHAR_SIZE] = {'\0', };
    sprintf(ch, format.c_str(), node.c_str(), msg.c_str(), tsnow, tscall, tsdiff);
    static std::string str;
    str = ch;
    que_string.push(str);
}

void fileOperationLoop() {
    ros::NodeHandlePtr node = boost::make_shared<ros::NodeHandle>();

#define CHECK_HZ    1000
    ros::Rate loop_rate(CHECK_HZ);

    int size;
    std::string str;

    while (ros::ok())
    {
        size = que_string.size();

        if (size) {
            // printf("%lf : que_string.size() : %d\n", ros::Time::now().toSec(), size);
            str = que_string.front();
            printf("%s", str.c_str());

            que_string.pop();
        } else {
        }

        loop_rate.sleep();
    }
}

int paramAgvModel;
int paramEnLidarCheck;
int paramEnVirtualWall;
int paramEnGazebo;
string paramRemapCmdVel;

//-----START 임수변수 정의 
// 내부 위치모드 테이블 이용시 필요한 플래그임.
unsigned int InnerCmd_flag=OFF;
unsigned int InnerCmdCount=0;
unsigned int InnerCmdstate=0;
unsigned int InnerCmdInput_flag=0;
chatterbox::ChatIn InnerCmdAgvData;           // 내부데이터 명령으로 내부에서 바로  AGV제어기로 주는 데이터를 저장하는 구조체
//-----END 임수변수 정의

ros::ServiceClient client_motor_position;

int main(int argc, char** argv) {

	ros::init(argc, argv, "recipelab");

    ros::NodeHandle nh("~");

#ifdef GAIN_TUNING
    #if 1
    boost::recursive_mutex config_mutex;
    dynamic_reconfigure::Server<recipelab::GainTuningConfig> srvGainTuning(config_mutex);
    #else
    dynamic_reconfigure::Server<recipelab::GainTuningConfig> srvGainTuning;
    #endif
    dynamic_reconfigure::Server<recipelab::GainTuningConfig>::CallbackType callbackGainTuning;
    callbackGainTuning = boost::bind(&dynConfCallback, _1, _2);
    srvGainTuning.setCallback(callbackGainTuning);
#endif

    // 파라미터 초기화
    #if 1
    ros::param::get("~AGV_MODEL", paramAgvModel);
    ros::param::get("~en_lidar_check", paramEnLidarCheck);
    ros::param::get("~en_virtual_wall", paramEnVirtualWall);
    ros::param::get("~en_gazebo", paramEnGazebo);
    ros::param::get("~remap_cmd_vel", paramRemapCmdVel);
    #else
    nh.getParam("AGV_MODEL", paramAgvModel);
    nh.getParam("en_lidar_check", paramEnLidarCheck);
    nh.getParam("en_virtual_wall", paramEnVirtualWall);
    nh.getParam("en_gazebo", paramEnGazebo);
    nh.getParam("remap_cmd_vel", paramRemapCmdVel);
    #endif
    // AGV 타입에 따른 1축 라이다 센서 위치값 설정 , 현재는 모든 AGV에 적용되도록 코드가 적용되어 if문을 주석처리함. 나중에 수정
    // if (paramAgvModel != PALLETE){
        AGVOdometry_TFSensor = AGV1AxisSensorBaseOdometry((AGV_MODEL)paramAgvModel);
    // }
    //--ROS AGV 토픽 설정 start-------------------------------------------------------------------------------------------------------
    #if 0
    AGVSensorsTopicSubscriberSet(_nh);         // AGV 센서 노드 토픽 설정
    #else
    // imu topic
    ros::Subscriber imu_sub;
    if (paramEnGazebo) {
        imu_sub = nh.subscribe("/imu", 10, imuCallBack);
    } else {
        imu_sub = nh.subscribe("/imu", 10, imuCallBack);
    }
    // battery topic
    #if 0
    ros::Subscriber battery_sub = nh.subscribe("/mt4n/battery", 10, batteryCallBack);
    #else
    ros::Subscriber battery_sub = nh.subscribe("/battery/battery", 10, batteryCallBack);
    #endif
    // lidar fr topic
    ros::Subscriber lidar_fr_sub = nh.subscribe("/rplidarNode_fr/scan", 10, lidar2dFrCallBack);
    // lidar bl topic
    ros::Subscriber lidar_bl_sub = nh.subscribe("/rplidarNode_bl/scan", 10, lidar2dBlCallBack);

    ros::Subscriber range_rf_sub;
    ros::Subscriber range_rb_sub;
    ros::Subscriber range_fr_sub;
    ros::Subscriber range_br_sub;

    if (paramAgvModel == TURTLESHIP) {
        std::cout << "AGV_MODEL(TURTLESHIP)\n";
        // tf40 rf topic
        range_rf_sub = nh.subscribe("/tf40_rf/tf40_rf", 10, rangeRfCallBack);
        // tf40 rb topic
        range_rb_sub = nh.subscribe("/tf40_rb/tf40_rb", 10, rangeRbCallBack);
        // tf02 fr topic
        range_fr_sub = nh.subscribe("/tf02_fr/TF02_fr", 10, rangeFrCallBack);
        // tf02 br topic    
        range_br_sub = nh.subscribe("/tf02_br/TF02_br", 10, rangeBrCallBack);
    } else if (paramAgvModel == LUNCHBOX) {
        std::cout << "AGV_MODEL(LUNCHBOX)\n";
        // tfmini rf topic
        range_rf_sub = nh.subscribe("/tfmini_rf/TFmini_rf", 10, rangeRfCallBack);
        // tfmini rb topic    
        range_rb_sub = nh.subscribe("/tfmini_rb/TFmini_rb", 10, rangeRbCallBack);
        // tfmini fr topic
        range_fr_sub = nh.subscribe("/tfmini_fr/TFmini_fr", 10, rangeFrCallBack);
        // tfmini br topic    
        range_br_sub = nh.subscribe("/tfmini_br/TFmini_br", 10, rangeBrCallBack);
    } else if (paramAgvModel == PALLETE) {
        std::cout << "AGV_MODEL(PALLETE)\n";
    } else {
        std::cout << "AGV_MODEL(UNKNOWN)\n";
        return -1;
    }

    // thread start
    boost::thread threadFileOperationLoop(fileOperationLoop);


    // odom topic
    ros::Subscriber odom_sub = nh.subscribe("/odom", 10, odomCallBack);
    ros::Subscriber lodom_sub;
    if (paramEnGazebo) {
        lodom_sub = nh.subscribe("/odom", 10, lodomCallBack);
    } else {
        // lodom_sub = nh.subscribe("/mecanum/lodom", 10, lodomCallBack);
    }
    // pose topic
    ros::Subscriber pose_sub = nh.subscribe("/amcl_pose", 10, poseCallBack);
    ros::Publisher path_pub = nh.advertise<nav_msgs::Path>("/recipelab/path", 10);
    // vel topic
    // ros::Subscriber vel_sub = nh.subscribe("/cmd_vel", 100, velCallBack);
    // pose2d topic
    ros::Subscriber pose2d_sub = nh.subscribe("/pose2d_sender/pose2d_command", 10, pose2dCallBack);
    // chatterbox remote topic
    ros::Subscriber chat_remote_sub = nh.subscribe("/chatterbox_remote/chatIn_topic", 10, chatRemoteCallBack);
    // chatterbox local topic
    ros::Subscriber chat_local_sub = nh.subscribe("/chatterbox_local/chatIn_topic", 10, chatLocalCallBack);
    // chatterbox web topic
    ros::Subscriber chat_web_sub = nh.subscribe("/chatterbox_web/chatIn_topic", 10, chatWebCallBack);
    // chatterbox joystick topic
    ros::Subscriber chat_joystick_sub = nh.subscribe("/chatterbox_joystick/chatIn_topic", 10, chatJoystickCallBack);
    
    // vel topic
    ros::Publisher vel_pub;
    if (paramEnGazebo) {
        vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    } else {
        // vel_pub = nh.advertise<mecanum::WrapTwist>(paramRemapCmdVel, 10);
    }

    // chatOut topic
    ros::Publisher chat_pub = nh.advertise<chatterbox::ChatOut>("/chatterbox/chatOut_topic", 10);
    // whisperOut topic
    ros::Publisher whisper_pub = nh.advertise<chatterbox::WhisperOut>("/chatterbox/whisperOut_topic", 10);
    
    // mecanum_info topic
    // ros::Subscriber mecanum_info_sub = nh.subscribe("/mecanum/info_topic", 10, mecanumInfoCallBack);
    
    // arduino safety topic
    // ros::Subscriber safety_sub = nh.subscribe("/faduino/safety", 10, safetyCallBack);

    // motor driver encoder service
    // client_motor_position = nh.serviceClient<mecanum::MotorPosition>("/mecanum/motor_position");
    #endif
//--ROS AGV 토픽 설정 END-------------------------------------------------------------------------------------------------------

    geometry_msgs::Twist twist;                 // AGV 출력값 설정(Vector3 linear(float64 x,y,z) , Vector3 angular(float64 x,y,z))
//    geometry_msgs::Twist Temptwist;             // twist 임시 저장 변수 --> 최종 출력값 변수를 먼저 Temptwist에 저장을 하고 
     
    // mecanum::WrapTwist wrapTwist;             // AGV 출력값 설정(Vector3 linear(float64 x,y,z) , Vector3 angular(float64 x,y,z))
	
    if (paramEnLidarCheck) {    AGVSensorsCheckFn();}                    // AGV 센서 데이터가 들어오는지 확인하는 함수

    ProfileValInit();		// 경로계획 관련 변수 초기화
    AGVVariableInit();		    // AGV 시스템 관련 변수 초기화                          
#if GAIN_TUNING
    AGVPIDValInit(&srvGainTuning);
#else
    AGVPIDValInit();            // AGV PID 관련 변수 초기화 함수 
#endif
    AGVOdomValInit();           // AGV Odom 관련 변수 초기화 함수
    // system variable init
    
    ros::Rate maintimer(TIME_main_Hz);   // 메인타이머 설정 100Hz -> 10msec
    ros::Time ts_now = ros::Time::now();

    AGVTimeValInit();
    ////--- START 특정 루프의 타이머를 체크할 때 사용.
    //static double time_cur = ros::Time::now().toSec();
    //static double time_pre = time_cur;
    ////--- END 특정 루프의 타이머를 체크할 때 사용.
    while(ros::ok()){
        ////--- START 특정 루프의 타이머를 체크할 때 사용.
        // time_cur = ros::Time::now().toSec();
        // static long countiii = 0;
        // if(++countiii == 10)
        // {
        //     countiii = 0;
        //     printf("main.cpp line %d, time period(ms): %lf \n", __LINE__, (time_cur - time_pre)*1000.0);
        // }
        // time_pre = time_cur;
        ////--- END 특정 루프의 타이머를 체크할 때 사용.
        //-----main time counter-------------------------------------------------- 
        TimeCount_T1++;
        TimeCount_W2_Front++;		// WatchDog_ *** 타이머
        TimeCount_W2_Rear++;		// WatchDog_ *** 타이머
        TimeCount_W2_RightF++;		// WatchDog_ *** 타이머
        TimeCount_W2_RightR++;		// WatchDog_ *** 타이머
        TimeCount_W3++;

        // While() 메인 타이머 체크 타이머 
        TimeMain_cur = ros::Time::now().toSec();
        TimeCheckMainLoop.time_diff = TimeMain_cur - TimeCheckMainLoop.time_pre;
        TimeCheckMainLoop.time_pre = TimeMain_cur;

//-----1. AGV 출력(AGV 주행제어 출력
            //-----1.1 AGV 주행제어를 위한 제어값 출력 함수
        if (paramEnGazebo) {
            vel_pub.publish(twist);
        } else {
            // wrapTwist.header.stamp = ros::Time::now();
            // wrapTwist.twist = twist;
            // vel_pub.publish(wrapTwist);
        }
//-----2. AGV 주행제어를 위한 상태 입력(센서입력등) 함수
//-----2.1 AGV Odometry관련 센서 및 정보 입력 함수
        FN_AGV_SYSTEM_INPUT_ODOMETRY_GET();    // 시스템 입력 함수  -- 모터드라이버 값이 주변 센서값 수신            
//-----2.2 AGV 상태 입력값으로 Odometry 함수
        FN_AGV_ODOMETRY_CALCULATE();
#if 1
        FN_AGV_POSITIONTABLEMODE();
#endif        
//-----2.3 RCM 명령 또는 AGV 내부 에러에 따른 시작 위치 재설정       
       if((chatIn_flag) || (LBox.state.ErrorStateFlag))
        {
            chatIn_flag = OFF;   // -->  아래로 이동. 에러가 아닐경우   
//-----2.4 RCM-->AGV 주행을 위한 주행 모드 맵핑
            if(LBox.state.ErrorStateFlag)
            {
                printf("recipelab_main.cpp line %d, ErrorCode : %d  , AGVControlMode : %d   \n", __LINE__, LBox.state.ErrorCode , LBox.state.AGVControlMode );
                LBox.state.ErrorStateFlag = OFF;
            }
            else
            {
//chatIn_flag = OFF;   // -->  아래로 이동. 에러가 아닐경우  -->  일단 보류
                FN_AGV_SYSTEM_INPUT_RCM_GET();          // RCM에서 들어오는 명령을 받아서 AGV 모드 정하는 함수
            }
            printf("recipelab_main.cpp line %d, ErrorCode : %d  , AGVControlMode : %d   \n", __LINE__, LBox.state.ErrorCode , LBox.state.AGVControlMode );
//-----2.5 RCM-->AGV 주행을 위한 주행 명령 수신 및 맵핑, 명령값 수신,모드별 상태변수 초기화
            FN_AGV_SYSTEM_INPUT_RCM_PROCESS();  
//-----2.6 AGV 모드별 경로 프로파일 수행 전의 모드별 경로 프로파일 변수 및 명령값 초기화
            FN_AGV_TRAJECTORY_PROFILE_INIT(&twist);
        }
//-----3. AGV 주행제어를 위한 알고리즘 수행 부분
//-----3.1 AGV 모드별 입력 처리 2(주행 알고리즘 및 속도프로파일, PID 제어) 
        if((TimeCount_T1 % TIME_50msec) == 0 )
        {
            TimeCount_T3_PID++;
            TimeMain_cur = ros::Time::now().toSec();
            TimeCheckControlLoop.time_diff = TimeMain_cur - TimeCheckControlLoop.time_pre;
            TimeCheckControlLoop.time_pre = TimeMain_cur;
            //printf("%d, %lf \n", __LINE__, ros::Time::now().toSec());
            //printf("LBox.state.AGVControlMode: %X \n", LBox.state.AGVControlMode);
            //-----3.2.1 AGV 위치 계산
            //-----3.2.2 경로 계획 속도 및 위치 프로파일 연산
            FN_AGV_TRAJECTORY_PROFILE();
            //-----3.2.3 AGV PID 제어
            FN_AGV_PID_CONTROL();
            //-----3.2.4 AGV 최종 출력값 정의     // AGV 최종 출력값 정의 이부분도 제어루틴 안에 넣어줘야 함!!!!
            //FN_AGV_SYSTEM_CONTROL_OUT(&Temptwist);
            FN_AGV_SYSTEM_CONTROL_OUT();
            // twist.linear.x = LBox.type.twist.linear.x;
            // twist.linear.y = LBox.type.twist.linear.y;    
            // twist.angular.z = LBox.type.twist.angular.z;

        }
        //-----4. AGV 내부 상태 감시를 통한 FailSafe 구현
        //-----4.1 AGV WATCHDOG 실행
        FN_AGV_SYSTEM_WATCHDOG();											// 3-1. SYSTEM WATCHDOG(입력 전압/전류, 출력 전압/전류, 온도, RS485통신
        
        // static long countiii = 0;
        // if(++countiii == 100)
        // {
        //     countiii = 0;
        //     printf("main.cpp line %d,  %d \n", __LINE__, LBox.state.ErrorState);
            
        // }
        //-----4.2 AGV 에러에 대한 처리
        //FN_AGV_SYSTEM_ERROR_DOACTING(&twist,&Temptwist);
        // 2022년4월2일 출력값 관련 수정.
        // twist 최종값은 FN_AGV_SYSTEM_CONTROL_OUT(); 이루틴에서 갱신되는 것이 아니고 
        // 아래 FN_AGV_SYSTEM_ERROR_DOACTING(&twist) 함수에서 갱신됨. !!!!
        FN_AGV_SYSTEM_ERROR_DOACTING(&twist);
            
        //-----5. RCM 송신
            //-----5.1 AGV 상태 모니터링을 위한 상위제어기 송신
            // 
        { // 사용자 출력 메시지
            whisperOut.header.stamp = ros::Time::now();
            whisperOut.val00.subject = "comd.vel.x";
            whisperOut.val00.value = LBox.type.comd.vel.x;
            whisperOut.val01.subject = "comd.vel.y";
            whisperOut.val01.value = LBox.type.comd.vel.y;
            whisperOut.val02.subject = "comd.vel.rz";
            whisperOut.val02.value = LBox.type.comd.vel.rz*R2D;

            whisperOut.val03.subject = "feed.vel.x";
            whisperOut.val03.value = LBox.type.feed.vel.x;
            whisperOut.val04.subject = "feed.vel.y";
            whisperOut.val04.value = LBox.type.feed.vel.y;
            whisperOut.val05.subject = "feed.vel.rz";
            whisperOut.val05.value = LBox.type.feed.vel.rz*R2D;

            whisperOut.val06.subject = "comd.pos.x";
            whisperOut.val06.value = LBox.type.comd.pos.x;
            whisperOut.val07.subject = "comd.pos.y";
            whisperOut.val07.value = LBox.type.comd.pos.y;
            whisperOut.val08.subject = "comd.pos.z";
            whisperOut.val08.value = LBox.type.comd.pos.rz*R2D;

            whisperOut.val09.subject = "feed.pos.x";
            whisperOut.val09.value = LBox.type.feed.pos.x;
            whisperOut.val10.subject = "feed.pos.y";
            whisperOut.val10.value = LBox.type.feed.pos.y;
            whisperOut.val11.subject = "feed.pos.rz";
            whisperOut.val11.value = LBox.type.feed.pos.rz*R2D;

            whisperOut.val12.subject = "pid.x.Out";
            whisperOut.val12.value = LBox.control.pid.x.Out;
            whisperOut.val13.subject = "pid.y.Out";
            whisperOut.val13.value = LBox.control.pid.y.Out;
            whisperOut.val14.subject = "pid.rz.Out";
            whisperOut.val14.value = LBox.control.pid.rz.Out;

            whisperOut.val15.subject = "pid.x.Ref";
            whisperOut.val15.value = LBox.control.pid.x.Ref;
            whisperOut.val16.subject = "pid.y.Ref";
            whisperOut.val16.value = LBox.control.pid.y.Ref;
            whisperOut.val17.subject = "pid.rz.Ref";
            whisperOut.val17.value = LBox.control.pid.rz.Ref*R2D;

            whisperOut.val18.subject = "pid.x.Fdb";
            whisperOut.val18.value = LBox.control.pid.x.Fdb;
            whisperOut.val19.subject = "pid.y.Fdb";
            whisperOut.val19.value = LBox.control.pid.y.Fdb;
            whisperOut.val20.subject = "pid.rz.Fdb";
            whisperOut.val20.value = LBox.control.pid.rz.Fdb*R2D;
            
            whisperOut.val21.subject = "twist linear x";
            whisperOut.val21.value = twist.linear.x;
            whisperOut.val22.subject = "twist linear y";
            whisperOut.val22.value = twist.linear.y;
            whisperOut.val23.subject = "twist angular z";
            whisperOut.val23.value = twist.angular.z;
            //static int tempCount = 0;
            /*
            whisperOut.val24.subject = "PxProfileState";
            whisperOut.val24.value = AGVPx.StartFlag;
            whisperOut.val25.subject = "PyProfileState";
            whisperOut.val25.value = AGVPy.StartFlag;
            whisperOut.val26.subject = "RzProfileState";
            whisperOut.val26.value = AGVYaw.StartFlag;
            whisperOut.val27.subject = "ErrorState";
            whisperOut.val27.value = LBox.state.ErrorState;
            whisperOut.val28.subject = "ErrorCode";
            whisperOut.val28.value = LBox.state.ErrorCode;
            whisperOut.val29.subject = "ErrorStateFlag";
            whisperOut.val29.value = LBox.state.ErrorStateFlag; 
            */
            whisperOut.val24.subject = "OdomImuBase_pos_rz";
            whisperOut.val24.value = OdomImuBase.odom.now.pos.rz *R2D;
            whisperOut.val25.subject = "feed.pos.rz_Q2O";
            whisperOut.val25.value = LBox.type.feed.pos.rz*R2D;
            //OdomAmclBase
            whisperOut.val26.subject = "OdomAmclBase_pos_x";
            whisperOut.val26.value = OdomAmclBase.odom.now.pos.x;
            whisperOut.val27.subject = "OdomAmclBase_pos_y";
            whisperOut.val27.value = OdomAmclBase.odom.now.pos.y;
            whisperOut.val28.subject = "OdomAmclBase_pos_rz";
            whisperOut.val28.value = OdomAmclBase.odom.now.pos.rz;
            whisperOut.val29.subject = "ErrorStateFlag";
            whisperOut.val29.value = LBox.state.ErrorStateFlag;            
            whisper_pub.publish(whisperOut);
        }    

        {               // -----6. RCM 송신 LabVIEW UI display topic start
            chatOut.header.stamp = ros::Time::now();

            chatOut.sensorState.front = range_fr_.range;
            chatOut.sensorState.back = range_br_.range;
            chatOut.sensorState.right_front = range_rf_.range;
            chatOut.sensorState.right_back = range_rb_.range;

            chatOut.posState.x = LBox.type.feed.pos.x;
            chatOut.posState.y = LBox.type.feed.pos.y;
            chatOut.posState.rz = LBox.type.feed.pos.rz*R2D;
            chatOut.posState.ang = Odom1AxisLidarBase.odom.feed.pos.rz*R2D;

            chatOut.platformState.mode = LBox.state.AGVControlMode;
            chatOut.platformState.state = LBox.state.AgvRunState;

            chatOut.batteryState.voltage = battery_.voltage;
            chatOut.batteryState.current = battery_.current;
            chatOut.batteryState.percentage = battery_.percentage;

            chat_pub.publish(chatOut);
        }
            // LabVIEW UI display topic end
        //-----5.2 제어 알고리즘 확인을 위한 화면 PRINTF --> 디버깅용
        if((TimeCount_T1 % (__uint32_t)TIME_1000msec) == 0 )
        {           
            TimeMain_cur = ros::Time::now().toSec();
            TimeCheckPrintfLoop.time_diff = TimeMain_cur - TimeCheckPrintfLoop.time_pre;
            TimeCheckPrintfLoop.time_pre = TimeMain_cur;
            FN_AGV_SYSTEM_OUTPUT_PRINTF(&twist);         // 임시 Printf 출력함수  
        }

        #if 0
        if (pose_callbacked) {
            nav_msgs::Path nav_path;
            nav_path.header.stamp = ros::Time::now();
            nav_path.header.frame_id = "map";
            geometry_msgs::PoseStamped geo_pose;
            geo_pose.header.stamp = nav_path.header.stamp;
            geo_pose.pose.position.x = pose_.pose.pose.position.x;
            geo_pose.pose.position.y = pose_.pose.pose.position.y;
            geo_pose.pose.position.z = pose_.pose.pose.position.z;
            geo_pose.pose.orientation.x = pose_.pose.pose.orientation.x;
            geo_pose.pose.orientation.y = pose_.pose.pose.orientation.y;
            geo_pose.pose.orientation.z = pose_.pose.pose.orientation.z;
            geo_pose.pose.orientation.w = pose_.pose.pose.orientation.w;
            nav_path.poses.push_back(geo_pose);
            path_pub.publish(nav_path);
            pose_callbacked = 0;
        }
        #endif

    	ros::spinOnce();
    	maintimer.sleep();
    }

    threadFileOperationLoop.join();

    return 0;
}
/*
ros::spinOnce();  
//호출되는 순간 콜백요청 큐에 쌓여있는 모든 요청을 처리하고 종료된다.콜백함수를 어느정도 빠르기로 처리할지 헤르츠를 설정하고, 
while의 조건문에서 ros::ok로 노드의 종료 여부를 판단한 후 사용자가 하고싶은 작업을 실행한다.
그리고 spinOnce를 통해서 사용자의 작업이 진행되는 동안 쌓인 콜백함수 요청을 처리하고, 
sleep을 통해서 원하는 만큼 기다린 후 다시 while문을 통해서 이전까지의 작업을 반복하는 것이다.
*/

void AGVSensorsTopicSubscriberSet(ros::NodeHandle nh){
    // // imu topic
    // ros::Subscriber imu_sub = nh.subscribe("/mw_ahrsv1/imu/data", 100, imuCallBack);
    // // lidar fr topic
    // ros::Subscriber lidar_fr_sub = nh.subscribe("/rplidarNode_fr/scan", 100, lidar2dFrCallBack);
    // // lidar bl topic
    // ros::Subscriber lidar_bl_sub = nh.subscribe("/rplidarNode_bl/scan", 100, lidar2dBlCallBack);
    // // tf40 rf topic
    // ros::Subscriber range_rf_sub = nh.subscribe("/tf40_rf/tf40_rf", 100, rangeRfCallBack);
    // // tf40 rb topic
    // ros::Subscriber range_rb_sub = nh.subscribe("/tf40_rb/tf40_rb", 100, rangeRbCallBack);
    // // tf02 fr topic
    // ros::Subscriber range_fr_sub = nh.subscribe("/tf02_fr/TF02_fr", 100, rangeFrCallBack);
    // // tf02 br topic    
    // ros::Subscriber range_br_sub = nh.subscribe("/tf02_br/TF02_br", 100, rangeBrCallBack);
    // // tfmini rf topic
    // ros::Subscriber range_rf_sub = nh.subscribe("/tfmini_rf/TFmini_rf", 100, rangeRfCallBack);
    // // tfmini rb topic    
    // ros::Subscriber range_rb_sub = nh.subscribe("/tfmini_rb/TFmini_rb", 100, rangeRbCallBack);
    // // odom topic	
    // ros::Subscriber odom_sub = nh.subscribe("/md_node/odom", 100, odomCallBack);
    // // vel topic
    // ros::Subscriber vel_sub = nh.subscribe("/cmd_vel", 100, velCallBack);
    // // pose2d topic
    // ros::Subscriber pose2d_sub = nh.subscribe("/pose2d_sender/pose2d_command", 100, pose2dCallBack);

    // // vel topic
    // ros::Publisher vel_pub = nh.advertise<geometry_msgs::Twist>("/md_node/md_vel_topic", 10);
    // // chatOut topic
    // ros::Publisher chat_pub = nh.advertise<chatterbox::ChatOut>("/chatterbox/chatOut_topic", 10);
}
void AGVSensorsCheckFn(void)
{
    ros::Rate Timer1(100);
    while(ros::ok()) {
        if(!range_rf_callbacked||!range_rb_callbacked||!range_fr_callbacked||!range_br_callbacked)
        {
            printf("range_rf:%d, range_rb:%d, range_fr:%d, range_br:%d\n", range_rf_callbacked, range_rb_callbacked, range_fr_callbacked, range_br_callbacked);
            
            ros::spinOnce();

            Timer1.sleep();
        } 
        else
        {   
            printf("line: %d ok\n", __LINE__);
            break;
        }
    }
}
void AGVOdomWheelbaseGet(double x, double y, double qx, double qy, double qz, double w, AGV_ODOMETRY *fodom)
{
    // feed value save
    fodom->odom.feed.pos.x = lodom_.pose.pose.position.x;
    fodom->odom.feed.pos.y = lodom_.pose.pose.position.y;
    tf::Quaternion q(
        lodom_.pose.pose.orientation.x,
        lodom_.pose.pose.orientation.y,
        lodom_.pose.pose.orientation.z,
        lodom_.pose.pose.orientation.w);
    tf::Matrix3x3 m(q);
    
    m.getRPY(fodom->odom.feed.pos.rx, fodom->odom.feed.pos.ry, fodom->odom.feed.pos.rz);
}
void AGVOdomImubaseGet(double x, double y, double z, double w,AGV_ODOMETRY *fodom)
{
    // feed value save
    fodom->odom.feed.pos.x = x;
    fodom->odom.feed.pos.y = y;
    fodom->odom.feed.pos.z = z;
    fodom->odom.feed.pos.w = w;
    tf::Quaternion q(
        fodom->odom.feed.pos.x,
        fodom->odom.feed.pos.y,
        fodom->odom.feed.pos.z,
        fodom->odom.feed.pos.w);
    tf::Matrix3x3 m(q);
    
    m.getRPY(fodom->odom.feed.pos.rx, fodom->odom.feed.pos.ry, fodom->odom.feed.pos.rz);
}
void AGVOdom1AxisLidarbaseGet(AGV_ODOMETRY *fOdom1AxisLidarBase)
{
    AGVLocationRecognition(range_rf_.range,range_rb_.range,range_br_.range,range_fr_.range,&AGVOdometry_TFSensor);
    fOdom1AxisLidarBase->odom.feed.pos.x = AGVOdometry_TFSensor.x1;                     // 기구학 계산을 거쳐서 나온 AGV Odometry Px
    fOdom1AxisLidarBase->odom.feed.pos.y = AGVOdometry_TFSensor.y1;                     // 기구학 계산을 거쳐서 나온 AGV Odometry Py
    fOdom1AxisLidarBase->odom.feed.pos.rz = AGVOdometry_TFSensor.theta;                 // 기구학 계산을 거쳐서 나온 AGV Odometry Yaw

}
void AGVOdomAmclbaseGet(double x, double y, double qx, double qy, double qz, double w, AGV_ODOMETRY *fodom)
{
    // feed value save
    fodom->odom.feed.pos.x = lodom_.pose.pose.position.x;
    fodom->odom.feed.pos.y = lodom_.pose.pose.position.y;
    tf::Quaternion q(
        lodom_.pose.pose.orientation.x,
        lodom_.pose.pose.orientation.y,
        lodom_.pose.pose.orientation.z,
        lodom_.pose.pose.orientation.w);
    tf::Matrix3x3 m(q);
    
    m.getRPY(fodom->odom.feed.pos.rx, fodom->odom.feed.pos.ry, fodom->odom.feed.pos.rz);
}
void AGVOdomPoseInit(void)
{

}
void ProfileValInit(void)		// 경로계획 관련 변수 초기화 함수
{	
    AGVPx.StartFlag	= PROFILEStop;      AGVPy.StartFlag= PROFILEStop;       AGVYaw.StartFlag	= PROFILEStop;
//    AGVPx.ONFlag    = OFF;              AGVPy.ONFlag= OFF;                  AGVYaw.ONFlag    = OFF;
//    AGVPx.SlopUp    = OFF;              AGVPy.SlopUp= OFF;                  AGVYaw.SlopUp    = OFF;
//    AGVPx.SlopDown  = OFF;              AGVPy.SlopDown= OFF;                AGVYaw.SlopDown    = OFF;
    
//    AGVPx.V_in		= 0.1;      /* m/sec */         AGVPy.V_in		= 0.1;          AGVYaw.V_in		    = 10.0*D2R;      /* rad/sec */	
//    AGVPx.A_in		= 0.01;     /* m/sec^2	*/      AGVPy.A_in		= 0.01;         AGVYaw.A_in		    = 1.0*D2R;		/* rad/sec^2 */
//    AGVPx.T_in		= 1.0;      /* sec */           AGVPy.T_in		= 1.0;          AGVYaw.T_in		    = 1.0;              /* sec */				
    AGVPx.Max_Vel 	= 0.5;      /* m/sec */	        AGVPy.Max_Vel 	= 0.5;          AGVYaw.Max_Vel 	    = 30.0*D2R;     /* rad/sec */		
    AGVPx.Max_Accel = 1.0;     /* m/sec^2 */	    AGVPy.Max_Accel = 1.0;         AGVYaw.Max_Accel 	= 60.0*D2R;     /* rad/sec^2 */        		
}
void AGVVariableInit(void)		// AGV 관련 변수 초기화 함수
{	
    LBox.type.tcomd.pos.x        = 0.0;			        // (m)Cartecian 좌표계에서 AGV의 X축 직선이동해야할 좌표값(절대좌표) 
    LBox.type.tcomd.pos.y        = 0.0;			        // (m)Cartecian 좌표계에서 AGV의 Y축 직선이동해야할 좌표값(절대좌표)
    LBox.type.tcomd.pos.rz       = 0.0; 		            // (rad)Cartecian 좌표계에서 AGV의 Z축 회전이동 해야할 좌표값(절대좌표)
    LBox.type.comd.pos.x        = 0.0;			        // (m)Cartecian 좌표계에서 AGV의 X축 직선이동해야할 좌표값(절대좌표) 
    LBox.type.comd.pos.y        = 0.0;			        // (m)Cartecian 좌표계에서 AGV의 Y축 직선이동해야할 좌표값(절대좌표)
    LBox.type.comd.pos.rz       = 0.0; 		            // (rad)Cartecian 좌표계에서 AGV의 Z축 회전이동 해야할 좌표값(절대좌표) 
    LBox.type.tcomd.vel.x        = 0.0;			        // (m)Cartecian 좌표계에서 AGV의 X축 직선이동해야할 좌표값(절대좌표) 
    LBox.type.tcomd.vel.y        = 0.0;			        // (m)Cartecian 좌표계에서 AGV의 Y축 직선이동해야할 좌표값(절대좌표)
    LBox.type.tcomd.vel.rz       = 0.0; 		            // (rad)Cartecian 좌표계에서 AGV의 Z축 회전이동 해야할 좌표값(절대좌표)
    LBox.type.tcomd.acc.x       = 0.05;                 // AGV 가속도 설정값 X축
    LBox.type.tcomd.acc.y       = 0.05;                 // AGV 가속도 설정값 Y축
    LBox.type.tcomd.acc.rz       = 0.017;                 // AGV 가속도 설정값 Z축 회전
    LBox.type.comd.vel.x        = 0.0;			        // (m)Cartecian 좌표계에서 AGV의 X축 직선이동해야할 좌표값(절대좌표) 
    LBox.type.comd.vel.y        = 0.0;			        // (m)Cartecian 좌표계에서 AGV의 Y축 직선이동해야할 좌표값(절대좌표)
    LBox.type.comd.vel.rz       = 0.0; 		            // (rad)Cartecian 좌표계에서 AGV의 Z축 회전이동 해야할 좌표값(절대좌표)
    LBox.type.feed.pos.x        = 0.0;			        // (m)현재 라이다에 의해 기구학 계산된 AGV Odometry X 좌표(절대좌표)
    LBox.type.feed.pos.y        = 0.0;			        // (m)현재 라이다에 의해 기구학 계산된 AGV Odometry Y 좌표(절대좌표) 
    LBox.type.feed.pos.rz       = 0.0;		            // (rad)현재 라이다에 의해 기구학 계산된 AGV Odometry Yaw 좌표(절대좌표) 
    LBox.state.AGVControlMode   = READYMODE;	        // AGV 제어 모드 설정 (JOGMODE ,POSMODE, INITMODE, READYMODE)
    LBox.state.AgvRunState      = READY;             // AGV 상태  설정  (READY, RUN, RUNNING, STOP, ERROR)
//    LBox.state.AgvMode3         = READYMODE;            // AGV 제어 모드 백업 변수
    LBox.state.AGVOdomType      = ODOM_KINEMATIC;       // AGV odometry 타입  -->  기본은 kinematic 
    LBox.control.inpos.x        = OFF;                  // AGV 위치제어 시 목표위치 도달했는지를 확인하는 변수 
    LBox.control.inpos.y        = OFF;
    LBox.control.inpos.rz       = OFF; 
    LBox.control.inpos.all      = OFF; 
    LBox.state.ErrorStateFlag   = OFF;  

    LBox.watchdog.WatchDogPositonERRStartFalg = OFF;               	        		
}
// void AGVWatchDogVariableInit(void){
    
// }
#ifdef GAIN_TUNING
void AGVPIDValInit(dynamic_reconfigure::Server<recipelab::GainTuningConfig>* srvGainTuning)
#else
void AGVPIDValInit(void)
#endif
{
    LBox.control.pid.x.Ref 	    =  0;			LBox.control.pid.y.Ref 	    =  0;		LBox.control.pid.rz.Ref     = 0;	
    LBox.control.pid.x.Kp 	    =  1.5;			LBox.control.pid.y.Kp 	    =  1.5;		LBox.control.pid.rz.Kp 		= 3.0;		
    LBox.control.pid.x.Ki 	    =  0.0;			LBox.control.pid.y.Ki 	    =  0.0;		LBox.control.pid.rz.Ki 		= 0.0;		
    LBox.control.pid.x.Kd 	    =  0.01;		LBox.control.pid.y.Kd 	    =  0.01;	LBox.control.pid.rz.Kd 		= 0.01;		
    LBox.control.pid.x.Kc       =  0.0;			LBox.control.pid.y.Kc 	    =  0.0;		LBox.control.pid.rz.Kc 		= 0.0;		
    LBox.control.pid.x.OutMax   =  0.6;			LBox.control.pid.y.OutMax   =  0.6;		LBox.control.pid.rz.OutMax 	= 30*D2R;		
    LBox.control.pid.x.OutMin   = -0.6;			LBox.control.pid.y.OutMin   = -0.6;		LBox.control.pid.rz.OutMin 	= -30*D2R;

#ifdef GAIN_TUNING
    recipelab::GainTuningConfig config;
    config.x_Kp = LBox.control.pid.x.Kp;
    config.x_Ki = LBox.control.pid.x.Ki;
    config.x_Kd = LBox.control.pid.x.Kd;
    config.x_Kc = LBox.control.pid.x.Kc;

    config.y_Kp = LBox.control.pid.y.Kp;
    config.y_Ki = LBox.control.pid.y.Ki;
    config.y_Kd = LBox.control.pid.y.Kd;
    config.y_Kc = LBox.control.pid.y.Kc;

    config.rz_Kp = LBox.control.pid.rz.Kp;
    config.rz_Ki = LBox.control.pid.rz.Ki;
    config.rz_Kd = LBox.control.pid.rz.Kd;
    config.rz_Kc = LBox.control.pid.rz.Kc;

#define GAIN_MIN_OFFSET 0
#define GAIN_MAX_OFFSET 10
    recipelab::GainTuningConfig config_min;
    config_min.x_Kp = GAIN_MIN_OFFSET;
    config_min.x_Ki = GAIN_MIN_OFFSET;
    config_min.x_Kd = GAIN_MIN_OFFSET;
    config_min.x_Kc = GAIN_MIN_OFFSET;

    config_min.y_Kp = GAIN_MIN_OFFSET;
    config_min.y_Ki = GAIN_MIN_OFFSET;
    config_min.y_Kd = GAIN_MIN_OFFSET;
    config_min.y_Kc = GAIN_MIN_OFFSET;

    config_min.rz_Kp = GAIN_MIN_OFFSET;
    config_min.rz_Ki = GAIN_MIN_OFFSET;
    config_min.rz_Kd = GAIN_MIN_OFFSET;
    config_min.rz_Kc = GAIN_MIN_OFFSET;

    recipelab::GainTuningConfig config_max;
    config_max.x_Kp = GAIN_MAX_OFFSET;
    config_max.x_Ki = GAIN_MAX_OFFSET;
    config_max.x_Kd = GAIN_MAX_OFFSET;
    config_max.x_Kc = GAIN_MAX_OFFSET;

    config_max.y_Kp = GAIN_MAX_OFFSET;
    config_max.y_Ki = GAIN_MAX_OFFSET;
    config_max.y_Kd = GAIN_MAX_OFFSET;
    config_max.y_Kc = GAIN_MAX_OFFSET;

    config_max.rz_Kp = GAIN_MAX_OFFSET;
    config_max.rz_Ki = GAIN_MAX_OFFSET;
    config_max.rz_Kd = GAIN_MAX_OFFSET;
    config_max.rz_Kc = GAIN_MAX_OFFSET;

    srvGainTuning->setConfigMin(config_min);
    srvGainTuning->setConfigMax(config_max);
    srvGainTuning->setConfigDefault(config);
    srvGainTuning->updateConfig(config);
#endif
}
void AGVOdomValInit(void)
{
        OdomImuBase.odom.init.pos.rz        = 0.0;
        OdomWheelBase.odom.init.pos.x       = 0.0;
        OdomWheelBase.odom.init.pos.y       = 0.0;
        OdomWheelBase.odom.init.pos.rz      = 0.0;
        Odom1AxisLidarBase.odom.init.pos.x  = 0.0;
        Odom1AxisLidarBase.odom.init.pos.y  = 0.0;
        Odom1AxisLidarBase.odom.init.pos.rz = 0.0;
        OdomAmclBase.odom.init.pos.x       = 0.0;
        OdomAmclBase.odom.init.pos.y       = 0.0;
        OdomAmclBase.odom.init.pos.rz      = 0.0;
        
}
void AGVPIDValReset(void)
{
	LBox.control.pid.x.Ui		= 0.0;      LBox.control.pid.y.Ui		= 0.0;      LBox.control.pid.rz.Ui		= 0.0;
	LBox.control.pid.x.Ud		= 0.0;      LBox.control.pid.y.Ud		= 0.0;      LBox.control.pid.rz.Ud		= 0.0;
	LBox.control.pid.x.Out	    = 0.0;      LBox.control.pid.y.Out	    = 0.0;      LBox.control.pid.rz.Out	    = 0.0;      
}
void AGVTimeValInit(void)
{
    TimeCount_T1        = 0;
    TimeCount_T2_kalman = 0;        // Kalman Filter 타이머
    TimeCount_T3_PID    = 0;
    TimeCount_W2_Front  = 0;		// WatchDog_ *** 타이머
    TimeCount_W2_Rear   = 0;		// WatchDog_ *** 타이머
    TimeCount_W2_RightF = 0;		// WatchDog_ *** 타이머
    TimeCount_W2_RightR = 0;		// WatchDog_ *** 타이머
    TimeCount_W3        = 0;

    TimeMain_cur = ros::Time::now().toSec();
    TimeCheckMainLoop.time_pre = TimeMain_cur;
    TimeCheckPrintfLoop.time_pre = TimeMain_cur;
    TimeCheckControlLoop.time_pre = TimeMain_cur;
    TimeCheckOdomWheelget.time_pre = TimeMain_cur;
    TimeCheckOdomKalmanFilter.time_pre = TimeMain_cur;
} 
void SYSVariableInit(void)
{
    LBox.state.ErrorCode    = 0;
    LBox.state.ErrorState   = 0;
}
void FN_AGV_SYSTEM_OUTPUT_PRINTF(geometry_msgs::Twist *twist)         // 임시 Printf 출력함수
{
    //  printf("LBox.state.ErrorCode: %d \n", LBox.state.ErrorCode);
    //  printf("LBox.state.ErrorState: %d \n", LBox.state.ErrorState);
    // printf("AGVFrontRange: %f AGVRearRange : %f AGVRigntFrontRange : %f AGVRigntRearRange : %f \n", AGVFrontRange, AGVRearRange, AGVRigntFrontRange, AGVRigntRearRange);

    //printf("TIME_10msec : %f , TIME_1000msec : %f , TIME_50msec : %f \n", TimeCheckMainLoop.time_diff , TimeCheckPrintfLoop.time_diff , TimeCheckControlLoop.time_diff);
    //printf("LBox.state.AGVControlMode: %X \n", LBox.state.AGVControlMode);
    //printf("LBox.state.AgvRunState: %X \n", LBox.state.AgvRunState);
    //printf("AGV type.comd.vel.xvel: %f , %f , %f\n", LBox.type.comd.vel.x, LBox.type.comd.vel.y, LBox.type.comd.vel.rz*R2D);
    //printf("AGV type.comd.pos.xose: %f , %f , %f\n", LBox.type.comd.pos.x, LBox.type.comd.pos.y, LBox.type.comd.pos.rz*R2D);
    //printf("AGV type.comd.twist: %f , %f , %f\n", twist->linear.x,twist->linear.y,twist->angular.z*R2D);
    //("AGV type.feed.vel.xvel: %f , %f , %f\n", LBox.type.feed.vel.x, LBox.type.feed.vel.y, LBox.type.feed.vel.rz*R2D);
    //printf("AGV PID OUTPUT: %f , %f , %f\n", LBox.control.pid.x.Out, LBox.control.pid.y.Out, LBox.control.pid.rz.Out*R2D);
    //printf("AGV STATE: %X ,\n", LBox.state.AGVControlMode);           // 16진수 대문자 출력
   // printf("AGV STATE: %X , %X , %X ,\n", AGVPx.StartFlag,AGVPy.StartFlag,AGVYaw.StartFlag);           // 16진수 대문자 출력
    //printf("AGV OdomWheelBase: %f , %f , %f\n", OdomWheelBase.odom.feed.pos.x , OdomWheelBase.odom.feed.pos.y, OdomWheelBase.odom.feed.pos.rz*R2D);
    //printf("AGV OdomWheelBase: %f , %f , %f\n", OdomWheelBase.odom.now.pos.x , OdomWheelBase.odom.now.pos.y, OdomWheelBase.odom.now.pos.rz*R2D);
    //printf("AGV OdomImuBase: %f , %f , %f\n", OdomImuBase.odom.now.pos.x , OdomImuBase.odom.now.pos.y, OdomImuBase.odom.now.pos.rz*R2D);
    //printf("AGV Odom1AxisLidarBase: %f , %f , %f\n", Odom1AxisLidarBase.odom.feed.pos.x , Odom1AxisLidarBase.odom.feed.pos.y, Odom1AxisLidarBase.odom.feed.pos.rz*R2D);
    //printf("AGV OdomKalmanBase: %f , %f , %f\n", OdomKalmanBase.odom.now.pos.x , OdomKalmanBase.odom.now.pos.y, OdomKalmanBase.odom.now.pos.rz*R2D);
}
void FN_AGV_SYSTEM_INPUT_ODOMETRY_GET(void)    // 시스템 입력 함수  -- 모터드라이버 값이 주변 센서값 수신
{
    // -2.1 AGV 주행제어에 필요한 주변 센서 수신
    // --2.1.1 IMU 센서를 통한 Odometry --> RZ 만 계산됨.
    if(imu_callbacked){
        imu_callbacked = 0x00;
        AGVOdomImubaseGet(imu_.orientation.x, imu_.orientation.y, imu_.orientation.z, imu_.orientation.w, &OdomImuBase);
        //2022년 10월26일 추가   쿼터니언 -180 ~ 180 값을 무한값으로 변경
        //OdomImuBase.odom.feed_buff.pos.rz =  fmod(((OdomImuBase.odom.feed.pos.rz-OdomImuBase.odom.feed_old.pos.rz) +  2.0*M_PI), 2.0*M_PI);
        //OdomImuBase.odom.feed_old.pos.rz = OdomImuBase.odom.feed.pos.rz;
        //OdomImuBase.odom.feed_now.pos.rz = OdomImuBase.odom.feed_now.pos.rz + OdomImuBase.odom.feed_buff.pos.rz;
        //OdomImuBase.odom.feed_now.pos.rz = OdomImuBase.odom.feed_now.pos.rz + fmod((OdomImuBase.odom.feed.pos.rz +  2.0*M_PI), 2.0*M_PI);
        OdomImuBase.odom.feed_now.pos.rz = fmod(OdomImuBase.odom.feed.pos.rz , M_PI);
        /*
        OdomImuBase.odom.feed_buff.pos.rz = fmod((OdomImuBase.odom.feed.pos.rz +  2.0*M_PI), 2.0*M_PI);
        OdomImuBase.odom.feed_now.pos.rz = OdomImuBase.odom.feed_now.pos.rz + (OdomImuBase.odom.feed_buff.pos.rz - OdomImuBase.odom.feed_old.pos.rz);
        OdomImuBase.odom.feed_old.pos.rz = OdomImuBase.odom.feed_buff.pos.rz;
        */
    }
    // --2.1.2 모터 엔코더에서 메카넘휠 기구학을 통한 Odometry --> X,Y, RZ 계산됨.
    if(lodom_callbacked)
    {
        lodom_callbacked = 0x00;
        // AGV odometry 위치 정보 수신, mecanum 패키지에서 수신
        AGVOdomWheelbaseGet(lodom_.pose.pose.position.x, lodom_.pose.pose.position.y,
        lodom_.pose.pose.orientation.x,lodom_.pose.pose.orientation.y,lodom_.pose.pose.orientation.z,lodom_.pose.pose.orientation.w, 
        &OdomWheelBase);
       
        //2022년 10월26일 추가   쿼터니언 -180 ~ 180 값을 무한값으로 변경
        //fmod((OdomWheelBase.odom.now.pos.rz + 2.0*M_PI), 2.0*M_PI);
        // AGV odometry 속도 정보 수신, mecanum 패키지에서 수신, 속도값 다이렉트 수신, 상대속도값임.
        OdomWheelBase.odom.now.vel.x = lodom_.twist.twist.linear.x;
        OdomWheelBase.odom.now.vel.y = lodom_.twist.twist.linear.y;
        OdomWheelBase.odom.now.vel.rz = lodom_.twist.twist.angular.z;  
        // !!! 중요  2022년 10월27일 임시로 각도값을 lodom.twist.twist.linear.z에 넣어서 보냄. 나중에 삭제(문제시 삭제)
         OdomWheelBase.odom.feed.pos.rz = lodom_.twist.twist.linear.z;       
    }
    // --2.1.3 1축 TF 라이다 센서에서 LocationRecognition 통한 Odometry --> X,Y, RZ 계산됨.
    // 적절한 if문을 넣어야 함. 추후
    AGVFrontRange = range_fr_.range;
    AGVRearRange  = range_br_.range;
    AGVRigntFrontRange = range_rf_.range;
    AGVRigntRearRange = range_rb_.range;
    AGVOdom1AxisLidarbaseGet(&Odom1AxisLidarBase);
    // --2.1.4 amcl 2축 RPLIDAR 두개를 적용한 라이다 기반 odometry 정보
    if (pose_callbacked) {
        pose_callbacked = 0x00;
        AGVOdomAmclbaseGet(pose_.pose.pose.position.x, pose_.pose.pose.position.y,
        pose_.pose.pose.orientation.x,pose_.pose.pose.orientation.y,pose_.pose.pose.orientation.z,pose_.pose.pose.orientation.w, 
        &OdomAmclBase);
        // AGV odometry 속도 정보 수신, 속도값은 상대값임으로 mecanum 패키지에서 수신, 속도값 다이렉트 수신, 상대속도값임.
        OdomAmclBase.odom.now.vel.x = lodom_.twist.twist.linear.x;
        OdomAmclBase.odom.now.vel.y = lodom_.twist.twist.linear.y;
        OdomAmclBase.odom.now.vel.rz = lodom_.twist.twist.angular.z;         
    }
    // -2.2 AGV 최종 위치 value 계산 
    // OdomImuBase , OdomWheelBase , Odom1AxisLidarBase feed value를 기준으로 최종 now value를 계산
    // 기구학 오도메트리 누적오차를 TF 센서 오도메트리를 비교하여 보정함.
    
    
    OdomImuBase.odom.now.pos.rz     = OdomImuBase.odom.feed_now.pos.rz      - OdomImuBase.odom.init.pos.rz;

    OdomWheelBase.odom.now.pos.x    = OdomWheelBase.odom.feed.pos.x     - OdomWheelBase.odom.init.pos.x ;
    OdomWheelBase.odom.now.pos.y    = OdomWheelBase.odom.feed.pos.y     - OdomWheelBase.odom.init.pos.y ;
    OdomWheelBase.odom.now.pos.rz   = OdomWheelBase.odom.feed.pos.rz    - OdomWheelBase.odom.init.pos.rz ;

    Odom1AxisLidarBase.odom.now.pos.x   = Odom1AxisLidarBase.odom.feed.pos.x  - Odom1AxisLidarBase.odom.init.pos.x ;
    Odom1AxisLidarBase.odom.now.pos.y   = Odom1AxisLidarBase.odom.feed.pos.y  - Odom1AxisLidarBase.odom.init.pos.y ;
    Odom1AxisLidarBase.odom.now.pos.rz  = Odom1AxisLidarBase.odom.feed.pos.rz - Odom1AxisLidarBase.odom.init.pos.rz ;

    OdomAmclBase.odom.now.pos.x    = OdomAmclBase.odom.feed.pos.x    - OdomAmclBase.odom.init.pos.x ;
    OdomAmclBase.odom.now.pos.y    = OdomAmclBase.odom.feed.pos.y    - OdomAmclBase.odom.init.pos.y ;
    OdomAmclBase.odom.now.pos.rz   = OdomAmclBase.odom.feed.pos.rz   - OdomAmclBase.odom.init.pos.rz ;
}
void FN_AGV_ODOMETRY_CALCULATE(void)
{
    double ErrValue_X;
    double ErrValue_Y;
    double ErrValue_RZ;
    //AGVOdomType
    // AGV Odometry Type
    LBox.state.AGVOdomType = chatIn_.sensor;
    switch(LBox.state.AGVOdomType)
    {
        case ODOM_KINEMATIC:
            LBox.type.feed.pos.x    = OdomWheelBase.odom.now.pos.x;                // 기구학 계산을 거쳐서 나온 AGV Odometry Px
            LBox.type.feed.pos.y    = OdomWheelBase.odom.now.pos.y;                // 기구학 계산을 거쳐서 나온 AGV Odometry Py
            LBox.type.feed.pos.rz   = OdomWheelBase.odom.now.pos.rz;               // 기구학 계산을 거쳐서 나온 AGV Odometry Yaw

            LBox.type.feed.vel.x    = OdomWheelBase.odom.now.vel.x;               // 속도 기구학 계산을 거쳐서 나온 AGV 속도 odometry
            LBox.type.feed.vel.y    = OdomWheelBase.odom.now.vel.y;               // 속도 기구학 계산을 거쳐서 나온 AGV 속도 odometry
            LBox.type.feed.vel.rz   = OdomWheelBase.odom.now.vel.rz;              // 속도 기구학 계산을 거쳐서 나온 AGV 속도 odometry
            //  printf("recipelab main.cpp line: %d -- kinematic mode --- LBox.type.feed.pos.x: %f , LBox.type.feed.pos.y: %f, LBox.type.feed.pos.rz: %f\n", __LINE__, LBox.type.feed.pos.x, LBox.type.feed.pos.y,LBox.type.feed.pos.rz);
        break;
        case ODOM_AXIS1LIDAR:
            LBox.type.feed.pos.x    = Odom1AxisLidarBase.odom.now.pos.x;           // TF센서를 통해 나온 AGV Odometry Px
            LBox.type.feed.pos.y    = Odom1AxisLidarBase.odom.now.pos.y;           // TF센서를 통해 나온 AGV Odometry Py
            LBox.type.feed.pos.rz   = Odom1AxisLidarBase.odom.now.pos.rz;          // TF센서를 통해 나온 AGV Odometry Yaw

            // LBox.type.feed.vel.x    = Odom1AxisLidarBase.odom.now.vel.x;               // 속도 기구학 계산을 거쳐서 나온 AGV 속도 odometry
            // LBox.type.feed.vel.y    = Odom1AxisLidarBase.odom.now.vel.y;               // 속도 기구학 계산을 거쳐서 나온 AGV 속도 odometry
            // LBox.type.feed.vel.rz   = Odom1AxisLidarBase.odom.now.vel.rz;              // 속도 기구학 계산을 거쳐서 나온 AGV 속도 odometry
            // printf("line: %d -- 1axis lidar mode --- LBox.type.feed.pos.x: %f , LBox.type.feed.pos.y: %f, LBox.type.feed.pos.rz: %f\n", __LINE__, LBox.type.feed.pos.x, LBox.type.feed.pos.y,LBox.type.feed.pos.rz);
        break;
        case ODOM_COMPLEMENT:
            ErrValue_X  = abs(OdomWheelBase.odom.now.pos.x - Odom1AxisLidarBase.odom.now.pos.x);
            ErrValue_Y  = abs(OdomWheelBase.odom.now.pos.y - Odom1AxisLidarBase.odom.now.pos.y);
            ErrValue_RZ = abs(OdomWheelBase.odom.now.pos.rz - Odom1AxisLidarBase.odom.now.pos.rz);
            if((ErrValue_X > 0.1) || (ErrValue_Y > 0.1) || (ErrValue_RZ > 0.087))
            {
                LBox.type.feed.pos.x = OdomWheelBase.odom.now.pos.x;
                LBox.type.feed.pos.y = OdomWheelBase.odom.now.pos.y;
                LBox.type.feed.pos.rz = OdomWheelBase.odom.now.pos.rz;

                LBox.type.feed.vel.x    = OdomWheelBase.odom.now.vel.x;               // 속도 기구학 계산을 거쳐서 나온 AGV 속도 odometry
                LBox.type.feed.vel.y    = OdomWheelBase.odom.now.vel.y;               // 속도 기구학 계산을 거쳐서 나온 AGV 속도 odometry
                LBox.type.feed.vel.rz   = OdomWheelBase.odom.now.vel.rz;              // 속도 기구학 계산을 거쳐서 나온 AGV 속도 odometry  
            }      
            else
            {
                LBox.type.feed.pos.x = Odom1AxisLidarBase.odom.now.pos.x;
                LBox.type.feed.pos.y = Odom1AxisLidarBase.odom.now.pos.y;
                LBox.type.feed.pos.rz = Odom1AxisLidarBase.odom.now.pos.rz;
                // LBox.type.feed.vel.x    = Odom1AxisLidarBase.odom.now.vel.x;               // 속도 기구학 계산을 거쳐서 나온 AGV 속도 odometry
                // LBox.type.feed.vel.y    = Odom1AxisLidarBase.odom.now.vel.y;               // 속도 기구학 계산을 거쳐서 나온 AGV 속도 odometry
                // LBox.type.feed.vel.rz   = Odom1AxisLidarBase.odom.now.vel.rz;              // 속도 기구학 계산을 거쳐서 나온 AGV 속도 odometry
            }
            // printf("line: %d -- basic fusion mode  --- LBox.type.feed.pos.x: %f , LBox.type.feed.pos.y: %f, LBox.type.feed.pos.rz: %f\n", __LINE__, LBox.type.feed.pos.x, LBox.type.feed.pos.y,LBox.type.feed.pos.rz);
        break;
        case ODOM_KALMAN:
        // 20210515 kalmanfilter odometry 계산
            if((TimeCount_T2_kalman % TIME_50msec) == 0 )
            {
                TimeMain_cur = ros::Time::now().toSec();
                TimeCheckOdomKalmanFilter.time_diff = TimeMain_cur - TimeCheckOdomKalmanFilter.time_pre;
                TimeCheckOdomKalmanFilter.time_pre = TimeMain_cur;
                double OdomKalmanPosOut[3];

                double OdomKalmanPosU[3] = {OdomWheelBase.odom.Relative.pos.x, OdomWheelBase.odom.Relative.pos.y, OdomWheelBase.odom.Relative.pos.rz};
                double OdomKalmanPosZ[3] = {Odom1AxisLidarBase.odom.now.pos.x, Odom1AxisLidarBase.odom.now.pos.x, Odom1AxisLidarBase.odom.now.pos.x};
                OdomKalmanFilter.KamanFilter(OdomKalmanPosOut,OdomKalmanPosU,OdomKalmanPosZ);
                OdomKalmanBase.odom.now.pos.x = OdomKalmanPosOut[0];
                OdomKalmanBase.odom.now.pos.y = OdomKalmanPosOut[1];
                OdomKalmanBase.odom.now.pos.rz = OdomKalmanPosOut[2];
            }
            LBox.type.feed.pos.x    =   OdomKalmanBase.odom.now.pos.x;
            LBox.type.feed.pos.y    =   OdomKalmanBase.odom.now.pos.y;
            LBox.type.feed.pos.rz    =   OdomKalmanBase.odom.now.pos.rz;
            //LBox.type.feed.vel.x    =   OdomKalmanBase.odom.now.vel.x;
            //LBox.type.feed.vel.y    =   OdomKalmanBase.odom.now.vel.y;
            //LBox.type.feed.vel.rz    =   OdomKalmanBase.odom.now.vel.rz;
            // printf("line: %d -- kalmanfilter mode --- LBox.type.feed.pos.x: %f , LBox.type.feed.pos.y: %f, LBox.type.feed.pos.rz: %f\n", __LINE__, LBox.type.feed.pos.x, LBox.type.feed.pos.y,LBox.type.feed.pos.rz);
        break;
        default:
            LBox.type.feed.pos.x    = OdomWheelBase.odom.now.pos.x;                // 기구학 계산을 거쳐서 나온 AGV Odometry Px
            LBox.type.feed.pos.y    = OdomWheelBase.odom.now.pos.y;                // 기구학 계산을 거쳐서 나온 AGV Odometry Py
            LBox.type.feed.pos.rz   = OdomWheelBase.odom.now.pos.rz;               // 기구학 계산을 거쳐서 나온 AGV Odometry Yaw

            LBox.type.feed.vel.x    = OdomWheelBase.odom.now.vel.x;               // 속도 기구학 계산을 거쳐서 나온 AGV 속도 odometry
            LBox.type.feed.vel.y    = OdomWheelBase.odom.now.vel.y;               // 속도 기구학 계산을 거쳐서 나온 AGV 속도 odometry
            LBox.type.feed.vel.rz   = OdomWheelBase.odom.now.vel.rz;              // 속도 기구학 계산을 거쳐서 나온 AGV 속도 odometry
        break;
    }
    //printf("line: %d -- mod:%u --- LBox.type.feed.pos.x: %f , LBox.type.feed.pos.y: %f, LBox.type.feed.pos.rz: %f\n", __LINE__,sensortype, LBox.type.feed.pos.x, LBox.type.feed.pos.y,LBox.type.feed.pos.rz);
}
void FN_AGV_SYSTEM_INPUT_RCM_GET(void)
{
    RcmAgvDataRX = chatIn_;
    switch(RcmAgvDataRX.command)
    {
        case 1 :        // AGV 정지 , JOG 모드로 정지(속도 프로파일 제어)
            LBox.state.AGVControlMode = STOPMODE;             
        break;
        case 2 :        // AGV 긴급정지, JOG 모드로 정지(속도 프로파일 제어)
            LBox.state.AGVControlMode = ESTOPMODE;                 
        break;
        case 3 :        // 위치모드 set
            LBox.state.AGVControlMode = POSMODE;          
        break;
        case 4 :        // 조그모드 set
            // 속도 프로파일 중첩을 피하기 위해 넣어주어야 함. 
            LBox.state.AGVControlMode = JOGMODE;
        break;
        case 5 :        // 조그모드-조이스틱 set
            // 속도 프로파일 중첩을 피하기 위해 넣어주어야 함. 
            LBox.state.AGVControlMode = JOYMODE;
        break;
        case 14 :       // AGV ERROR MODE 진입 (에러가 발생한 경우 RCM(상위제어기)가 에러 처리를 하기 위한 진입)
            LBox.state.AGVControlMode = ERRORMODE;
            break;
        case 15 :        // AGV 오도메트리 초기화 모드 ( 초기화 모드 들어갈때는 AGV 상태가 READY or STOP 상태일때만 )
            if((LBox.state.AgvRunState == READY) || (LBox.state.AgvRunState == STOP))
            {
                LBox.state.AGVControlMode = INITMODE;
            };
            // 일단은 AGV 가 구동 시에는 오도메트리 초기화 모드를 진행하지 않는것으로, 메인 컨트롤 GUI에서 AGV 상태를 표시 해줘야 함.
            // else    // 초기화 작업 버튼 후 AGV가 구동 중일경우에는 먼저 정지를 시키고 대기, 그다음 다시 초기화 작업 버튼을 눌러야 함.
            // {
            //     LBox.state.AGVControlMode = STOPMODE;
            // }                
            break;
        default :
            LBox.state.AGVControlMode = READYMODE;
        break;
    }     
}
void FN_AGV_POSITIONTABLEMODE(void)
{
    if(chatIn_flag)
    {
        if(chatIn_.command==3)
        {
            #if 0
            InnerCmdInput_flag = 1;
            InnerCmdstate = 0;
            #else
            #define MAGIC_VAL_POSTABLE_X    123.0
            #define MAGIC_VAL_POSTABLE_Y    456.0
            #define MAGIC_VAL_POSTABLE_Z    789.0
            if (int(chatIn_.posParam.posState.x) == int(MAGIC_VAL_POSTABLE_X) &&
                int(chatIn_.posParam.posState.y) == int(MAGIC_VAL_POSTABLE_Y) &&
                int(chatIn_.posParam.posState.rz) == int(MAGIC_VAL_POSTABLE_Z)) {
                InnerCmdInput_flag = 1;
                InnerCmdstate = 0;
            }
            #endif
        }
        else    // 포지션 테이블 명령이 아닐경우 초기화 하는 루틴임.
        {
            InnerCmdInput_flag = 0;
            InnerCmd_flag = 0;
            InnerCmdstate = 0;
        }    
    }
    if(InnerCmdInput_flag)
    {
        if((LBox.state.AgvRunState == READY) || (LBox.state.AgvRunState == STOP))          // AGV 현재 상태가 ready 상태 일때만 동작되도록. 위치 프로파일 중복 방지
        {
            InnerCmd_flag = 1;
        }
        else InnerCmd_flag = 0;
    }
    if(InnerCmd_flag)
    {
        switch(InnerCmdstate)
        {
            case 0:
                chatIn_flag = 0xFF;
                chatIn_.posParam.posState.x = 1.0;          //m                 
                chatIn_.posParam.posState.y = 0.0;          // m                  
                chatIn_.posParam.posState.rz = 0.0;         // deg
                InnerCmdstate = 1;
            break;
            case 1:
                chatIn_flag = 0xFF;
                chatIn_.posParam.posState.x = 0.0;          //mm                   
                chatIn_.posParam.posState.y = 1.0;          // mm                  
                chatIn_.posParam.posState.rz = 0.0;         // deg
                InnerCmdstate = 2;
            break;
            case 2:
                chatIn_flag = 0xFF;
                chatIn_.posParam.posState.x = -1.0;          //mm                   
                chatIn_.posParam.posState.y = 0.0;          // mm                  
                chatIn_.posParam.posState.rz = 0.0;         // deg
                InnerCmdstate = 3;
            break;
            case 3:
                chatIn_flag = 0xFF;
                chatIn_.posParam.posState.x = 0.0;          //mm                   
                chatIn_.posParam.posState.y = -1.0;          // mm                  
                chatIn_.posParam.posState.rz = 0.0;         // deg
                InnerCmdstate = 0;
            break;
            case 4:
                InnerCmdstate = 0;
                InnerCmd_flag = 0;
                InnerCmdInput_flag = 0;
            break;
            default:
            break;
        }
    }
}
void FN_AGV_SYSTEM_INPUT_RCM_PROCESS(void)
{
    if((RcmAgvDataRX.posParam.velParams.x.vel < 0.001) || (RcmAgvDataRX.posParam.velParams.y.vel < 0.001) || (RcmAgvDataRX.posParam.velParams.rz.vel < 0.0017) ||
       (RcmAgvDataRX.posParam.velParams.x.acc < 0.001) || (RcmAgvDataRX.posParam.velParams.y.acc < 0.001) || (RcmAgvDataRX.posParam.velParams.rz.acc < 0.0017) ||
       (RcmAgvDataRX.jogParam.velParams.x.vel < 0.001) || (RcmAgvDataRX.jogParam.velParams.y.vel < 0.001) || (RcmAgvDataRX.jogParam.velParams.rz.vel < 0.0017) ||
       (RcmAgvDataRX.jogParam.velParams.x.acc < 0.001) || (RcmAgvDataRX.jogParam.velParams.y.acc < 0.001) || (RcmAgvDataRX.jogParam.velParams.rz.acc < 0.0017))
    {
        //20210607 입력값이 이상하게 들어올 경우 에러처리
    }
    switch(LBox.state.AGVControlMode)
    {
        case STOPMODE:   // AGV STOP 모드
            RcmAgvDataRX.jogParam.velParams.x.vel  =  0.0;   
            RcmAgvDataRX.jogParam.velParams.y.vel  =  0.0;   
            RcmAgvDataRX.jogParam.velParams.rz.vel =  0.0;   
            break;
        case ESTOPMODE:   // AGV EMERGENCY STOP 모드
            RcmAgvDataRX.jogParam.velParams.x.vel =  0.0;   
            RcmAgvDataRX.jogParam.velParams.y.vel =  0.0;   
            RcmAgvDataRX.jogParam.velParams.rz.vel =  0.0;   
            break;
        case JOGMODE:  // 조그 모드(속도 모드) - open loop control
            if(RcmAgvDataRX.jogParam.jogInfo.front)         {RcmAgvDataRX.jogParam.velParams.x.vel =  RcmAgvDataRX.jogParam.velParams.x.vel;}
            else if(RcmAgvDataRX.jogParam.jogInfo.back)     {RcmAgvDataRX.jogParam.velParams.x.vel =  -RcmAgvDataRX.jogParam.velParams.x.vel;}
            else{
                RcmAgvDataRX.jogParam.velParams.x.vel =  0.0;
            }
            if(RcmAgvDataRX.jogParam.jogInfo.left)          {RcmAgvDataRX.jogParam.velParams.y.vel =  RcmAgvDataRX.jogParam.velParams.y.vel;}
            else if(RcmAgvDataRX.jogParam.jogInfo.right)    {RcmAgvDataRX.jogParam.velParams.y.vel =  -RcmAgvDataRX.jogParam.velParams.y.vel;}
            else{
                RcmAgvDataRX.jogParam.velParams.y.vel =  0.0;
            }
            if(RcmAgvDataRX.jogParam.jogInfo.ccw)           {RcmAgvDataRX.jogParam.velParams.rz.vel =  RcmAgvDataRX.jogParam.velParams.rz.vel;}
            else if(RcmAgvDataRX.jogParam.jogInfo.cw)       {RcmAgvDataRX.jogParam.velParams.rz.vel =  -RcmAgvDataRX.jogParam.velParams.rz.vel;}
            else{
                RcmAgvDataRX.jogParam.velParams.rz.vel =  0.0;
            }         
        //printf("AGV type.comd.vel.xel: %f \n", RcmAgvDataRX.jogParam.velParams.x.vel);
            break;
        case JOYMODE :        // 조그모드-조이스틱 set
            // 조이스틱 모드는 명령값 자체를 얻기에 여기서는 할게 없음.        
        //printf("AGV type.comd.vel.xel: %f \n", RcmAgvDataRX.jogParam.velParams.x.vel);
            break;
        case POSMODE:  // 위치 모드            - closed loop control
            break; 
        case PTABLEMODE :       // 위치 포지션 테이블 모드 ( 저장된 포지션을 이동하는 모드)  임시 모드 
            break;
        case INITMODE:   // 초기화 모드
            // switch-case 안에 변수 생성하지 말 것(현재 임시용)
            // static mecanum::MotorPosition motorPosition1;
            // motorPosition1.request.fl = 0;
            // motorPosition1.request.fr = 1;
            // motorPosition1.request.bl = 2;
            // motorPosition1.request.br = 3;
            // if (setMotorPosition(motorPosition1)) {
            //     printf("setPosition  : fl:%10d, fr:%10d, bl:%10d, br:%10d\n",
            //         motorPosition1.request.fl, motorPosition1.request.fr,
            //         motorPosition1.request.bl, motorPosition1.request.br);
            //     printf("lastPosition : fl:%10d, fr:%10d, bl:%10d, br:%10d\n",
            //         motorPosition1.response.fl, motorPosition1.response.fr,
            //         motorPosition1.response.bl, motorPosition1.response.br);
            // } else {
            //     printf("setMotorPosition FAILED\n");
            // }
            break;
        case READYMODE:   // AGV READY 모드
            break;
        case ERRORMODE:     // AGV ERROR처리 모드
            // switch-case 안에 변수 생성하지 말 것(현재 임시용)
            LBox.state.ErrorCode = OFF;
            LBox.state.ErrorState = OFF;
            LBox.watchdog.WatchDogPositonERRStartFalg = OFF;
            // static mecanum::MotorPosition motorPosition2;
            // if (getMotorPosition(motorPosition2)) {
            //     printf("getPosition  : fl:%10d, fr:%10d, bl:%10d, br:%10d\n",
            //         motorPosition2.response.fl, motorPosition2.response.fr,
            //         motorPosition2.response.bl, motorPosition2.response.br);
            // } else {
            //     printf("getMotorPosition FAILED\n");
            // }
            break;
        default:
            // PID 변수 리셋
            break; 
    }
}
void FN_AGV_TRAJECTORY_PROFILE_INIT(geometry_msgs::Twist *twist)        // 현재 twist는 안씀.
{
    switch(LBox.state.AGVControlMode)
    {
        case STOPMODE:   // AGV STOP 모드
        // AGV STOP 모드일 때는 속도 프로파일을 우선적으로 적용.
        // i) AGV 이전 모드가 JOGMODE , POSMODE 일경우
            LBox.state.AgvRunState = RUN;               // RUN 모드이고, 추후 속도 프로파일이 종료가 되면 RUN 을 AGVStop으로 변경해야 함.
            AGVPx.StartFlag = PROFILEInit;
            AGVPy.StartFlag = PROFILEInit;
            AGVYaw.StartFlag = PROFILEInit;
            // 속도값이 제대로 들어오기에 센서 입력 부분에서  LBox.type.feed.vel 값을 수신함. 여기서는 할 필요가 없음.
            //LBox.type.feed.vel.x  = twist->linear.x;    LBox.type.feed.vel.y    = twist->linear.y;  LBox.type.feed.vel.rz   = twist->angular.z;
            //LBox.type.tcomd.vel.x = RcmAgvDataRX.jogParam.velParams.x.vel; LBox.type.tcomd.vel.y   = RcmAgvDataRX.jogParam.velParams.y.vel;    LBox.type.tcomd.vel.rz  = RcmAgvDataRX.jogParam.velParams.z.vel*D2R;
            LBox.type.tcomd.vel.x   = 0.0;                          // stop 모드는 무조건 서야 하기에 수정
            LBox.type.tcomd.vel.y   = 0.0;
            LBox.type.tcomd.vel.rz  = 0.0*D2R;
            LBox.type.tcomd.acc.x   = LBox.type.feed.vel.x;         // 현재는 가속도 값을 현재속도값으로 계산 추후에 수정
            LBox.type.tcomd.acc.y   = LBox.type.feed.vel.y;
            LBox.type.tcomd.acc.rz  = LBox.type.feed.vel.rz;
            printf("recipelab_main.cpp line %d  현재 AGV 속도 (x,y,rz) = (%f , %f , %f)   \n",__LINE__,LBox.type.feed.vel.x,LBox.type.feed.vel.y,LBox.type.feed.vel.rz );
        // ii) AGV 이전 모드가 POSMODE 일경우
            break;
        case ESTOPMODE:   // AGV EMERGENCY STOP 모드
            LBox.state.AgvRunState = RUN;               // RUN 모드이고, 추후 속도 프로파일이 종료가 되면 RUN 을 AGVStop으로 변경해야 함.
            AGVPx.StartFlag = PROFILEInit;
            AGVPy.StartFlag = PROFILEInit;
            AGVYaw.StartFlag = PROFILEInit;
            LBox.type.tcomd.vel.x   = 0.0;                          // Estop 모드는 무조건 서야 하기에 수정
            LBox.type.tcomd.vel.y   = 0.0;
            LBox.type.tcomd.vel.rz  = 0.0*D2R;
            LBox.type.tcomd.acc.x   = LBox.type.feed.vel.x*2.0;     // 현재는 가속도 값을 현재속도값*2로 단순히 함. 추후 변수 및 계산값으로 수정
            LBox.type.tcomd.acc.y   = LBox.type.feed.vel.y*2.0;
            LBox.type.tcomd.acc.rz  = LBox.type.feed.vel.rz*2.0;
            printf("recipelab_main.cpp line %d  현재 AGV 속도 (x,y,rz) = (%f , %f , %f)   \n",__LINE__,LBox.type.feed.vel.x,LBox.type.feed.vel.y,LBox.type.feed.vel.rz );
            break;
        case JOGMODE:  // 조그 모드(속도 모드) - open loop control
            //chatIn_.jogParam.jogInfo.ccw
            LBox.state.AgvRunState = RUN;       // RUN 모드이고, 추후 속도 프로파일이 종료가 되면 RUN 을 AGVStop으로 변경해야 함.
            AGVPx.StartFlag = PROFILEInit;
            AGVPy.StartFlag = PROFILEInit;
            AGVYaw.StartFlag = PROFILEInit;
            //20210515수정             
            LBox.type.tcomd.vel.x   = RcmAgvDataRX.jogParam.velParams.x.vel;
            LBox.type.tcomd.vel.y   = RcmAgvDataRX.jogParam.velParams.y.vel;
            LBox.type.tcomd.vel.rz  = RcmAgvDataRX.jogParam.velParams.rz.vel*D2R;
            LBox.type.tcomd.acc.x   = RcmAgvDataRX.jogParam.velParams.x.acc;
            LBox.type.tcomd.acc.y   = RcmAgvDataRX.jogParam.velParams.y.acc;
            LBox.type.tcomd.acc.rz  = RcmAgvDataRX.jogParam.velParams.rz.acc*D2R;           
            // if(bufcount11<20) printf("line: %d --  %u --- AGVPx.SlopUp: %d , AGVPx.SlopDown:%d , LBox.type.comd.vel.x : %f\n", __LINE__,bufcount11, AGVPx.SlopUp, AGVPx.SlopDown,LBox.type.comd.vel.x);
            break;
        case JOYMODE:  // 조그모드-조이스틱 - open loop control
            //chatIn_.jogParam.jogInfo.ccw
            LBox.state.AgvRunState = RUN;       // RUN 모드이고, 추후 속도 프로파일이 종료가 되면 RUN 을 AGVStop으로 변경해야 함.
            AGVPx.StartFlag = PROFILEInit;
            AGVPy.StartFlag = PROFILEInit;
            AGVYaw.StartFlag = PROFILEInit;
            //20210515수정             
            LBox.type.tcomd.vel.x   = RcmAgvDataRX.jogParam.velParams.x.vel;
            LBox.type.tcomd.vel.y   = RcmAgvDataRX.jogParam.velParams.y.vel;
            LBox.type.tcomd.vel.rz  = RcmAgvDataRX.jogParam.velParams.rz.vel*D2R;
            LBox.type.tcomd.acc.x   = RcmAgvDataRX.jogParam.velParams.x.acc;
            LBox.type.tcomd.acc.y   = RcmAgvDataRX.jogParam.velParams.y.acc;
            LBox.type.tcomd.acc.rz  = RcmAgvDataRX.jogParam.velParams.rz.acc*D2R;           
            // if(bufcount11<20) printf("line: %d --  %u --- AGVPx.SlopUp: %d , AGVPx.SlopDown:%d , LBox.type.comd.vel.x : %f\n", __LINE__,bufcount11, AGVPx.SlopUp, AGVPx.SlopDown,LBox.type.comd.vel.x);
            break;
        case POSMODE:  // 위치 모드            - closed loop control
            if((LBox.state.AgvRunState == READY) || (LBox.state.AgvRunState == STOP))          // AGV 현재 상태가 ready 상태 일때만 동작되도록. 위치 프로파일 중복 방지
            {
                LBox.state.AgvRunState = RUN;
                AGVPx.StartFlag     = PROFILEInit;
                AGVPy.StartFlag     = PROFILEInit;
                AGVYaw.StartFlag    = PROFILEInit;
                // 일단은 Feedback 위치는 라이다에서 들어오는것을 계산하여 사용
                //추후  기구학 오도메트리와 라이다 오도메트리 IMU 센서 오도메트리를 퓨전하여 최종 Feedback 위치 계산할 예정임.
                // 실질적으로는 now pos임.  
                LBox.type.now.pos.x     = LBox.type.feed.pos.x;                             
                LBox.type.now.pos.y     = LBox.type.feed.pos.y;                             
                LBox.type.now.pos.rz    = LBox.type.feed.pos.rz;                           
                LBox.type.tcomd.pos.x   = RcmAgvDataRX.posParam.posState.x + LBox.type.now.pos.x;                   
                LBox.type.tcomd.pos.y   = RcmAgvDataRX.posParam.posState.y + LBox.type.now.pos.y;                    
                LBox.type.tcomd.pos.rz  = RcmAgvDataRX.posParam.posState.rz*D2R + LBox.type.now.pos.rz;
                LBox.type.tcomd.vel.x   = RcmAgvDataRX.posParam.velParams.x.vel;
                LBox.type.tcomd.vel.y   = RcmAgvDataRX.posParam.velParams.y.vel;
                LBox.type.tcomd.vel.rz  = RcmAgvDataRX.posParam.velParams.rz.vel*D2R;
                LBox.type.tcomd.acc.x   = RcmAgvDataRX.posParam.velParams.x.acc;
                LBox.type.tcomd.acc.y   = RcmAgvDataRX.posParam.velParams.y.acc;
                LBox.type.tcomd.acc.rz  = RcmAgvDataRX.posParam.velParams.rz.acc*D2R;
                LBox.control.inpos.x    = OFF;
                LBox.control.inpos.y    = OFF;
                LBox.control.inpos.rz   = OFF;
                LBox.control.inpos.all   = OFF;
                printf("recipelab_main.cpp line %d  --> POSMODE 정상적인 위치 프로파일 초기화 \n", __LINE__ );    
            }
            else
            {
                printf("recipelab_main.cpp line %d --> !!! POSMODE AGV가 READY or STOP 상태가 아닐경우\n", __LINE__ ); 
                printf("recipelab_main.cpp line %d  AgvRunState = %d \n",__LINE__,LBox.state.AgvRunState );
                // STOP mode 로 변경한다.
                LBox.state.AGVControlMode = STOPMODE;
                LBox.state.AgvRunState = RUN;               // RUN 모드이고, 추후 속도 프로파일이 종료가 되면 RUN 을 AGVStop으로 변경해야 함.
                AGVPx.StartFlag = PROFILEInit;
                AGVPy.StartFlag = PROFILEInit;
                AGVYaw.StartFlag = PROFILEInit;
                // 속도값이 제대로 들어오기에 센서 입력 부분에서  LBox.type.feed.vel 값을 수신함. 여기서는 할 필요가 없음.
                //LBox.type.feed.vel.x  = twist->linear.x;    LBox.type.feed.vel.y    = twist->linear.y;  LBox.type.feed.vel.rz   = twist->angular.z;
                //LBox.type.tcomd.vel.x = RcmAgvDataRX.jogParam.velParams.x.vel; LBox.type.tcomd.vel.y   = RcmAgvDataRX.jogParam.velParams.y.vel;    LBox.type.tcomd.vel.rz  = RcmAgvDataRX.jogParam.velParams.z.vel*D2R;
                LBox.type.tcomd.vel.x   = 0.0;                          // stop 모드는 무조건 서야 하기에 수정
                LBox.type.tcomd.vel.y   = 0.0;
                LBox.type.tcomd.vel.rz  = 0.0*D2R;
                LBox.type.tcomd.acc.x   = LBox.type.feed.vel.x;         // 현재는 가속도 값을 현재속도값으로 계산 추후에 수정
                LBox.type.tcomd.acc.y   = LBox.type.feed.vel.y;
                LBox.type.tcomd.acc.rz  = LBox.type.feed.vel.rz;
                printf("recipelab_main.cpp line %d  현재 AGV 속도 (x,y,rz) = (%f , %f , %f)   \n",__LINE__,LBox.type.feed.vel.x,LBox.type.feed.vel.y,LBox.type.feed.vel.rz );
            }
            break; 
        case INITMODE:   // 초기화 모드
            // AGV 위치 초기화 루틴.
            // AGVOdomPoseInit()
            // 센서 콜백을 통한 센서 데이터를 갱신을 다시 해야 할 것인가? 아님 기존 데이터를 중심으로 할 것인가?
            // 이전 init 값을 feed 값으로 갱신
            //20221027일 임시 됨.
            OdomImuBase.odom.init.pos.rz        = OdomImuBase.odom.feed_now.pos.rz;

            OdomWheelBase.odom.init.pos.x       = OdomWheelBase.odom.feed.pos.x;
            OdomWheelBase.odom.init.pos.y       = OdomWheelBase.odom.feed.pos.y;
            OdomWheelBase.odom.init.pos.rz      = OdomWheelBase.odom.feed.pos.rz;
            Odom1AxisLidarBase.odom.init.pos.x  = Odom1AxisLidarBase.odom.feed.pos.x;
            Odom1AxisLidarBase.odom.init.pos.y  = Odom1AxisLidarBase.odom.feed.pos.y;
            Odom1AxisLidarBase.odom.init.pos.rz = Odom1AxisLidarBase.odom.feed.pos.rz;
            OdomKalmanFilter.FirstRun           = 1;    // 칼만필터 초기화
            
            OdomWheelBase.odom.now.pos.x        = 0.0 ;
            OdomWheelBase.odom.now.pos.y        = 0.0 ;
            OdomWheelBase.odom.now.pos.rz       = 0.0 ;

            Odom1AxisLidarBase.odom.now.pos.x   = 0.0 ;
            Odom1AxisLidarBase.odom.now.pos.y   = 0.0 ;
            Odom1AxisLidarBase.odom.now.pos.rz  = 0.0 ;

            OdomAmclBase.odom.now.pos.x         = 0.0 ;
            OdomAmclBase.odom.now.pos.y         = 0.0 ;
            OdomAmclBase.odom.now.pos.rz        = 0.0 ;
            //LBox.state.AGVControlMode = READYMODE;
            LBox.state.AgvRunState = READY;              // INITMODE 실행조건은 AGV LBox.state.AgvRunState = STOP or READY 상태임으로 여기서 정의할 필요없다.
            break;
        case READYMODE:   // AGV READY 모드
            LBox.state.AgvRunState = READY;
            break;
        case ERRORMODE:     // 속도프로파일모드에서는 에러모드가 필요가 없음.
            LBox.state.AgvRunState = READY;
            break;
        case PTABLEMODE :       // 위치 포지션 테이블 모드 ( 저장된 포지션을 이동하는 모드)  임시 모드 
            break;
        default:
            // PID 변수 리셋
            break; 
    }
    // if(bufcount11<100) printf("line: %d --  %u --- AGVControlMode: %X , LBox.type.comd.vel.x : %f\n", __LINE__,bufcount11, LBox.state.AGVControlMode, LBox.type.comd.vel.x);
}
// -3.2 AGV 모드별 입력 처리 2(주행 알고리즘 및 속도프로파일, PID 제어)
// -3.2.1 경로 계획 속도 및 위치 프로파일 연산 
void FN_AGV_TRAJECTORY_PROFILE(void)
{
    switch(LBox.state.AGVControlMode)
    {
        case STOPMODE:   // AGV STOP 모드
        // AGV STOP 모드일 때는 속도 프로파일을 우선적으로 적용
            LBox.type.comd.vel.x     = VelProfile(LBox.type.feed.vel.x,LBox.type.tcomd.vel.x,LBox.type.tcomd.acc.x,&AGVPx);
            LBox.type.comd.vel.y     = VelProfile(LBox.type.feed.vel.y,LBox.type.tcomd.vel.y,LBox.type.tcomd.acc.y,&AGVPy);
            LBox.type.comd.vel.rz    = VelProfile(LBox.type.feed.vel.rz,LBox.type.tcomd.vel.rz,LBox.type.tcomd.acc.rz,&AGVYaw);
            break;
        case ESTOPMODE:   // AGV EMERGENCY STOP 모드
        // AGV EMERGENCY STOP 모드일 때는 일단 속도 프로파일을 우선적으로 적용
            LBox.type.comd.vel.x     = VelProfile(LBox.type.feed.vel.x,LBox.type.tcomd.vel.x,LBox.type.tcomd.acc.x,&AGVPx);
            LBox.type.comd.vel.y     = VelProfile(LBox.type.feed.vel.y,LBox.type.tcomd.vel.y,LBox.type.tcomd.acc.y,&AGVPy);
            LBox.type.comd.vel.rz    = VelProfile(LBox.type.feed.vel.rz,LBox.type.tcomd.vel.rz,LBox.type.tcomd.acc.rz,&AGVYaw);
            break;
        case JOGMODE:  // 조그 모드(속도 모드) - open loop control
            LBox.type.comd.vel.x     = VelProfile(LBox.type.feed.vel.x,LBox.type.tcomd.vel.x,LBox.type.tcomd.acc.x,&AGVPx);
            LBox.type.comd.vel.y     = VelProfile(LBox.type.feed.vel.y,LBox.type.tcomd.vel.y,LBox.type.tcomd.acc.y,&AGVPy);
            LBox.type.comd.vel.rz    = VelProfile(LBox.type.feed.vel.rz,LBox.type.tcomd.vel.rz,LBox.type.tcomd.acc.rz,&AGVYaw);
            //printf("AGVPx.StartFlag: %X \n", AGVPx.StartFlag);
            //printf("LBox.type.feed.vel.x: %f , LBox.type.comd.vel.x : %f\n", LBox.type.feed.vel.x , LBox.type.comd.vel.x);
            break;
        case JOYMODE:  // 조그모드-조이스틱 - open loop control                 
            LBox.type.comd.vel.x     = VelProfile(LBox.type.feed.vel.x,LBox.type.tcomd.vel.x,LBox.type.tcomd.acc.x,&AGVPx);
            LBox.type.comd.vel.y     = VelProfile(LBox.type.feed.vel.y,LBox.type.tcomd.vel.y,LBox.type.tcomd.acc.y,&AGVPy);
            LBox.type.comd.vel.rz    = VelProfile(LBox.type.feed.vel.rz,LBox.type.tcomd.vel.rz,LBox.type.tcomd.acc.rz,&AGVYaw);
            break;
        case POSMODE:  // 위치 모드            - closed loop control
            //if( AGVPx.StartFlag == 0xC0 && AGVPx.StartFlag == 0xC0 && AGVYaw.StartFlag == 0xC0 )
            //{
                LBox.type.comd.pos.x     = PosProfile(LBox.type.now.pos.x,LBox.type.tcomd.pos.x,LBox.type.tcomd.vel.x,LBox.type.tcomd.acc.x,&AGVPx);
                LBox.type.comd.pos.y     = PosProfile(LBox.type.now.pos.y,LBox.type.tcomd.pos.y,LBox.type.tcomd.vel.y,LBox.type.tcomd.acc.y,&AGVPy);
                LBox.type.comd.pos.rz    = PosProfile(LBox.type.now.pos.rz,LBox.type.tcomd.pos.rz,LBox.type.tcomd.vel.rz,LBox.type.tcomd.acc.rz,&AGVYaw);
            //}
            break;
        case INITMODE:   // 초기화 모드
            break;
        case READYMODE:   // AGV READY 모드
            break;
        case ERRORMODE:     // 속도프로파일모드에서는 에러모드가 필요가 없음.
            break;
        default:
        // 프로파일 변수 초기화 할 필요 없음.
        break; 
    }
}       
// -3.2.2 AGV PID 제어
void FN_AGV_PID_CONTROL(void)
{
    switch(LBox.state.AGVControlMode)
    {
        case POSMODE:  // 위치 모드            - closed loop control
            LBox.control.pid.x.Ref = LBox.type.comd.pos.x;     LBox.control.pid.x.Fdb = LBox.type.feed.pos.x;       LBox.control.pid.x.pidfn();
            LBox.control.pid.y.Ref = LBox.type.comd.pos.y;     LBox.control.pid.y.Fdb = LBox.type.feed.pos.y;       LBox.control.pid.y.pidfn();           
            LBox.control.pid.rz.Ref = LBox.type.comd.pos.rz;   LBox.control.pid.rz.Fdb = LBox.type.feed.pos.rz;     LBox.control.pid.rz.pidfn(); 
            break;
            // 위치모드를 제외한 모드에서는 PID를 쓰지 않음.
        case STOPMODE:   // AGV STOP 모드
        case ESTOPMODE:   // AGV EMERGENCY STOP 모드
        case JOGMODE:  // 조그 모드(속도 모드) - open loop control
        case JOYMODE:  // 조그모드-조이스틱 - open loop control
        case INITMODE:   // 초기화 모드
        case READYMODE:   // AGV READY 모드
        case ERRORMODE:     // 속도프로파일모드에서는 에러모드가 필요가 없음.
        default:
            // PID 변수 리셋
            AGVPIDValReset();
            //if(bufcount11<20){
            //bufcount11++;
            // printf("line: %d --  %u --- AGVPx.SlopUp: %d , AGVPx.SlopDown:%d , LBox.type.comd.vel.x : %f\n", __LINE__,bufcount11, AGVPx.SlopUp, AGVPx.SlopDown,LBox.type.comd.vel.x);
            //}
            break; 
    }
    // if(bufcount11<100) printf("line: %d --  %u --- AGVControlMode: %X , LBox.type.comd.vel.x : %f\n", __LINE__,bufcount11, LBox.state.AGVControlMode, LBox.type.comd.vel.x);
}
// -3.2.3 AGV 최종 출력값 정의
// 여기에서 프로파일 상태를 보고 출력값을 내보냄.
//void FN_AGV_SYSTEM_CONTROL_OUT(geometry_msgs::Twist *twist)
// 2022년4월2일 수정 :  twist 값을 여기서 갱신하는 것이 아니고 void FN_AGV_SYSTEM_ERROR_DOACTING 에서 최종 갱신으로 변경
//                  :  에러가 발생했을 때 이전 출력값을 유지 하기 위해서 -->  에러가 발생하고 twist 값을 이전 값으로 유지하기 위해서 버퍼 개념으로 
//                        LBox.type.twist.linear 값을 추가함.

void FN_AGV_SYSTEM_CONTROL_OUT(void)
{
    switch(LBox.state.AGVControlMode)
    {
        case STOPMODE:   // AGV STOP 모드
            LBox.type.twist.linear.x  = LBox.type.comd.vel.x;
            LBox.type.twist.linear.y  = LBox.type.comd.vel.y;    
            LBox.type.twist.angular.z = LBox.type.comd.vel.rz;
            if((AGVPx.StartFlag == PROFILEStop) && (AGVPy.StartFlag == PROFILEStop) && (AGVYaw.StartFlag == PROFILEStop))
            {
                LBox.type.twist.linear.x  = 0.0;
                LBox.type.twist.linear.y  = 0.0;    
                LBox.type.twist.angular.z = 0.0;
                LBox.state.AgvRunState = STOP;
            }
            break;
        case ESTOPMODE:   // AGV EMERGENCY STOP 모드
            LBox.type.twist.linear.x  = LBox.type.comd.vel.x;
            LBox.type.twist.linear.y  = LBox.type.comd.vel.y;    
            LBox.type.twist.angular.z = LBox.type.comd.vel.rz;
            if((AGVPx.StartFlag == PROFILEStop) && (AGVPy.StartFlag == PROFILEStop) && (AGVYaw.StartFlag == PROFILEStop))
            {
                LBox.type.twist.linear.x  = 0.0;
                LBox.type.twist.linear.y  = 0.0;    
                LBox.type.twist.angular.z = 0.0;
                LBox.state.AgvRunState = STOP;
            }
            break;
        case JOGMODE:  // 조그 모드(속도 모드)      - open loop control
            //printf("LBox.type.comd.vel.x: %f \n", LBox.type.comd.vel.x);
            LBox.type.twist.linear.x  = LBox.type.comd.vel.x;
            LBox.type.twist.linear.y  = LBox.type.comd.vel.y;    
            LBox.type.twist.angular.z = LBox.type.comd.vel.rz;
            if((AGVPx.StartFlag == PROFILEStop) && (AGVPy.StartFlag == PROFILEStop) && (AGVYaw.StartFlag == PROFILEStop))
            {
                LBox.type.twist.linear.x  = 0.0;
                LBox.type.twist.linear.y  = 0.0;    
                LBox.type.twist.angular.z = 0.0;
                LBox.state.AgvRunState = STOP;
            }
            break;
        case JOYMODE:  // 조그모드-조이스틱 - open loop control
            LBox.type.twist.linear.x  = LBox.type.comd.vel.x;
            LBox.type.twist.linear.y  = LBox.type.comd.vel.y;    
            LBox.type.twist.angular.z = LBox.type.comd.vel.rz;
            if((AGVPx.StartFlag == PROFILEStop) && (AGVPy.StartFlag == PROFILEStop) && (AGVYaw.StartFlag == PROFILEStop))
            {
                LBox.type.twist.linear.x  = 0.0;
                LBox.type.twist.linear.y  = 0.0;    
                LBox.type.twist.angular.z = 0.0;
                LBox.state.AgvRunState = STOP;
            }
            break;
        case POSMODE:  // 위치 모드(출력은 속도)    - closed loop control
            // (m) 위치 에러값이 최소 에러값보다 작을 경우는 PID 출력값은 0
/*
            double x_err1;
            x_err1   = abs(LBox.control.pid.x.Ref -  LBox.control.pid.x.Fdb); 
            double y_err1;
            y_err1   = abs(LBox.control.pid.y.Ref -  LBox.control.pid.y.Fdb);   
            double rz_err1;
            rz_err1  = abs(LBox.control.pid.rz.Ref -  LBox.control.pid.rz.Fdb);
            // printf("line: %d --  %u --- AGVControlMode: %X , x_err1 : %f , y_err1 : %f , rz_err1 : %f\n", __LINE__,bufcount11, LBox.state.AGVControlMode, x_err1,y_err1,rz_err1);
*/
            //if(abs(LBox.control.pid.x.Ref -  LBox.control.pid.x.Fdb) < 0.005)        LBox.control.pid.x.Out = 0.0;
            //if(abs(LBox.control.pid.y.Ref -  LBox.control.pid.y.Fdb) < 0.005)        LBox.control.pid.y.Out = 0.0;
            //if(abs(LBox.control.pid.rz.Ref -  LBox.control.pid.rz.Fdb) < 0.0017)     LBox.control.pid.rz.Out = 0.0;
            if((AGVPx.StartFlag == PROFILEStop) && (AGVPy.StartFlag == PROFILEStop) && (AGVYaw.StartFlag == PROFILEStop))
            {
                if((abs(LBox.control.pid.x.Err) <0.01) && (abs(LBox.control.pid.y.Err)<0.01) && (abs(LBox.control.pid.rz.Err)<0.0017))
                {
                    LBox.control.pid.x.Out  = 0.0;
                    LBox.control.pid.y.Out  = 0.0;
                    LBox.control.pid.rz.Out = 0.0;
                    LBox.state.AgvRunState  = STOP;
                    LBox.control.inpos.x    = ON;
                    LBox.control.inpos.y    = ON;
                    LBox.control.inpos.rz   = ON;
                    LBox.control.inpos.all  = ON;

                    // 20221022 일단 READYMODE 로 변환 위치 피드백 값이 튀어서 일단 이동한다.
                    // 20210603 일단 POSMODE 로 계속 유지하고 PID도 계속 유지한다.
                    LBox.state.AGVControlMode = READYMODE;      // posmode 에서 PID 제어를 마치고 Readymode
                }
                else
                {
                    //20220308 위치제어를 계속 하는 조건으로 진행. 인포지션 상태도 정의 
                    LBox.state.AgvRunState = RUNNING;
                    LBox.control.inpos.x    = OFF;
                    LBox.control.inpos.y    = OFF;
                    LBox.control.inpos.rz   = OFF;
                    LBox.control.inpos.all  = OFF;
                }
            }
            LBox.type.twist.linear.x  = LBox.control.pid.x.Out;
            LBox.type.twist.linear.y  = LBox.control.pid.y.Out;    
            LBox.type.twist.angular.z = LBox.control.pid.rz.Out;
            break;
        case INITMODE:   // 초기화 모드
            LBox.state.AgvRunState = READY;
            LBox.type.twist.linear.x  = 0.0;
            LBox.type.twist.linear.y  = 0.0;    
            LBox.type.twist.angular.z = 0.0;
            break;
        case READYMODE:   // AGV READY 모드
            LBox.state.AgvRunState = READY;
            LBox.type.twist.linear.x  = 0.0;
            LBox.type.twist.linear.y  = 0.0;    
            LBox.type.twist.angular.z = 0.0;
            break;
        case ERRORMODE:   // AGV ERROR 모드 임으로 
            LBox.state.AgvRunState = READY;
            LBox.type.twist.linear.x  = 0.0;
            LBox.type.twist.linear.y  = 0.0;    
            LBox.type.twist.angular.z = 0.0;
            break;
        default:
            LBox.state.AgvRunState = READY;
            LBox.type.twist.linear.x  = 0.0;
            LBox.type.twist.linear.y  = 0.0;    
            LBox.type.twist.angular.z = 0.0;
            break; 
    }
    // if(bufcount11<200){
    //     bufcount11++;
    //     if(bufcount11<100) printf("line: %d --  %u --- AGVControlMode: %X , LBox.type.comd.vel.x : %f , twist->linear.x : %f\n", __LINE__,bufcount11, LBox.state.AGVControlMode, LBox.type.comd.vel.x,twist.linear.x);
    // }
}
// 4. AGV 내부 상태 감시를 통한 FailSafe 구현
// -4.1 AGV WATCHDOG 실행
void FN_AGV_SYSTEM_WATCHDOG(void)
{
//    WatchDog_Battery_VDC_UVP(&LBox);        // 항상 리얼타임으로 감시
    WatchDog_Positon_ERR(&LBox);

    if (paramEnVirtualWall) {
        WatchDog_VitualWall_Detect_Front(&LBox);        // 항상 리얼타임으로 감시
        WatchDog_VitualWall_Detect_Rear(&LBox);        // 항상 리얼타임으로 감시
        WatchDog_VitualWall_Detect_RightF(&LBox);
        WatchDog_VitualWall_Detect_RightR(&LBox);
    }

}											
// -4.2 AGV 에러에 대한 처리
// 에러에 대한 처리를 할 경우 LBox.state.ErrorStateFlag 이변수로 입력값 수신을 결정함.
void FN_AGV_SYSTEM_ERROR_DOACTING(geometry_msgs::Twist *twist)
{
    static __uint16_t SystemErrorDoActingStartFlag = OFF;
    // 2022년4월2일 twist 출력값 이전데이타를 저장하기 위한 변수 추가. static으로 설정. 지속적으로 값을 갱신해야 함으로 . 이 함수내부에서만 사용함으로 
    static double twistlinear_old_x = 0.0;
    static double twistlinear_old_y = 0.0;
    static double twistangular_old_z = 0.0;
    double twistlinear_now_x    = LBox.type.twist.linear.x;
    double twistlinear_now_y    = LBox.type.twist.linear.y;
    double twistangular_now_z   = LBox.type.twist.angular.z;
    double twistlinear_x;
    double twistlinear_y;
    double twistangular_z;

    if(!SystemErrorDoActingStartFlag)   // 함수 처음 실행될 때 초기화 해주는 루틴
    {
        SystemErrorDoActingStartFlag = ON;
        // 함수 처음 실행될 때 twist 이전 출력값은 이상 동작이 발생할 수 있으므로 현재 출력값으로 저장함. 
        twistlinear_old_x  = twistlinear_now_x;
        twistlinear_old_y  = twistlinear_now_y;    
        twistangular_old_z = twistangular_now_z;
    }
    switch(LBox.state.ErrorCode)
    {
        case ERROR_BAT_UV:
        break;
        case ERROR_AGV_VIRTUALWALL_CRASH :
            switch(LBox.state.AGVControlMode)
            {
                case JOGMODE :
                    if( CHECKBIT(LBox.state.ErrorState,BIT_AGV_CRASH_FRONT_ERR) || CHECKBIT(LBox.state.ErrorState,BIT_AGV_CRASH_REAR_ERR) || CHECKBIT(LBox.state.ErrorState,BIT_AGV_CRASH_RIGHT_F_ERR) || CHECKBIT(LBox.state.ErrorState,BIT_AGV_CRASH_RIGHT_R_ERR))
                    {
                        if(CHECKBIT(LBox.state.ErrorState,BIT_AGV_CRASH_FRONT_ERR))     if(twistlinear_now_x > 0.0) twistlinear_x = 0.0;
                        if(CHECKBIT(LBox.state.ErrorState,BIT_AGV_CRASH_REAR_ERR))      if(twistlinear_now_x < 0.0) twistlinear_x = 0.0;
                        if(CHECKBIT(LBox.state.ErrorState,BIT_AGV_CRASH_RIGHT_F_ERR))   if(twistlinear_now_y < 0.0) twistlinear_y = 0.0;
                        if(CHECKBIT(LBox.state.ErrorState,BIT_AGV_CRASH_RIGHT_R_ERR))   if(twistlinear_now_y < 0.0) twistlinear_y = 0.0;
                    }
                    else
                    {
                        LBox.state.ErrorCode = ERROR_NON; 	// AGV 에러코드 표현
                    }
                break;
                case POSMODE :
                    //if(CHECKBIT(LBox.state.ErrorState,BIT_AGV_CRASH_FRONT_ERR))
                    // 2022년4월2일 : 최종 twist 값 갱신은 아래에서 함.
                    twistlinear_x  = twistlinear_now_x;
                    twistlinear_y  = twistlinear_now_y;    
                    twistangular_z = twistangular_now_z;
                    LBox.state.AGVControlMode = STOPMODE;
                break;
                default :
                //if(CHECKBIT(LBox.state.ErrorState,BIT_AGV_CRASH_FRONT_ERR))
                    twistlinear_x  = twistlinear_now_x;
                    twistlinear_y  = twistlinear_now_y;    
                    twistangular_z = twistangular_now_z;
                    LBox.state.AGVControlMode = STOPMODE;
                break;
            }
            break;
        case ERROR_AGV_POSITION_ERROR :
            switch(LBox.state.AGVControlMode)
            {
                case JOGMODE :
                // 조그모드 로 진행 중에 AGv 위치 피드백 에러가 발생하면 현재 속도값으로 갱신
                    twistlinear_x  = twistlinear_now_x;
                    twistlinear_y  = twistlinear_now_y;    
                    twistangular_z = twistangular_now_z;
                    LBox.state.AGVControlMode = STOPMODE;
                    LBox.state.ErrorStateFlag = ON;
                    
                    break;
                case POSMODE :
                // 위치모드로 진행 중에 AGV 위치 피드백 에러가 발생하면 PID 제어기로 인해 twist 출력값이 최대값이 되어 이상작동함.
                // 0.0 으로 하면 갑자기 서기 때문에 안됨.
                // 따라서 이전 twist 출력값으로 갱신하고 STOPMODE로 변경
                    twistlinear_x  = twistlinear_old_x;
                    twistlinear_y  = twistlinear_old_y;    
                    twistangular_z = twistangular_old_z;
                    LBox.state.AGVControlMode = STOPMODE;
                    LBox.state.ErrorStateFlag = ON;
                break;
                default :
                // 기본값일경우는 어떻게 해야 하는게 좋을까?  조그모드처럼?  아님 위치모드 처럼 ?
                // 일단은 위험 요소를 제거해야 함으로 위치모드로 설정. 갑자기 덜컹거리면 의심해야 함.
                // 기본 모드임으로 현재값으로 갱신. POSMODE 에서 STOP 모드로 갈경우 
                    twistlinear_x  = twistlinear_now_x;
                    twistlinear_y  = twistlinear_now_y;    
                    twistangular_z = twistangular_now_z;
                //    LBox.state.AGVControlMode = STOPMODE;
                break;
            }
            break;
        //case ERROR_NON:
        default :
        // 에러가 없는 경우는 현재값으로 갱신
            twistlinear_x  = twistlinear_now_x;
            twistlinear_y  = twistlinear_now_y;    
            twistangular_z = twistangular_now_z;
        break;  
    }   
    // twist 내부 이전 출력값 업데이트 
    twistlinear_old_x  = twistlinear_now_x;
    twistlinear_old_y  = twistlinear_now_y;    
    twistangular_old_z = twistangular_now_z;
    //최종 twist 출력값 업데이트
    twist->linear.x = twistlinear_x;
    twist->linear.y = twistlinear_y;
    twist->angular.z = twistangular_z;
    
}
void WatchDog_Battery_VDC_UVP(AGVModels *fAGV)           // 배터리 전압이 23.6V 이하 under voltage 상태일 경우
{
	static __uint16_t ErrOnFlag = 0;
	if(fAGV->state.BatVolt < BAT_MINIMUM_VDC)		// 입력전압이 MINIMUM_VDC 보다 작을 경우
	{
		if(ErrOnFlag)
		{
			if(TimeCount_W1 >= BAT_MINIMUM_VDC_TIMER) // 2초 이상이 UVP가 될 때
			{
				TimeCount_W1 = BAT_MINIMUM_VDC_TIMER;
				if(fAGV->state.ErrorCode == ERROR_NON)		// 이 조건문을 넣는 이유는 다른 에러가 걸렸을 경우 먼저 걸린 에러를 처리하기 위해서이다.
				{
					fAGV->state.ErrorCode = ERROR_BAT_UV; 	// AGV 에러코드 표현
				}
				SETBIT(fAGV->state.ErrorState,BIT_BAT_VDC_UV_ERR);  // AGV 에러코드에 대한 비트 
			}
		}
		else
		{
			ErrOnFlag 	= 1;
			TimeCount_W1 = 0;
		}
	}
	else
	{
		ErrOnFlag 	= 0;
		TimeCount_W1 = 0;
	}
}
void WatchDog_Positon_ERR(AGVModels *fAGV)
{
    static __uint16_t ErrOnFlag = 0;
    //static __uint16_t ErrWatchDogStartFlag = 0;
    double Err_x ;
    double Err_y ;
    double Err_rz;

    if(!fAGV->watchdog.WatchDogPositonERRStartFalg)
    {
        fAGV->watchdog.WatchDogPositonERRStartFalg = ON;
        fAGV->type.old.pos.x = fAGV->type.feed.pos.x;
        fAGV->type.old.pos.y = fAGV->type.feed.pos.y;
        fAGV->type.old.pos.rz = fAGV->type.feed.pos.rz;
    }
    Err_x = abs(fAGV->type.feed.pos.x - fAGV->type.old.pos.x);
    Err_y = abs(fAGV->type.feed.pos.y - fAGV->type.old.pos.y);
    Err_rz = abs(fAGV->type.feed.pos.rz - fAGV->type.old.pos.rz);
    // static long countiii = 0;
    // if(++countiii == 100)
    // {
    //     countiii = 0;
    //     printf("main.cpp line %d,  %lf     %lf     %lf  \n", __LINE__, Err_x , Err_y , Err_rz);
    //     printf("main.cpp line %d,  ErrorCode : %d  ErrorState :%d \n", __LINE__, fAGV->state.ErrorCode , fAGV->state.ErrorState);
        
    // }
    if((Err_x > POS_ERR_LIMIT_VALUE_X)||(Err_y > POS_ERR_LIMIT_VALUE_Y)||(Err_rz > POS_ERR_LIMIT_VALUE_RZ))	// 입력전압이 MINIMUM_VDC 보다 작을 경우
    {
        if(ErrOnFlag)
        {
            if(TimeCount_W3 >= WATCHDOG_POSITON_ERR_TIMER) // 0.02초 이상이 포지션 에러가 발생할 때
            {
                TimeCount_W3 = WATCHDOG_POSITON_ERR_TIMER;
                if(fAGV->state.ErrorCode == ERROR_NON)		// 이 조건문을 넣는 이유는 다른 에러가 걸렸을 경우 먼저 걸린 에러를 처리하기 위해서이다.
                {
                    fAGV->state.ErrorCode = ERROR_AGV_POSITION_ERROR; 	// AGV 에러코드 표현
                }
                SETBIT(fAGV->state.ErrorState,BIT_AGV_POSITION_ERR);  // AGV 에러코드에 대한 비트 
            }
        }
        else
        {
            ErrOnFlag 	= 1;
            TimeCount_W3 = 0;
        }
    }
    else
    {
        ErrOnFlag 	= 0;
        TimeCount_W3 = 0;
    }
    fAGV->type.old.pos.x = fAGV->type.feed.pos.x;
    fAGV->type.old.pos.y = fAGV->type.feed.pos.y;
    fAGV->type.old.pos.rz = fAGV->type.feed.pos.rz;	
}

void WatchDog_VitualWall_Detect_Front(AGVModels *fAGV)        // 항상 리얼타임으로 감시
{
    static __uint16_t ErrOnFlag = 0;
    // double AGVFrontRange = range_fr_.range;
    // double AGVRearRange  = range_br_.range;
    // double AGVRigntFrontRange = range_rf_.range;
    // double AGVRigntRearRange = range_rb_.range;

    if(AGVFrontRange < AGV_FRONT_RANGE)
    {
        if(ErrOnFlag)
		{
			if(TimeCount_W2_Front >= AGV_VIRTUALWALL_DETECT_TIMER) // 2초 이상이 UVP가 될 때
			{
				TimeCount_W2_Front = AGV_VIRTUALWALL_DETECT_TIMER;
				if(fAGV->state.ErrorCode == ERROR_NON)		// 이 조건문을 넣는 이유는 다른 에러가 걸렸을 경우 먼저 걸린 에러를 처리하기 위해서이다.
				{
					fAGV->state.ErrorCode = ERROR_AGV_VIRTUALWALL_CRASH; 	// AGV 에러코드 표현
				}
				SETBIT(fAGV->state.ErrorState,BIT_AGV_CRASH_FRONT_ERR);  // AGV 에러코드에 대한 비트 
			}
		}
		else
		{
			ErrOnFlag 	= 1;
			TimeCount_W2_Front = 0;
		}
    }
    else
    {
        ErrOnFlag 	= 0;
        TimeCount_W2_Front = 0;
//        fAGV->state.ErrorCode = ERROR_NON; 	// AGV 에러코드 표현
        CLEARBIT(fAGV->state.ErrorState,BIT_AGV_CRASH_FRONT_ERR);  // AGV 에러코드에 대한 비트
    }
}
void WatchDog_VitualWall_Detect_Rear(AGVModels *fAGV)        // 항상 리얼타임으로 감시
{
    static __uint16_t ErrOnFlag = 0;
    // double AGVFrontRange = range_fr_.range;
    // double AGVRearRange  = range_br_.range;
    // double AGVRigntFrontRange = range_rf_.range;
    // double AGVRigntRearRange = range_rb_.range;

    if(AGVRearRange < AGV_REAR_RANGE)
    {
        if(ErrOnFlag)
		{
			if(TimeCount_W2_Rear >= AGV_VIRTUALWALL_DETECT_TIMER) // 2초 이상이 UVP가 될 때
			{
				TimeCount_W2_Rear = AGV_VIRTUALWALL_DETECT_TIMER;
				if(fAGV->state.ErrorCode == ERROR_NON)		// 이 조건문을 넣는 이유는 다른 에러가 걸렸을 경우 먼저 걸린 에러를 처리하기 위해서이다.
				{
					fAGV->state.ErrorCode = ERROR_AGV_VIRTUALWALL_CRASH; 	// AGV 에러코드 표현
				}
				SETBIT(fAGV->state.ErrorState,BIT_AGV_CRASH_REAR_ERR);  // AGV 에러코드에 대한 비트 
			}
		}
		else
		{
			ErrOnFlag 	= 1;
			TimeCount_W2_Rear = 0;
		}
    }
    else
    {
        ErrOnFlag 	= 0;
        TimeCount_W2_Rear = 0;
//        fAGV->state.ErrorCode = ERROR_NON; 	// AGV 에러코드 표현
        CLEARBIT(fAGV->state.ErrorState,BIT_AGV_CRASH_REAR_ERR);  // AGV 에러코드에 대한 비트
    }
}
void WatchDog_VitualWall_Detect_RightF(AGVModels *fAGV)
{
    static __uint16_t ErrOnFlag = 0;
    // double AGVFrontRange = range_fr_.range;
    // double AGVRearRange  = range_br_.range;
    // double AGVRigntFrontRange = range_rf_.range;
    // double AGVRigntRearRange = range_rb_.range;

    if(AGVRigntFrontRange < AGV_RIGHT_FRONT_RANGE)
    {
        if(ErrOnFlag)
		{
			if(TimeCount_W2_RightF >= AGV_VIRTUALWALL_DETECT_TIMER) // 2초 이상이 UVP가 될 때
			{
				TimeCount_W2_RightF = AGV_VIRTUALWALL_DETECT_TIMER;
				if(fAGV->state.ErrorCode == ERROR_NON)		// 이 조건문을 넣는 이유는 다른 에러가 걸렸을 경우 먼저 걸린 에러를 처리하기 위해서이다.
				{
					fAGV->state.ErrorCode = ERROR_AGV_VIRTUALWALL_CRASH; 	// AGV 에러코드 표현
				}
				SETBIT(fAGV->state.ErrorState,BIT_AGV_CRASH_RIGHT_F_ERR);  // AGV 에러코드에 대한 비트 
			}
		}
		else
		{
			ErrOnFlag 	= 1;
			TimeCount_W2_RightF = 0;
		}
    }
    else
    {
        ErrOnFlag 	= 0;
        TimeCount_W2_RightF = 0;
//        fAGV->state.ErrorCode = ERROR_NON; 	// AGV 에러코드 표현
        CLEARBIT(fAGV->state.ErrorState,BIT_AGV_CRASH_RIGHT_F_ERR);  // AGV 에러코드에 대한 비트
    }
}
void WatchDog_VitualWall_Detect_RightR(AGVModels *fAGV)
{
    static __uint16_t ErrOnFlag = 0;
    // double AGVFrontRange = range_fr_.range;
    // double AGVRearRange  = range_br_.range;
    // double AGVRigntFrontRange = range_rf_.range;
    // double AGVRigntRearRange = range_rb_.range;

    if(AGVRigntRearRange < AGV_RIGHT_REAR_RANGE)
    {
        if(ErrOnFlag)
		{
			if(TimeCount_W2_RightR >= AGV_VIRTUALWALL_DETECT_TIMER) // 2초 이상이 UVP가 될 때
			{
				TimeCount_W2_RightR = AGV_VIRTUALWALL_DETECT_TIMER;
				if(fAGV->state.ErrorCode == ERROR_NON)		// 이 조건문을 넣는 이유는 다른 에러가 걸렸을 경우 먼저 걸린 에러를 처리하기 위해서이다.
				{
					fAGV->state.ErrorCode = ERROR_AGV_VIRTUALWALL_CRASH; 	// AGV 에러코드 표현
				}
				SETBIT(fAGV->state.ErrorState,BIT_AGV_CRASH_RIGHT_R_ERR);  // AGV 에러코드에 대한 비트 
			}
		}
		else
		{
			ErrOnFlag 	= 1;
			TimeCount_W2_RightR = 0;
		}
    }
    else
    {
        ErrOnFlag 	= 0;
        TimeCount_W2_RightR = 0;
//        fAGV->state.ErrorCode = ERROR_NON; 	// AGV 에러코드 표현
        CLEARBIT(fAGV->state.ErrorState,BIT_AGV_CRASH_RIGHT_R_ERR);  // AGV 에러코드에 대한 비트
    }
}

#define FAIL    -1
#define SUCCESS 1

#define READ_ENCODER_COUNT 0
#define SET_ENCODER_COUNT 1

/*
int opMotorPosition(int op, mecanum::MotorPosition& motorPosition) {
    motorPosition.request.set = op;

    client_motor_position.call(motorPosition);

    if (!motorPosition.response.success) {
        return FAIL;
    }

    return SUCCESS;
}

int getMotorPosition(mecanum::MotorPosition& motorPosition) {
    return opMotorPosition(READ_ENCODER_COUNT, motorPosition);
}

int setMotorPosition(mecanum::MotorPosition& motorPosition) {
    return opMotorPosition(SET_ENCODER_COUNT, motorPosition);
}
*/
