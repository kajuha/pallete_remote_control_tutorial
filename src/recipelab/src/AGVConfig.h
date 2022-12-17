/*======================================================================
	File name	:	AGVConfig.h                    
                    
	Originator	:	AGV Control

	Target		:	ROS 

	Version		:	1.00


======================================================================*/

/*======================================================================
	History		:
		2021-04-20,		Version 1.00
======================================================================*/
#ifndef _ABC
#define _ABC
#include <stdio.h>
#include <math.h>
#include <cstdlib>
#include <sys/time.h>
#include <tf/tf.h>
#include "Eigen/Dense"
//#include <Eigen/Core>

#define SETBIT(data,loc)	((data) |= (0x1<<(loc-1)))
#define CLEARBIT(data,loc)	((data) &= ~(0x1<<(loc-1)))
#define CHECKBIT(data,loc)	((data) &  (0x1<<(loc-1)))

/*************************************************************************************************************/
/*************************************************************************************************************/
// ======= General Variable define ==============================================
// 명령어 종류
#define ON					1
#define OFF					0
// ======= ERROR bit DEFINE AGV Internal Fail STATUS =================================================
#define	BIT_BAT_VDC_OV_ERR		    0	// 배터리 전원의 전압 over voltage 에러
#define	BIT_BAT_VDC_UV_ERR		    1	// 배터리 전원의 전압 under voltage 에러
#define BIT_AGV_CRASH_FRONT_ERR     2   // AGV 가상벽 충돌 에러 전방
#define BIT_AGV_CRASH_REAR_ERR      3   // AGV 가상벽 충돌 에러 후방
#define BIT_AGV_CRASH_RIGHT_F_ERR   4   // AGV 가상벽 충돌 에러 우측 전방
#define BIT_AGV_CRASH_RIGHT_R_ERR   5   // AGV 가상벽 충돌 에러 우측 후방
#define BIT_AGV_POSITION_ERR        6   // AGV 위치 에러



/*
#define IN_IAC_OC_ERR		2	// 입력 전류 over current error
#define IN_IAC_UC_ERR		3	// 입력 전류 under current error
#define OUT_VDC_OV_ERR		4	// 출력  전압 over voltage error
#define OUT_VDC_UV_ERR		5	// 출력  전압 under voltage error
#define OUT_IDC_OC_ERR		6	// 출력  total 전류 over current error
#define OUT_IDC_UC_ERR		7	// 출력  total 전류 under current error
#define	OUT_I1I2_SAME_ERR	8	// 출력 전류 IO1과 IO2 가 같지 않음 error
#define TEMP_DIODE1_OV_ERR	9
#define TEMP_DIODE2_OV_ERR	10
#define TEMP_IGBT_OV_ERR	11
#define TEMP_BOARD_OV_ERR	12
#define TEMP_BUSBAR_OV_ERR	13
#define RS485_CONNECT_ERR	14
#define COOLANT_ERR			15	// 냉각수 차단 에러
*/
// ======= ERROR bit DEFINE of MODULE ERROR CODE of MODULE_MODBUS_PROTOCOL_DATA ==
#define ERROR_NON			            0
#define	ERROR_COMM			            1		// 통신 에러
#define	ERROR_CMD			            2		// 명령 에러
#define	ERROR_BAT_OV		            3		// 배터리 전원의 전압 over voltage 에러  
#define	ERROR_BAT_UV		            4		// 배터리 전원의 전압 under voltage 에러

#define	ERROR_AGV_VIRTUALWALL_CRASH		5		// AGV 가상벽 충돌 에러

#define ERROR_AGV_POSITION_ERROR        6       // AGV command 대비 feedback value 오차 에러
/*
#define	ERROR_OUT_OC		5		// 출력 전류 over current 에러
#define	ERROR_OUT_OV		6		// 출력 전압 over voltage 에러
#define	ERROR_PT_OT			7		// Pulse trans. Heatsink Over Temperature
#define	ERROR_DRIVE_FAULT	8		// drive fault
#define	ERROR_OT_FAIL		9		// over temperature fail
#define	ERROR_CUR_SEN_FAIL	10		// 전류 센서 고장 에러
#define	ERROR_SYS_FAIL		11		// 시스템 fail(정의되지 않는 에러 포함)
#define	ERROR_NO_LOAD_FAIL	12		// 출력 전류 under current 에러
*/
// ======= STATUS bit DEFINE of STATUS of MODULE_PROTOCOL_DATA ========
#define	STATUS_READY		0
#define	STATUS_RUN			1
#define STATUS_SWITCH_POS	2
#define reserved1			3
#define STATUS_MODE			4
#define reserved2			5
#define reserved3			6
#define STATUS_RESET		7

#define MIN_POS             0.001                   // 위치프로파일 최소 이동거리 , 최소이동거리보다 적을경우 동작하지 않도록 한다.
#define MIN_VEL             0.000                   // 속도프로파일 최소 이동거리 , 최소이동거리보다 적을경우 동작하지 않도록 한다.
// ======= Timer varialbe setting ==================
#define TIME_main_Hz        100.0                                   // (Hz)  메인타이머 100Hz ,
#define TIME_main_sec       (1.0/TIME_main_Hz)                      // (sec) 메인타이머 0.01sec
#define Ts_main			    0.05                                    // (sec) 위치,속도 프로파일 단위시간 0.05sec
#define TIME_main_count     (__uint32_t)(1000*TIME_main_sec)        // 메인타이머가 10msec임으로 1msec 기준으로 메인타이머의 10msec는 10 count 임.
#define TIME_10msec         (10/TIME_main_count)                    // (count)main timer 10msec , 10msec count = 1
#define TIME_50msec         (50/TIME_main_count)                    // (count)main timer 10msec , 50msec count = 5
#define TIME_100msec        (100/TIME_main_count)                   // (count)main timer 10msec , 100msec count = 10
#define TIME_1000msec       (1000/TIME_main_count)                  // (count)main timer 10msec , 1000msec count = 100


// WatchDog Limit Variable
#define BAT_MINIMUM_VDC			        23.6				    // (V) 배터리 최저전압
#define POS_ERR_LIMIT_VALUE_X           0.1                     // (m) AGV 위치 X축 에러 범위값
#define POS_ERR_LIMIT_VALUE_Y           0.1                     // (m) AGV 위치 X축 에러 범위값
#define POS_ERR_LIMIT_VALUE_RZ          (5 * D2R)               // (rad) AGV 위치 X축 에러 범위값

#define AGV_FRONT_RANGE                 0.5                     // (m) AGV 전방 가상벽 값 0.5m  --> AGV 센서 기준
#define AGV_REAR_RANGE                  0.5                     // (m) AGV 후방 가상벽 값 0.5m  --> AGV 센서 기준
#define AGV_RIGHT_FRONT_RANGE           0.3                    // (m) AGV 우측 가상벽 값 0.65m  --> AGV 센서 기준
#define AGV_RIGHT_REAR_RANGE            0.3                    // (m) AGV 우측 가상벽 값 0.65m  --> AGV 센서 기준
#define AGV_FEEDBACK_X_RANGE            0.1                     // (m) AGV X축 PID 에러값 0.1m
#define AGV_FEEDBACK_Y_RANGE            0.1                     // (m) AGV Y축 PID 에러값 0.1m
#define AGV_FEEDBACK_RZ_RANGE           0.087                   // (m) AGV RZ축 PID 에러값 5deg 0.087rad


#define BAT_MINIMUM_VDC_TIMER	        (TIME_1000msec*2)		// (count) 배터리 이상전압 체크 시간 2초
#define WATCHDOG_POSITON_ERR_TIMER	    (TIME_10msec*2)		    // (count) 위치에러 감지시간 시간 0.02초

#define AGV_VIRTUALWALL_DETECT_TIMER    (TIME_100msec*3)        // (count) AGV 가상벽 충돌 체크 시간 0.3초
#define AGV_FEEDBACK_ERROR_TIMER        (TIME_50msec*1)         // (count) agv edback error 체크 시간 0.05초 실시간
// ======= AGV Module constant value define ==================
/*
#define MAXIMUM_TEMP				70.0			// 온도 최대값
#define MINIMUM_TEMP				-5.0			// 온도 최소값
#define MAXIMUM_RECOVER_TEMP		45.0		// 냉각수가 차단되고 다시 회복되는 온도 (45도)
#define MAXIMUM_RECOVER_BUS_TEMP	55.0		// 냉각수가 차단되고 다시 회복되는 온도 (45도)
#define MAX_BUSBAR_TEMP		85.0 			// BUSBAR 최대 온도 75 -> 85변경
#define MIN_BUSBAR_TEMP		-5.0			// BUSBAR 최소 온도
#define MAXIMUM_TOT_CURT	1450.0//1500.0	// Total Current 최대값
#define MINIMUM_TOT_CURT	50.0			// Total Current 최소값. 기본값 : 100 ; 20130123에 50A로 수정
#define MAXIMUM_VAC			750.0			// Rect 입력전원 전압 최대값
#define MINIMUM_VAC			495.0  			// Rect 입력전원 전압 최소값
#define LOW_RECOVER_VAC		290.0			// Rect 입력 전원 under Voltage 경계값
#define HIGH_RECOVER_VAC	515.0			// Rect 입력 전원 over  Voltage 경계값
#define MAXIMUM_VDC			28.0			// Rect(다이오드 출력단) 출력 전압 최대값
#define MINIMUM_VDC			2.5//2.5			// Rect(다이오드 출력단) 출력 전압 최소값
#define LOW_RECOVER_VDC		19.0			// Rect(다이오드 출력단) 출력 under Voltage 경계값
#define HIGH_RECOVER_VDC	2.0 			// Rect(다이오드 출력단) 출력 over  Voltage 경계값
#define MAXIMUM_RCM_VDC		240				// RCM에서 들어오는 출력전압 명령의 최대치 설정	240 --> 24.0
#define	MAXIMUM_RCM_ITOT_20	1300			// RCM에서 들어오는 출력전류 명령의 최대치 설정 --> 명령전압이 20V이하일 경우
#define	MAXIMUM_RCM_ITOT_24	1125			// RCM에서 들어오는 출력전류 명령의 최대치 설정 --> 명령전압이 20V이상일 경우
#define COOLANT_START_TEMP	41.0			// 냉각수 차단 와치독을 시작하는 시점 온도. 35도 --> 45도로 변경
#define MAXIMUM_TEMPDOT		2.0				// 냉각수가 차단될 때의 온도 변화량 (7도) (1.86도) 8.0 --> 2.0으로 변경
#define MAXIMUM_RECOVER_IGBT_TEMP1	44.0		// 냉각수가 차단되고 다시 회복되는 온도 (45도)
#define MINIMUM_VAC_TIMER	5000			// 100usec 카운터이기에 값이 5000이면 500msec이다.
*/
// MainMode Type
#define JOGMODE		        0xA0        // 160
#define POSMODE		        0xA1        // 161
#define STOPMODE	        0xA2        // 162
#define ESTOPMODE	        0xA3        // 163
#define JOYMODE             0xA4        // 164
#define POSEMODE            0xA5        // 165
#define PTABLEMODE          0xA6        // 166
#define ERRORMODE           0xAA        // 170
#define INITMODE            0xAF        // 175
#define READYMODE           0xAE        // 174

// Control Mode 

// AgvRunState  Mode
#define READY               0xB0        // 176
#define RUN                 0xB1        // 177
#define RUNNING             0xB2        // 178
#define STOP                0xB3        // 179
#define ERROR               0xB4        // 180

// Agv profile State  Mode
#define PROFILEInit         0xA0        // 160
#define PROFILERun          0xB0        // 176
#define PROFILERunning      0xB1        // 177
#define PROFILEStop         0xC0        // 192

// AGV Odometry Type
#define ODOM_KINEMATIC      0
#define ODOM_AXIS1LIDAR     1
#define ODOM_COMPLEMENT     2
#define ODOM_KALMAN         3
#define ODOM_AXIS2LIDAR     4



#define GEAR51 	        51				// AGV 감속기 설정. 감속기는 51:1, 풀리는 1:1 임.
//#define MY_PI  3.1415926535
#define D2R             (M_PI/180.0)
#define R2D             (180.0/M_PI)


#define agv_pos_linear_saturation 0.030         // Uint : m,
#define agv_pos_angular_saturation (1.5 * D2R)  // Uint : rad,


/*-----------------------------------------------------------------------------
Default initalizer for the PIDREG3 object.
-----------------------------------------------------------------------------*/ 
class AGV_TIMETEMP {        // 제어 주기등을 알기위한 클래스
    public:
    __uint16_t FirstRun;
    double time_pre;
    double time_diff;
    double time_step;
    AGV_TIMETEMP(){
        FirstRun = ON;
    }
    ~AGV_TIMETEMP() {}
}; 
class PID {   // 클래스 이름은 PID
    public:
    double  Ref;   			// Input: Reference input 
    double  Fdb;   			// Input: Feedback input 
    double  Err;			// Variable: Error
    double  Kp;				// Parameter: Proportional gain
    double  Up;				// Variable: Proportional output 
    double  Up1;		   	    // History: Previous proportional output
    double  Ui;				// Variable: Integral output 
    double  Ud;				// Variable: Derivative output 	
    double  OutPreSat; 		// Variable: Pre-saturated output
    double  OutMax;		    // Parameter: Maximum output 
    double  OutMin;	    	// Parameter: Minimum output
    double  Out;   			// Output: PID output
    double  OutOld;          // Output previous value 
    double  SatErr;			// Variable: Saturated difference
    double  Ki;			    // Parameter: Integral gain
    double  Kc;		     	// Parameter: Integral correction gain
    double  Kd; 		        // Parameter: Derivative gain

    public:
    //PID funtion
    void pidfn()
    {	
        double ad;
        double ads;
        double adsf;
        double errbuff;
        // Compute the error
        Err = Ref - Fdb;
        // Compute the proportional output
        Up = Kp*Err;
        // Compute the integral output
        Ui = Ui + Ki*Up + Kc*SatErr;
        // Compute the derivative output
        ad = (Up - Up1);
        ads = Kd;
        adsf = ads*ad;
        Ud = adsf;
        // Compute the pre-saturated output
        OutPreSat = Up + Ui + Ud;
        // Saturate the output
        if (OutPreSat > OutMax)
        Out =  OutMax;
        else if (OutPreSat < OutMin)
        Out =  OutMin;
        else
        Out = OutPreSat;
        // Compute the saturate difference
        SatErr = Out - OutPreSat;
        // Update the previous proportional output
        Up1 = Up; 
    }
    PID() {
		Ref = 0.0;   			// Input: Reference input 
        Fdb = 0.0;   			// Input: Feedback input 
        Err = 0.0;				// Variable: Error
        Kp = 0.0;				// Parameter: Proportional gain
        Up = 0.0;				// Variable: Proportional output 
        Up1 = 0.0;		   	    // History: Previous proportional output
        Ui = 0.0;				// Variable: Integral output 
        Ud = 0.0;				// Variable: Derivative output 	
        OutPreSat = 0.0; 		// Variable: Pre-saturated output
        OutMax = 0.0;		    // Parameter: Maximum output 
        OutMin = 0.0;	    	// Parameter: Minimum output
        Out = 0.0;   			// Output: PID output 
        SatErr = 0.0;			// Variable: Saturated difference
        Ki = 0.0;			    // Parameter: Integral gain
        Kc = 0.0;		     	// Parameter: Integral correction gain
        Kd = 0.0; 		        // Parameter: Derivative gain
	}
    ~PID() {}
};
class AGVKamanFilter {
    public:
    __uint16_t FirstRun;
    Eigen::Matrix3d A;
    Eigen::Matrix3d B;
    Eigen::Matrix3d H;
    Eigen::Matrix3d Q;
    Eigen::Matrix3d R;
    Eigen::Matrix3d P;
    Eigen::Matrix3d P_p;
    Eigen::Matrix3d K;
    Eigen::Matrix <double, 3, 1> x;			// 추정 초기값
    Eigen::Matrix <double, 3, 1> x_p;		// 추정값
    Eigen::Matrix <double, 3, 1> u;		    // 
    Eigen::Matrix <double, 3, 1> z;		    // 측정 센서값
    
    public:
    void KamanFilter(double *fx,double *fu, double *fz)
    {
        if(FirstRun)
        {
            A = Eigen::Matrix3d::Identity() ;
            B = Eigen::Matrix3d::Identity() ;
            H = Eigen::Matrix3d::Identity() ;
            Q = 0.001 * Eigen::Matrix3d::Identity() ;
            R = 1000 * Eigen::Matrix3d::Identity() ;
            P = 1 * Eigen::Matrix3d::Identity() ;
            //x(0,0) = 0.0; x(1,0) = 0.0; x(2,0) = 0.0;
            x(0,0) = fu[0]; x(1,0) = fu[1]; x(2,0) = fu[2];
            FirstRun = OFF;		 
        }
        for(__uint16_t i=0;i<3;i++){
            u(i,0) = fu[i];
            z(i,0) = fz[i];
        }
        x_p = A * x + B * u;
        P_p = A * P * A.transpose() + Q;
        Eigen::Matrix3d comp = H * P_p * H.transpose() + R ;
        K   = P_p * H.transpose() * comp.inverse();
        x   = x_p + K *(z - H * x_p);
        P   = P_p - K * H * P_p;
        fx[0] = x(0,0);
        fx[1] = x(1,0);
        fx[2] = x(2,0);
    }
    AGVKamanFilter(){
        FirstRun = ON;
    }
    ~AGVKamanFilter() {}
};
/*
class AGV_ODOMETRY {   // ?��?��?�� ?��름�?? AGVODOMETRY
    public:
    double  x;   			// Input: Reference input 
    double  y;   			// Input: Feedback input 
    double  z;				// Variable: Error
    double  w;
    double  rx;				// Parameter: Proportional gain
    double  ry;				// Variable: Proportional output 
    double  rz;		   	    // History: Previous proportional output

    public:
    //AGVODOMETRY Class variable initialize
    AGV_ODOMETRY() {
		x = 0.0;   			// Input: Reference input 
        y = 0.0;   			// Input: Feedback input 
        z = 0.0;				// Variable: Error
        w = 0.0;				// Variable: Error
        rx = 0.0;				// Parameter: Proportional gain
        ry = 0.0;				// Variable: Proportional output 
        rz = 0.0;		   	    // History: Previous proportional output
    }    
    ~AGV_ODOMETRY() {}
};
*/

/*------------------------------------------------------------------------------
// define Velocity Profile ----------------------------------------------------
-------------------------------------------------------------------------------*/
typedef struct{	double Init_Pos;		    // 초기 위치	--> 엔코더 값에 의해 계산되어짐
				double Final_Pos;	    // 최종 위치	--> 초기위치와 이동해야 할 거리로 계산되어짐.
				double Length_Pos;	    // 이동해야 할 거리	--> Input Command
				double Output_Pos;	    // 출력 위치
				double Init_Vel;		    // 초기 속도	--> 메카넘휠 odometry에 의해 계산되어짐
				double Final_Vel;	    // 최종 속도	--> 초기속도와 동작해야할 속도로 계산되어짐.
				double Length_Vel;	    // 이동해야할 속도	--> Input Command
				double Output_Vel;	    // 출력 속도				
				double Max_Vel;		    // 시스템의 최대 허용 속도	--> 시스템 상태에 따라 달라짐.
				double Comd_Vel;		// 이동하는 동안의 속도 --> Input Command					
				double Max_Accel;	    // 시스템의 최대 허용 각속도	--> 시스템 상태에 따라 달라짐.
				double Design_Accel;	// 이동해야 할 거리 와 원하는 속도에 의해 구해진다.				
				double Ts;			    // 가속구간 및 감속구간의 시간
				double T;			    // 이동하는 동안 걸리는 시간	--> 속도와 가속도에 의해 구해진다.
				double tcounter;		// 시간의 흐름
				__uint8_t StartFlag;	// 프로파일 시작 플레그
//                __uint8_t ONFlag;	    // 속도 프로파일 동작 플레그
//                __uint8_t SlopUp;     // 상위명령에서 속도 프로파일 명령줄 때 활용.
//                __uint8_t SlopDown;    // 상위명령에서 속도 프로파일 명령줄 때 활용.
				}PROFILE_JOINT;
typedef struct{	double Init_Pos[3];	    // 초기 위치	--> 엔코더 값에 의해 계산되어짐
				double Final_Pos[3];	    // 최종 위치	--> 초기위치와 이동해야 할 거리로 계산되어짐.
				double Length_Pos;	    // 이동해야 할 거리	--> Input Command
				double Output_Pos[3];    // 출력 위치				
				double Max_Vel;		    // 시스템의 최대 허용 속도	--> 시스템 상태에 따라 달라짐.
				double Comd_Vel;		    // 이동하는 동안의 속도 --> Input Command					
				double Max_Accel;	    // 시스템의 최대 허용 각속도	--> 시스템 상태에 따라 달라짐.
				double Design_Accel;	    // 이동해야 할 거리 와 원하는 속도에 의해 구해진다.				
				double Ts;			    // 가속구간 및 감속구간의 시간
				double T;			    // 이동하는 동안 걸리는 시간	--> 속도와 가속도에 의해 구해진다.
				double tcounter;		    // 시간의 흐름
				__uint8_t StartFlag;	// 속도 프로파일 시작 플레그
				}PROFILE_CART;
/*-----------------------------------------------------------------------------
// define Cartecian, Joint, Motor Variable 구조체 선언--------------------------
-----------------------------------------------------------------------------*/
// 사용자 정의 함수 정의 
// 사용자 정의 함수 정의
struct Inpos2D {
    __uint32_t	x;	    //linear		     
    __uint32_t 	y;	    //linear			    			    
    __uint32_t 	rz;	    //angular
    __uint32_t 	all;	// 전좌표 		         
};
struct Inpos3D {
    __uint32_t	x;	    //linear		     
    __uint32_t 	y;	    //linear
    __uint32_t	z;	    //linear	
    __uint32_t	w;	    //linear	     
    __uint32_t 	rx;	    //angular
    __uint32_t 	ry;	    //angular    			    
    __uint32_t 	rz;	    //angular
    __uint32_t 	all;	// 전좌표 		         
};
struct Axis2D {
    double	x;	    //linear		     
    double 	y;	    //linear			    			    
    double 	rz;	    //angular		         
};
struct Axis3D {
    double	x;	    //linear		     
    double 	y;	    //linear			    
    double 	z;	    //linear
    double  w;      // quaternion w
    double	rx;	    //angular			     
    double 	ry;	    //angular			    
    double 	rz;	    //angular		         
};
struct Mode2D {
    Axis2D pos;
    Axis2D vel;
    Axis2D acc;
};
struct Mode3D {
    Axis3D pos;
    Axis3D vel;
    Axis3D acc;
};
struct Twist3D {
    Axis3D linear;
    Axis3D angular;
};
struct Type2D {
    Mode2D tcomd;       // 상위제어기에서 받는 명령어를 저장하는 공간
    Mode2D comd;
    Mode2D feed;
    Mode2D feed_old;
    Mode2D feed_buff;
    Mode2D feed_now;
    Mode2D init;
    Mode2D old;
    Mode2D now;
    Mode2D Relative;
    Mode2D abs;
    Mode2D error;
    Twist3D twist;
};
struct Type3D {
    Mode3D tcomd;       // 상위제어기에서 받는 명령어를 저장하는 공간
    Mode3D comd;
    Mode3D feed;
    Mode3D feed_old;
    Mode3D feed_buff;
    Mode3D feed_now;
    Mode3D init;
    Mode3D old;
    Mode3D now;
    Mode3D Relative;
    Mode3D abs;
    Mode3D error;
};
struct States {
    __uint32_t	AGVId;			    // AGV 식별을 위한 ID 부여
    __uint32_t	AGVersion;		    // AGV 내장 프로그램 버전           
    __uint32_t	AGVControlMode;	    // AGV 제어모드 설정 (상위제어기에서 명령받아서 변경, 단 위치제어기에서는 위치제어가 끝난후 자동으로 대기모드로 감. ) (  A0 : 조그 모드(속도 모드)  ,  A1 : 위치 모드 , AF : 초기화모드 , AE : 대기모드)
    __uint32_t 	AgvRunState;		// AGV 현재 동작상태  (  B0 : Ready , B1 : run , B2 : running , B3 : stop , B4 : Error , 등등)
//    __uint32_t 	AgvMode3; 		    // AGV 모드 설정  (  C0 : Ready , C1 : AGV 상태 수신 , C2 : AGV 상태 송신 , C3 : AGV 초기화 ) 
    __uint32_t	StateIS;			// AGV 상태
    __uint32_t  Inpos;              // AGV 위치 제어시 
    __uint32_t  ErrorStateFlag;     // AGV 에러가 처음 발생 했을경우 에러가 발생하고 RCM 명령 함수에 진입하기 위한 변수임. 
    __uint32_t	ErrorState;		    // AGV 에러 상태 bit로 표현   -->  에러가 없을경우 0x0000 임.	
    __uint32_t	ErrorCode;		    // AGV 에러 코드 --> 우선적으로 발생된 에러만 처리
    __uint32_t  AGVOdomType;         // AGV Odometry Type
    double      BatVolt;            // 주전원 배터리 전압        
};
struct Watchdog {
    __uint32_t  WatchDogPositonERRStartFalg;     // AGV WatchDog_Positon_ERR 함수내 초기값 설정 하기 위한 변수 임.
};
struct Range {
    Mode2D max;
    Mode2D min;
    Mode2D mid;
};
struct AGV_ODOMETRY {   //  AGVODOMETRY
    Type3D odom;
};
struct Profile {

};
struct PidMode {
    PID x;
    PID y;
    PID rz;
};

struct Control {
    PidMode     pid;
    Inpos2D     inpos;              // AGV 위치제어 시 목표위치 도달했는지를 확인하는 변수
    Inpos2D     invel;              // AGV 위치제어 시 목표위치 도달했는지를 확인하는 변수
};

struct AGVModels {
    States      state;
    Range       range;
    Type2D      type;
    Control     control;
    Watchdog    watchdog;
    
};

//---- 1축 Lidar 센서 조합의 AGV Odometry 알고리즘 ----//
enum AGV_MODEL {
	LUNCHBOX = 0,
	TURTLESHIP = 1,
	PALLETE = 2
};

class AGV1AxisSensorBaseOdometry {   // 구조체 이름은 AGV1AxisSensorBaseOdometry
    private:    
    double dl1x;
    double dl1y;
    double dl2x;
    double dl2y;
    double dl3x;
    double dl3y;
    double dl4x;
    double dl4y;

    public:
    double x1;              // unit:m,   AGV odometry x
    double y1;              // unit:m,   AGV odometry y
    double x2;              // unit:m,   AGV odometry x2 
    double theta;           // unit:rad, AGV 전방 벽 x 위치
    double theta_deg;       // unit:deg, theta from rad to deg
    double dl1;             // unit:m , TF40-Front 센서 측정거리
    double dl2;             // unit:m , TF40-End   센서 측정거리
    double dl3;             // unit:m , TF02-End   센서 측정거리
    double dl4;             // unit:m , TF02-Front 센서 측정거리

    public:
    //void AGVLocationRecognition_ForwardKinematics(double dl1, double dl2, double dl3, double dl4)
    void AGVLocationRecognition_ForwardKinematics(void)
    {
        // theta = atan2(dl1-dl2 , dl1x+dl2x);
        // theta_deg = theta * R2D;
        // y1 = (dl2y + dl2 + dl2x*tan(theta))*cos(theta);
        // x1 = (dl3x + dl3 + dl3y*tan(theta))*cos(theta);
        // x2 = (dl4x + dl4 + dl4y*tan(theta))*cos(theta) + x1;
        theta = atan2(dl1-dl2 , dl1x+dl2x);
        theta_deg = theta * R2D;
        y1 = (dl1y + dl1 - dl1x*tan(theta))*cos(theta);
        x1 = (dl3x + dl3 - dl3y*tan(theta))*cos(theta);
        x2 = (dl4x + dl4 + dl4y*tan(theta))*cos(theta) + x1;
    }
    //void AGVLocationRecognition_InverseKinematics(double x1, double y1111, double x2, double theta)
    void AGVLocationRecognition_InverseKinematics(void)
    {
        dl1 = y1/cos(theta) + dl1x*tan(theta) - dl1y;
        dl2 = dl1 - (dl1x+dl2x)*tan(theta);
        dl3 = x1/cos(theta) + dl3y*tan(theta) - dl3x;
        dl4 = (x2-x1)/cos(theta) - dl4y*tan(theta) - dl4x;

    }
    AGV1AxisSensorBaseOdometry() {}
    AGV1AxisSensorBaseOdometry(AGV_MODEL agvModel) {
        switch (agvModel) {
            case LUNCHBOX:
                dl1x = 0.513; 
                dl1y = 0.556;           // Uint : m,        TF40-Front 센서 장착위치 (AGV 중심 기준)
                dl2x = 0.513; 
                dl2y = 0.556;           // Uint : m,        TF40-End 센서 장착위치 (AGV 중심 기준)
                dl3x = 0.6338; 
                dl3y = 0.458;          // Uint : m,        TF02-End 센서 장착위치 (AGV 중심 기준)
                dl4x = 0.6338; 
                dl4y = 0.458;          // Uint : m,        TF02-Front 센서 장착위치 (AGV 중심 기준)
            break;
            case TURTLESHIP:
                dl1x = 0.710; 
                dl1y = 0.658;           // Uint : m,        TF40-Front 센서 장착위치 (AGV 중심 기준)
                dl2x = 0.710; 
                dl2y = 0.658;           // Uint : m,        TF40-End 센서 장착위치 (AGV 중심 기준)
                dl3x = 0.8308; 
                dl3y = 0.560;          // Uint : m,        TF02-End 센서 장착위치 (AGV 중심 기준)
                dl4x = 0.8308; 
                dl4y = 0.560;          // Uint : m,        TF02-Front 센서 장착위치 (AGV 중심 기준)
            break;
            default:
            break;
        }
    }
    ~AGV1AxisSensorBaseOdometry() {}
};                   // typedef를 사용하여  별칭을 AGV1AxisSensorBaseOdometry 정의
//---- 1축 Lidar 센서 조합의 AGV Odometry 알고리즘 ----//
/*------------------------------------------------------------------------------
------------------------------------------------------------------------------*/
void MemCopy(__uint16_t *SourceAddr, __uint16_t* SourceEndAddr, __uint16_t* DestAddr);
double PosProfile(double fInit_Pos,double fFinal_Pos,double fv_in,double fa_in,PROFILE_JOINT *fpm);
double VelProfile(double fInit_Vel,double fFinal_Vel,double fa_in,PROFILE_JOINT *fpm);
void PosProfileCartecian(double* fInit_Pos,double* fFinal_Pos,double fv_in,double fa_in,PROFILE_CART *fpm);
void AGVLocationRecognition(double fdl1, double fdl2, double fdl3, double fdl4, AGV1AxisSensorBaseOdometry *fAGVodom);
//===========================================================================
// End of file.
//===========================================================================

#endif
