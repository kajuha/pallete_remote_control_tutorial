#include <std_msgs/Empty.h>
#include <geometry_msgs/Twist.h>
#include <boost/thread.hpp>

#include <ros/ros.h>

#include <string>

#include "chatterbox/ChatIn.h"
#include "chatterbox/ChatOut.h"
#include "chatterbox/WhisperOut.h"

#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>

#include <chrono>
#include <iostream>
#include <sys/time.h>
#include <ctime>

#define ON                  1
#define OFF                 0
#define TCP_DUMMY_SEND      OFF

// twist
typedef struct _Axis {
    double x;
    double y;
    double z;
} Axis;

typedef struct _Twist {
    Axis linear;
    Axis angular;
} Twist;

// 센서 종류
uint32_t sensor;

// 플랫폼 위치 파라미터
typedef struct _PosStateIn {
	double x;	// unit: m
	double y;	// unit: m
	double rz;	// unit: deg
} PosStateIn;

typedef struct _PosStateOut {
	double x;	// unit: m
	double y;	// unit: m
	double rz;	// unit: deg
	double ang;	// unit: deg
} PosStateOut;

// 속도 파라미터
typedef struct _VelParam {
	double vel;		// unit: m/s
	double acc;		// unit: m/s^2
	double vmax;	// unit: m/s
} VelParam;

// 플랫폼 속도 파라미터
typedef struct _VelParams {
	VelParam x;
	VelParam y;
	VelParam rz;
} VelParams;

// 조그 정보
typedef struct _JogInfo {
	uint32_t front;	// push: 1, release: 0
	uint32_t back;	// push: 1, release: 0
	uint32_t left;	// push: 1, release: 0
	uint32_t right;	// push: 1, release: 0
	uint32_t cw;	// push: 1, release: 0
	uint32_t ccw;	// push: 1, release: 0
} JogInfo;

// 정지 정보
typedef struct _StopInfo {
	uint32_t stop;
} StopInfo;

// 상태입력 정보
typedef struct _StateInfo {
	uint32_t period;
} StateInfo;

// 센서 정보
typedef struct _SensorState {
	double front;		// m
	double back;		// m
	double right_front;	// m
	double right_back;	// m
} SensorState;

// 플랫폼 정보
typedef struct _PlatformState {
    uint32_t mode;
	uint32_t state;
} PlatformState;

// 배터리 정보
typedef struct _BatteryState {
    double voltage;
    double current;
    double percentage;
} BatteryState;

// 총 정보
#pragma pack(push, 1)
typedef struct _TotalState {
	SensorState sensor;
	PosStateOut pos;
	PlatformState platform;
    BatteryState battery;
} TotalState;
#pragma pack(pop)

// 상세메시지정보
struct CustomString {
#define SUBJECT_SIZE    16
    unsigned char subject[SUBJECT_SIZE];
    double value;
};

#pragma pack(push, 1)
struct WhisperState {
    CustomString val00;
    CustomString val01;
    CustomString val02;
    CustomString val03;
    CustomString val04;
    CustomString val05;
    CustomString val06;
    CustomString val07;
    CustomString val08;
    CustomString val09;
    CustomString val10;
    CustomString val11;
    CustomString val12;
    CustomString val13;
    CustomString val14;
    CustomString val15;
    CustomString val16;
    CustomString val17;
    CustomString val18;
    CustomString val19;
    CustomString val20;
    CustomString val21;
    CustomString val22;
    CustomString val23;
    CustomString val24;
    CustomString val25;
    CustomString val26;
    CustomString val27;
    CustomString val28;
    CustomString val29;
};
#pragma pack(pop)

#pragma pack(push, 1)
typedef struct _PTParam {
    int num;
    int type;
    double x;
    double y;
    double z;
    double xvel;
    double yvel;
    double zvel;
    double xacc;
    double yacc;
    double zacc;
    double xmax;
    double ymax;
    double zmax;
} PTParam;

typedef struct _PosList {
    int num;
    PTParam list[10];
} PosList;
#pragma pack(pop)

// 명령어 종류
typedef enum _Command {
	None=0,
	Stop=1, Estop,
	SetPosMode=3, SetJogMode, SetControlMode,
	SetJogParam=6, SetPosParam,
	SetJogState=8,
	SetPos=9,
	StartPosControl=10,
	GetTotal=11, GetPlatform, GetSensor, GetPos,
	InitPlatform=15,
    GetWhisper=16,
    SetSensor=17,
    SetJoyMode=18, SetPoseMode,
    SetTwist=20,
    SetPTParam=21,
    SetErrorMode=22,
    GetBattery=23,
} Command;

SensorState sensorStateOut;
PosStateOut posStateOut;
PlatformState platformStateOut;
BatteryState batteryStateOut;
TotalState totalStateOut;
WhisperState whisperStateOut;

StateInfo totalState;
StateInfo platformState;
StateInfo sensorState;
StateInfo posState;
StateInfo whisperState;
StateInfo batteryState;

int sock, client_sock;

// 시그널 핸들러(클라이언트 TCP 해제)
#include <signal.h>
int readWriteInfinite = 1;
int clientOpen = 1;

void sigpipe_handler(int sig) {
	// signal(SIGPIPE, sigpipe_handler);
	printf("[s] received SIGPIPE: %d \n", sig);
	readWriteInfinite = 0;
}

void sigint_handler(int sig) {
	// signal(SIGINT, sigint_handler);
	printf("[s] received SIGINT: %d \n", sig);
	close(sock);
	clientOpen = 0;
}

chatterbox::ChatOut chatOut_;
void chatOutCallBack(const chatterbox::ChatOut chatOut) {
    chatOut_ = chatOut;
}

chatterbox::WhisperOut whisperOut_;
void whisperOutCallBack(const chatterbox::WhisperOut whisperOut) {
    whisperOut_ = whisperOut;
}

void fThread(int* thread_rate, ros::Publisher *chatIn_pub) {
    ros::NodeHandlePtr node = boost::make_shared<ros::NodeHandle>();
    ros::Rate rate(*thread_rate);

    chatterbox::ChatIn chatIn;

    struct sockaddr_in addr, client_addr;
    #define BUF_SIZE	1024
    char buffer[BUF_SIZE];
    uint32_t command;
    Command cmd = None;
    int len, addr_len, recv_len;
    int ret;

    ros::Time now;

    printf("[s] clientOpen while start (%d line)\n", __LINE__);
    while (clientOpen && ros::ok()) {
        printf("[s] clientOpen while started (%d line)\n", __LINE__);
        // 소켓 열기
        if ((sock = socket(AF_INET, SOCK_STREAM, 0)) < 0) {
            // perror("[s] socket open error, ");
            printf("[s] socket open error\n");
            // return;
            continue;
        }
        printf("[s] socket open\n");

        // 소켓 설정(TCP, 포트)
        memset(&addr, 0x00, sizeof(addr));
        addr.sin_family = AF_INET;
        addr.sin_addr.s_addr = htonl(INADDR_ANY);

        int tcp_port;  // sin_port == uint16_t
        ros::param::get("~tcp_port", tcp_port);
        printf("[s] tcp_port: %d \n", tcp_port);
        addr.sin_port = htons(tcp_port);

        // time_wait 제거하기
        int option = 1;
        setsockopt(sock, SOL_SOCKET, SO_REUSEADDR, &option, sizeof(option));

        // 소켓 설정 등록
        if (bind(sock, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
            // perror("[s] bind error, ");
            printf("[s] bind error\n");
            // return;
            usleep(500000);
            continue;
        }
        printf("[s] bind registerd\n");

        // 시그널 핸들러 등록
        // signal(SIGINT, sigint_handler);

        // 시그널 핸들러 등록
        readWriteInfinite = 1;
        signal(SIGPIPE, sigpipe_handler);

        // 리슨을 타임아웃으로 설정하고 싶을 경우
        struct timeval timeout;
#define TCP_TIMEOUT_SEC 3
        timeout.tv_sec = TCP_TIMEOUT_SEC;
        timeout.tv_usec = 0;

        if (setsockopt (sock, SOL_SOCKET, SO_RCVTIMEO, (char *)&timeout, sizeof(timeout)) < 0) {
            // perror("[s] setsockopt failed, ");
            printf("[s] setsockopt failed\n");
        }

        // 연결 요청 대기
        if (listen(sock, 5) < 0) {
            // perror("[s] listen error, ");
            printf("[s] listen error\n");
            // return;
            usleep(500000);
            continue;
        } else {
            printf("[s] listen\n");
        }

        addr_len = sizeof(client_addr);

        printf("[s] waiting for client..\n");

        printf("[s] socket : %d\n", sock);
        // 연결 수락
        client_sock = accept(sock, (struct sockaddr *)&client_addr, (socklen_t*)&addr_len);
        if (client_sock < 0) {
            // perror("[s] accept error, ");
            printf("[s] accept error\n");
            // goto PROGRAM_END;
            // return;
            // printf("[s] socket : %d\n", sock);
            // printf("[s] client_sock: %d\n", client_sock);
            close(client_sock);
            close(sock);
            usleep(500000);
            continue;
        } else {
            printf("[s] clinet ip : %s\n", inet_ntoa(client_addr.sin_addr));
            printf("[s] client accept\n");
        }

        memset(buffer, '\0', sizeof(buffer));

        struct timeval time_now{};
        gettimeofday(&time_now, nullptr);
        time_t ts_now, ts_total, ts_platform, ts_sensor, ts_position, ts_battery, ts_whisper, ts_dummy, ts_total_once;
        ts_total_once = ts_dummy = ts_total = ts_platform = ts_sensor = ts_position = ts_battery = ts_whisper = ts_now = (time_now.tv_sec * 1000) + (time_now.tv_usec / 1000);

        char *u8_ptr;
        int byte_size;

        // 통신 데이터 입력
        memset((char*)&totalState, '\0', sizeof(totalState));
        memset((char*)&sensorState, '\0', sizeof(sensorState));
        memset((char*)&platformState, '\0', sizeof(platformState));
        memset((char*)&posState, '\0', sizeof(posState));
        memset((char*)&batteryState, '\0', sizeof(batteryState));

        // 통신 데이터 출력
        memset((char*)&sensorStateOut, '\0', sizeof(sensorStateOut));
        memset((char*)&posStateOut, '\0', sizeof(posStateOut));
        memset((char*)&platformStateOut, '\0', sizeof(platformStateOut));
        memset((char*)&batteryStateOut, '\0', sizeof(batteryStateOut));
        memset((char*)&totalStateOut, '\0', sizeof(totalStateOut));

        printf("[s] readWriteInfinite while start (%d line)\n", __LINE__);
        // 통신
        while (readWriteInfinite && ros::ok()) {
            // printf("[s] readWriteInfinite while started (%d line)\n", __LINE__);
            gettimeofday(&time_now, nullptr);
            ts_now = (time_now.tv_sec * 1000) + (time_now.tv_usec / 1000);
            #define DATA_LENGTH_BYTE	4
            // 버퍼개수 읽기(MSG_PEEK는 버퍼확인용)
            recv_len = recv(client_sock, buffer, DATA_LENGTH_BYTE, MSG_PEEK|MSG_DONTWAIT);
            if (recv_len >= DATA_LENGTH_BYTE) {
                #if 0
                printf("[s] 1st recv_len: %d, ", DATA_LENGTH_BYTE);
                for (int cnt=0; cnt<DATA_LENGTH_BYTE; cnt++) {
                    printf("%02x ", buffer[cnt]&0xff);
                }
                printf("\n");
                #endif

                int trail_len;
                // 뒤따르는 자료개수 확인
                memcpy(&trail_len, buffer, DATA_LENGTH_BYTE);
                // printf("[s] trail_len: %x, %d\n", trail_len, trail_len);

                // 뒤따르는 버퍼 읽기(MSG_PEEK는 버퍼확인용)
                recv_len = recv(client_sock, buffer, DATA_LENGTH_BYTE+trail_len, MSG_PEEK|MSG_DONTWAIT);
                if (recv_len >= DATA_LENGTH_BYTE+trail_len) {
                    #if 0
                    printf("[s] 2st recv_len: %d, ", trail_len);
                    for (int i=0; i<trail_len; i++) {
                        printf("%02x ", *(buffer+DATA_LENGTH_BYTE+i)&0xff);
                    }
                    printf("\n");
                    #endif

                    // 응답 프레임 송신
                    #if 1
                    char ack_buffer[BUF_SIZE];
                    // 버퍼개수 버리기(4바이트)
                    recv(client_sock, buffer, DATA_LENGTH_BYTE, 0);
                    memcpy(ack_buffer, buffer, DATA_LENGTH_BYTE);
                    // 뒤따르는 버퍼 읽기
                    recv(client_sock, buffer, trail_len, 0);
                    memcpy(ack_buffer+DATA_LENGTH_BYTE, buffer, trail_len);

                    // ACK 프레임으로 만들기
                    #define ACK_OK		1
                    #define ACK_IDX		6
                    #define ACK_NONE	0x00
                    #define ACK_ACK		0x01
                    #define ACK_RES		0x02
                    #define ACK_ONE		0x04
                    #define ACK_STR		0x08
                    #if ACK_OK
                    ack_buffer[ACK_IDX] = ACK_ACK;
                    #else
                    ack_buffer[ACK_IDX] = ACK_NONE;
                    #endif
                    send(client_sock, ack_buffer, DATA_LENGTH_BYTE+trail_len, 0);
                    #else
                    // 버퍼개수 버리기(4바이트)
                    recv(client_sock, buffer, DATA_LENGTH_BYTE, 0);
                    // 뒤따르는 버퍼 읽기
                    recv(client_sock, buffer, trail_len, 0);
                    #endif

                    // 명령어 바이트 읽기
                    #define COMMAND_SIZE	4
                    memcpy((char*)&command, buffer, COMMAND_SIZE);
                    cmd = (Command)command;
                    // printf("[s] command: %5d\tts: %ld\n", command, ts_now);

                    switch (cmd) {
                        case None:
                            printf("[s] None command\n");
                        break;
                        case Stop:
                            StopInfo stopInfoIn;
                            printf("[s] Receviced StopInfoIn, trail byte(%d)\n", trail_len-COMMAND_SIZE);
                            memcpy(&stopInfoIn, buffer+COMMAND_SIZE, sizeof(stopInfoIn));
                            printf("[s] stop: %d\n", stopInfoIn.stop);
                            platformStateOut.mode = Stop;

                            now = ros::Time::now();
                            chatIn.header.stamp = now;

                            chatIn.command = platformStateOut.mode;

                            chatIn_pub->publish(chatIn);
                        break;
                        case Estop:
                            printf("[s] Receviced Estop, trail byte(%d)\n", trail_len-COMMAND_SIZE);
                            platformStateOut.mode = Estop;

                            now = ros::Time::now();
                            chatIn.header.stamp = now;

                            chatIn.command = platformStateOut.mode;

                            chatIn_pub->publish(chatIn);
                        break;
                        case SetPosMode:
                            printf("[s] Receviced SetPosMode, trail byte(%d)\n", trail_len-COMMAND_SIZE);
                            platformStateOut.mode = SetPosMode;

                            now = ros::Time::now();
                            chatIn.header.stamp = now;

                            chatIn.command = platformStateOut.mode;

                            // chatIn_pub->publish(chatIn);
                        break;
                        case SetJogMode:
                            printf("[s] Receviced SetJogMode, trail byte(%d)\n", trail_len-COMMAND_SIZE);
                            platformStateOut.mode = SetJogMode;

                            now = ros::Time::now();
                            chatIn.header.stamp = now;

                            chatIn.command = platformStateOut.mode;

                            // chatIn_pub->publish(chatIn);
                        break;
                        case SetControlMode:
                            static uint32_t controlMode, controlModePre;
                            memcpy((char*)&controlMode, buffer+COMMAND_SIZE, sizeof(controlMode));

                            #if ACK_OK
                            if (controlMode == 0) {
                                printf("[s] Request SetControlMode, trail byte(%d)\n", trail_len-COMMAND_SIZE);
                                printf("[s] mode: %d\n", controlModePre);

                                byte_size = sizeof(command) + sizeof(controlMode);
                                u8_ptr = (char*)&byte_size;
                                memcpy(buffer, u8_ptr, sizeof(byte_size));
                                u8_ptr = (char*)&command;
                                memcpy(buffer+sizeof(byte_size), u8_ptr, sizeof(command));
                                u8_ptr = (char*)&controlModePre;
                                memcpy(buffer+sizeof(byte_size)+sizeof(command), u8_ptr, sizeof(controlModePre));
                                buffer[ACK_IDX] = ACK_RES;
                                send(client_sock, buffer, sizeof(byte_size)+sizeof(command)+sizeof(controlModePre), 0);
                            } else {
                                printf("[s] Receviced SetControlMode, trail byte(%d)\n", trail_len-COMMAND_SIZE);
                                printf("[s] mode: %d\n", controlMode);

                                controlModePre = controlMode;
                                platformStateOut.mode = controlMode;

                                now = ros::Time::now();
                                chatIn.header.stamp = now;

                                chatIn.command = platformStateOut.mode;

                                // chatIn_pub->publish(chatIn);
                            }
                            #endif
                        break;
                        case SetJogParam:
                            static VelParams jogVelParamsIn, jogVelParamsInPre;
                            memcpy(&jogVelParamsIn, buffer+COMMAND_SIZE, sizeof(jogVelParamsIn));

                            #if ACK_OK
                            static VelParams *jvpi = &jogVelParamsIn;
                            if (jvpi->x.vel == 0.0 && jvpi->x.acc == 0.0 && jvpi->x.vmax == 0.0 &&
                                jvpi->y.vel == 0.0 && jvpi->y.acc == 0.0 && jvpi->y.vmax == 0.0 &&
                                jvpi->rz.vel == 0.0 && jvpi->rz.acc == 0.0 && jvpi->rz.vmax == 0.0) {
                                printf("[s] Request SetJogParam, trail byte(%d)\n", trail_len-COMMAND_SIZE);
                                printf("[s] x.v: %.3lf, x.a: %.3lf, x.vm: %.3lf, y.v: %.3lf, y.a: %.3lf, y.vm: %.3lf, z.v: %.3lf, z.a: %.3lf, z.vm: %.3lf\n",
                                    jogVelParamsInPre.x.vel, jogVelParamsInPre.x.acc, jogVelParamsInPre.x.vmax,
                                    jogVelParamsInPre.y.vel, jogVelParamsInPre.y.acc, jogVelParamsInPre.y.vmax,
                                    jogVelParamsInPre.rz.vel, jogVelParamsInPre.rz.acc, jogVelParamsInPre.rz.vmax);

                                byte_size = sizeof(command) + sizeof(jogVelParamsIn);
                                u8_ptr = (char*)&byte_size;
                                memcpy(buffer, u8_ptr, sizeof(byte_size));
                                u8_ptr = (char*)&command;
                                memcpy(buffer+sizeof(byte_size), u8_ptr, sizeof(command));
                                u8_ptr = (char*)&jogVelParamsInPre;
                                memcpy(buffer+sizeof(byte_size)+sizeof(command), u8_ptr, sizeof(jogVelParamsInPre));
                                buffer[ACK_IDX] = ACK_RES;
                                send(client_sock, buffer, sizeof(byte_size)+sizeof(command)+sizeof(jogVelParamsInPre), 0);
                            } else {
                                printf("[s] Receviced SetJogParam, trail byte(%d)\n", trail_len-COMMAND_SIZE);
                                printf("[s] x.v: %.3lf, x.a: %.3lf, x.vm: %.3lf, y.v: %.3lf, y.a: %.3lf, y.vm: %.3lf, z.v: %.3lf, z.a: %.3lf, z.vm: %.3lf\n",
                                    jogVelParamsIn.x.vel, jogVelParamsIn.x.acc, jogVelParamsIn.x.vmax,
                                    jogVelParamsIn.y.vel, jogVelParamsIn.y.acc, jogVelParamsIn.y.vmax,
                                    jogVelParamsIn.rz.vel, jogVelParamsIn.rz.acc, jogVelParamsIn.rz.vmax);

                                memcpy((char*)&jogVelParamsInPre, (char*)&jogVelParamsIn, sizeof(jogVelParamsIn));
                            }
                            #endif
                        break;
                        case SetPosParam:
                            static VelParams posVelParamsIn, posVelParamsInPre;
                            memcpy(&posVelParamsIn, buffer+COMMAND_SIZE, sizeof(posVelParamsIn));

                            #if ACK_OK
                            static VelParams *pvpi = &posVelParamsIn;
                            if (pvpi->x.vel == 0.0 && pvpi->x.acc == 0.0 && pvpi->x.vmax == 0.0 &&
                                pvpi->y.vel == 0.0 && pvpi->y.acc == 0.0 && pvpi->y.vmax == 0.0 &&
                                pvpi->rz.vel == 0.0 && pvpi->rz.acc == 0.0 && pvpi->rz.vmax == 0.0) {
                                printf("[s] Request SetPosParam, trail byte(%d)\n", trail_len-COMMAND_SIZE);
                                printf("[s] x.v: %.3lf, x.a: %.3lf, x.vm: %.3lf, y.v: %.3lf, y.a: %.3lf, y.vm: %.3lf, z.v: %.3lf, z.a: %.3lf, z.vm: %.3lf\n",
                                    posVelParamsInPre.x.vel, posVelParamsInPre.x.acc, posVelParamsInPre.x.vmax,
                                    posVelParamsInPre.y.vel, posVelParamsInPre.y.acc, posVelParamsInPre.y.vmax,
                                    posVelParamsInPre.rz.vel, posVelParamsInPre.rz.acc, posVelParamsInPre.rz.vmax);

                                byte_size = sizeof(command) + sizeof(posVelParamsIn);
                                u8_ptr = (char*)&byte_size;
                                memcpy(buffer, u8_ptr, sizeof(byte_size));
                                u8_ptr = (char*)&command;
                                memcpy(buffer+sizeof(byte_size), u8_ptr, sizeof(command));
                                u8_ptr = (char*)&posVelParamsInPre;
                                memcpy(buffer+sizeof(byte_size)+sizeof(command), u8_ptr, sizeof(posVelParamsInPre));
                                buffer[ACK_IDX] = ACK_RES;
                                send(client_sock, buffer, sizeof(byte_size)+sizeof(command)+sizeof(posVelParamsInPre), 0);
                            } else {
                                printf("[s] Receviced SetPosParam, trail byte(%d)\n", trail_len-COMMAND_SIZE);
                                printf("[s] x.v: %.3lf, x.a: %.3lf, x.vm: %.3lf, y.v: %.3lf, y.a: %.3lf, y.vm: %.3lf, z.v: %.3lf, z.a: %.3lf, z.vm: %.3lf\n",
                                    posVelParamsIn.x.vel, posVelParamsIn.x.acc, posVelParamsIn.x.vmax,
                                    posVelParamsIn.y.vel, posVelParamsIn.y.acc, posVelParamsIn.y.vmax,
                                    posVelParamsIn.rz.vel, posVelParamsIn.rz.acc, posVelParamsIn.rz.vmax);

                                memcpy((char*)&posVelParamsInPre, (char*)&posVelParamsIn, sizeof(posVelParamsIn));
                            }
                            #endif
                        break;
                        case SetJogState:
                            JogInfo jogInfoIn;
                            memcpy(&jogInfoIn, buffer+COMMAND_SIZE, sizeof(jogInfoIn));
                            printf("[s] Receviced SetJogState, trail byte(%d)\n", trail_len-COMMAND_SIZE);
                            printf("[s] f: %d, b: %d, l: %d, r: %d, cw: %d, ccw: %d\n",
                                jogInfoIn.front, jogInfoIn.back, jogInfoIn.left, jogInfoIn.right, jogInfoIn.cw, jogInfoIn.ccw);

                            now = ros::Time::now();
                            chatIn.header.stamp = now;

                            chatIn.command = platformStateOut.mode;

                            chatIn.sensor = sensor;

                            chatIn.jogParam.jogInfo.front = jogInfoIn.front;
                            chatIn.jogParam.jogInfo.back = jogInfoIn.back;
                            chatIn.jogParam.jogInfo.left = jogInfoIn.left;
                            chatIn.jogParam.jogInfo.right = jogInfoIn.right;
                            chatIn.jogParam.jogInfo.cw = jogInfoIn.cw;
                            chatIn.jogParam.jogInfo.ccw = jogInfoIn.ccw;

                            chatIn.jogParam.velParams.x.acc = jogVelParamsIn.x.acc;
                            chatIn.jogParam.velParams.y.acc = jogVelParamsIn.y.acc;
                            chatIn.jogParam.velParams.rz.acc = jogVelParamsIn.rz.acc;
                            chatIn.jogParam.velParams.x.vel = jogVelParamsIn.x.vel;
                            chatIn.jogParam.velParams.y.vel = jogVelParamsIn.y.vel;
                            chatIn.jogParam.velParams.rz.vel = jogVelParamsIn.rz.vel;
                            chatIn.jogParam.velParams.x.vmax =jogVelParamsIn.x.vmax;
                            chatIn.jogParam.velParams.y.vmax =jogVelParamsIn.y.vmax;
                            chatIn.jogParam.velParams.rz.vmax =jogVelParamsIn.rz.vmax;

                            chatIn_pub->publish(chatIn);
                        break;
                        case SetPos:
                            static PosStateIn posStateIn, posStateInPre;
                            memcpy(&posStateIn, buffer+COMMAND_SIZE, sizeof(posStateIn));

                            #if ACK_OK
                            static PosStateIn *psi = &posStateIn;
                            if (psi->x == 0.0 && psi->y == 0.0 && psi->rz == 0.0) {
                                printf("[s] Request SetPos, trail byte(%d)\n", trail_len-COMMAND_SIZE);
                                printf("[s] pos.x: %.3lf, pos.y: %.3lf, pos.theta: %.3lf\n", posStateInPre.x, posStateInPre.y, posStateInPre.rz);

                                byte_size = sizeof(command) + sizeof(posStateIn);
                                u8_ptr = (char*)&byte_size;
                                memcpy(buffer, u8_ptr, sizeof(byte_size));
                                u8_ptr = (char*)&command;
                                memcpy(buffer+sizeof(byte_size), u8_ptr, sizeof(command));
                                u8_ptr = (char*)&posStateInPre;
                                memcpy(buffer+sizeof(byte_size)+sizeof(command), u8_ptr, sizeof(posStateInPre));
                                buffer[ACK_IDX] = ACK_RES;
                                send(client_sock, buffer, sizeof(byte_size)+sizeof(command)+sizeof(posStateInPre), 0);
                            } else {
                                printf("[s] Receviced SetPos, trail byte(%d)\n", trail_len-COMMAND_SIZE);
                                printf("[s] pos.x: %.3lf, pos.y: %.3lf, pos.theta: %.3lf\n", posStateIn.x, posStateIn.y, posStateIn.rz);

                                memcpy((char*)&posStateInPre, (char*)&posStateIn, sizeof(posStateIn));
                            }
                            #endif
                        break;
                        case StartPosControl:
                            printf("[s] Receviced StartPosControl, trail byte(%d)\n", trail_len-COMMAND_SIZE);

                            now = ros::Time::now();
                            chatIn.header.stamp = now;

                            chatIn.command = platformStateOut.mode;

                            chatIn.sensor = sensor;

                            chatIn.posParam.posState.x = posStateIn.x;
                            chatIn.posParam.posState.y = posStateIn.y;
                            chatIn.posParam.posState.rz = posStateIn.rz;

                            chatIn.posParam.velParams.x.acc = posVelParamsIn.x.acc;
                            chatIn.posParam.velParams.y.acc = posVelParamsIn.y.acc;
                            chatIn.posParam.velParams.rz.acc = posVelParamsIn.rz.acc;
                            chatIn.posParam.velParams.x.vel = posVelParamsIn.x.vel;
                            chatIn.posParam.velParams.y.vel = posVelParamsIn.y.vel;
                            chatIn.posParam.velParams.rz.vel = posVelParamsIn.rz.vel;
                            chatIn.posParam.velParams.x.vmax = posVelParamsIn.x.vmax;
                            chatIn.posParam.velParams.y.vmax = posVelParamsIn.y.vmax;
                            chatIn.posParam.velParams.rz.vmax = posVelParamsIn.rz.vmax;

                            chatIn_pub->publish(chatIn);
                        break;
                        case GetTotal:
                            // printf("[s] Receviced GetTotal, trail byte(%d)\n", trail_len-COMMAND_SIZE);
                            memcpy(&totalState, buffer+COMMAND_SIZE, sizeof(StateInfo));
                            // printf("[s] period: %d\n", totalState.period);

                            // printf("[s] ts_total_once time diff(ms) : %ld\n", ts_now-ts_total_once);
                            ts_total_once = ts_now;

                            if (totalState.period == 0) {
                            // 전체 송신할 바이트 계산
                                #if TCP_DUMMY_SEND
                                totalStateOut.sensor.front += 1;
                                totalStateOut.sensor.back += 2;
                                totalStateOut.sensor.right_front += 3;
                                totalStateOut.sensor.right_back += 4;

                                totalStateOut.pos.x += 5;
                                totalStateOut.pos.y += 6;
                                totalStateOut.pos.rz += 7;
                                totalStateOut.pos.ang += 8;

                                totalStateOut.platform.mode += 8;
                                totalStateOut.platform.state += 9;

                                totalStateOut.battery.voltage +=10;
                                totalStateOut.battery.current +=11;
                                totalStateOut.battery.percentage +=12;
                                #else
                                totalStateOut.sensor.front = chatOut_.sensorState.front;
                                totalStateOut.sensor.back = chatOut_.sensorState.back;
                                totalStateOut.sensor.right_front = chatOut_.sensorState.right_front;
                                totalStateOut.sensor.right_back = chatOut_.sensorState.right_back;

                                totalStateOut.pos.x = chatOut_.posState.x;
                                totalStateOut.pos.y = chatOut_.posState.y;
                                totalStateOut.pos.rz = chatOut_.posState.rz;
                                totalStateOut.pos.ang = chatOut_.posState.ang;

                                totalStateOut.platform.mode = chatOut_.platformState.mode;
                                totalStateOut.platform.state = chatOut_.platformState.state;

                                totalStateOut.battery.voltage = chatOut_.batteryState.voltage;
                                totalStateOut.battery.current = chatOut_.batteryState.current;
                                totalStateOut.battery.percentage = chatOut_.batteryState.percentage;
                                #endif

                                #if 0
                                byte_size = sizeof(command) + sizeof(totalStateOut);
                                u8_ptr = (char*)&byte_size;
                                send(client_sock, u8_ptr, sizeof(byte_size), 0);
                                u8_ptr = (char*)&command;
                                send(client_sock, u8_ptr, sizeof(command), 0);
                                u8_ptr = (char*)&totalStateOut;
                                send(client_sock, u8_ptr, sizeof(totalStateOut), 0);
                                #else
                                byte_size = sizeof(command) + sizeof(totalStateOut);
                                u8_ptr = (char*)&byte_size;
                                memcpy(buffer, u8_ptr, sizeof(byte_size));
                                u8_ptr = (char*)&command;
                                memcpy(buffer+sizeof(byte_size), u8_ptr, sizeof(command));
                                u8_ptr = (char*)&totalStateOut;
                                memcpy(buffer+sizeof(byte_size)+sizeof(command), u8_ptr, sizeof(totalStateOut));
                                buffer[ACK_IDX] = ACK_ONE;
                                send(client_sock, buffer, sizeof(byte_size)+sizeof(command)+sizeof(totalStateOut), 0);
                                #endif
                            } else {
                            }
                        break;
                        case GetPlatform:
                            // printf("[s] Receviced GetPlatform, trail byte(%d)\n", trail_len-COMMAND_SIZE);
                            memcpy(&platformState, buffer+COMMAND_SIZE, sizeof(StateInfo));
                            // printf("[s] period: %d\n", platformState.period);

                            if (platformState.period == 0) {
                                // 전체 송신할 바이트 계산
                                #if TCP_DUMMY_SEND
                                platformStateOut.mode += 1;
                                platformStateOut.state += 2;
                                #else
                                platformStateOut.mode = chatOut_.platformState.mode;
                                platformStateOut.state = chatOut_.platformState.state;
                                #endif

                                #if 0
                                byte_size = sizeof(command) + sizeof(platformStateOut);
                                u8_ptr = (char*)&byte_size;
                                send(client_sock, u8_ptr, sizeof(byte_size), 0);
                                u8_ptr = (char*)&command;
                                send(client_sock, u8_ptr, sizeof(command), 0);
                                u8_ptr = (char*)&platformStateOut;
                                send(client_sock, u8_ptr, sizeof(platformStateOut), 0);
                                #else
                                byte_size = sizeof(command) + sizeof(platformStateOut);
                                u8_ptr = (char*)&byte_size;
                                memcpy(buffer, u8_ptr, sizeof(byte_size));
                                u8_ptr = (char*)&command;
                                memcpy(buffer+sizeof(byte_size), u8_ptr, sizeof(command));
                                u8_ptr = (char*)&platformStateOut;
                                memcpy(buffer+sizeof(byte_size)+sizeof(command), u8_ptr, sizeof(platformStateOut));
                                buffer[ACK_IDX] = ACK_ONE;
                                send(client_sock, buffer, sizeof(byte_size)+sizeof(command)+sizeof(platformStateOut), 0);
                                #endif
                            } else {
                            }
                        break;
                        case GetSensor:
                            // printf("[s] Receviced GetSensor, trail byte(%d)\n", trail_len-COMMAND_SIZE);
                            memcpy(&sensorState, buffer+COMMAND_SIZE, sizeof(StateInfo));
                            // printf("[s] period: %d\n", sensorState.period);

                            if (sensorState.period == 0) {
                                // 전체 송신할 바이트 계산
                                #if TCP_DUMMY_SEND
                                sensorStateOut.front += 1;
                                sensorStateOut.back += 2;
                                sensorStateOut.right_front += 3;
                                sensorStateOut.right_back += 4;
                                #else
                                sensorStateOut.front = chatOut_.sensorState.front;
                                sensorStateOut.back = chatOut_.sensorState.back;
                                sensorStateOut.right_front = chatOut_.sensorState.right_front;
                                sensorStateOut.right_back = chatOut_.sensorState.right_back;
                                #endif

                                #if 0
                                byte_size = sizeof(command) + sizeof(sensorStateOut);
                                u8_ptr = (char*)&byte_size;
                                send(client_sock, u8_ptr, sizeof(byte_size), 0);
                                u8_ptr = (char*)&command;
                                send(client_sock, u8_ptr, sizeof(command), 0);
                                u8_ptr = (char*)&sensorStateOut;
                                send(client_sock, u8_ptr, sizeof(sensorStateOut), 0);
                                #else
                                byte_size = sizeof(command) + sizeof(sensorStateOut);
                                u8_ptr = (char*)&byte_size;
                                memcpy(buffer, u8_ptr, sizeof(byte_size));
                                u8_ptr = (char*)&command;
                                memcpy(buffer+sizeof(byte_size), u8_ptr, sizeof(command));
                                u8_ptr = (char*)&sensorStateOut;
                                memcpy(buffer+sizeof(byte_size)+sizeof(command), u8_ptr, sizeof(sensorStateOut));
                                buffer[ACK_IDX] = ACK_ONE;
                                send(client_sock, buffer, sizeof(byte_size)+sizeof(command)+sizeof(sensorStateOut), 0);
                                #endif
                            } else {
                            }
                        break;
                        case GetPos:
                            // printf("[s] Receviced GetPos, trail byte(%d)\n", trail_len-COMMAND_SIZE);
                            memcpy(&posState, buffer+COMMAND_SIZE, sizeof(StateInfo));
                            // printf("[s] period: %d\n", posState.period);

                            if (posState.period == 0) {
                                // 전체 송신할 바이트 계산
                                #if TCP_DUMMY_SEND
                                posStateOut.x += 1;
                                posStateOut.y += 2;
                                posStateOut.rz += 3;
                                posStateOut.ang += 4;
                                #else
                                posStateOut.x = chatOut_.posState.x;
                                posStateOut.y = chatOut_.posState.y;
                                posStateOut.rz = chatOut_.posState.rz;
                                posStateOut.ang = chatOut_.posState.ang;
                                #endif

                                #if 0
                                byte_size = sizeof(command) + sizeof(posStateOut);
                                u8_ptr = (char*)&byte_size;
                                send(client_sock, u8_ptr, sizeof(byte_size), 0);
                                u8_ptr = (char*)&command;
                                send(client_sock, u8_ptr, sizeof(command), 0);
                                u8_ptr = (char*)&posStateOut;
                                send(client_sock, u8_ptr, sizeof(posStateOut), 0);
                                #else
                                byte_size = sizeof(command) + sizeof(posStateOut);
                                u8_ptr = (char*)&byte_size;
                                memcpy(buffer, u8_ptr, sizeof(byte_size));
                                u8_ptr = (char*)&command;
                                memcpy(buffer+sizeof(byte_size), u8_ptr, sizeof(command));
                                u8_ptr = (char*)&posStateOut;
                                memcpy(buffer+sizeof(byte_size)+sizeof(command), u8_ptr, sizeof(posStateOut));
                                buffer[ACK_IDX] = ACK_ONE;
                                send(client_sock, buffer, sizeof(byte_size)+sizeof(command)+sizeof(posStateOut), 0);
                                #endif
                            } else {
                            }
                        break;
                        case InitPlatform:
                            printf("[s] Receviced InitPlatform, trail byte(%d)\n", trail_len-COMMAND_SIZE);
                            platformStateOut.mode = InitPlatform;

                            now = ros::Time::now();
                            chatIn.header.stamp = now;

                            chatIn.command = platformStateOut.mode;

                            chatIn_pub->publish(chatIn);
                        break;
                        case GetWhisper:
                            // printf("[s] Receviced GetWhisper, trail byte(%d)\n", trail_len-COMMAND_SIZE);
                            memcpy(&whisperState, buffer+COMMAND_SIZE, sizeof(StateInfo));
                            // printf("[s] period: %d\n", whisperState.period);

                            if (whisperState.period == 0) {
                                // 전체 송신할 바이트 계산
                                #if TCP_DUMMY_SEND
                                whisperOut_.val00.subject = "val00val00val00val00";
                                memset(whisperStateOut.val00.subject, '\0', SUBJECT_SIZE);
                                memcpy(whisperStateOut.val00.subject, whisperOut_.val00.subject.c_str(), whisperOut_.val00.subject.length()>SUBJECT_SIZE?SUBJECT_SIZE:whisperOut_.val00.subject.length());
                                whisperStateOut.val00.value += 1;
                                whisperOut_.val19.subject = "val19val19val19val19";
                                memset(whisperStateOut.val19.subject, '\0', SUBJECT_SIZE);
                                memcpy(whisperStateOut.val19.subject, whisperOut_.val19.subject.c_str(), whisperOut_.val19.subject.length()>SUBJECT_SIZE?SUBJECT_SIZE:whisperOut_.val19.subject.length());
                                whisperStateOut.val19.value += 2;
                                #else
                                memset(whisperStateOut.val00.subject, '\0', SUBJECT_SIZE);
                                memcpy(whisperStateOut.val00.subject, whisperOut_.val00.subject.c_str(), whisperOut_.val00.subject.length()>SUBJECT_SIZE?SUBJECT_SIZE:whisperOut_.val00.subject.length());
                                whisperStateOut.val00.value = whisperOut_.val00.value;
                                memset(whisperStateOut.val01.subject, '\0', SUBJECT_SIZE);
                                memcpy(whisperStateOut.val01.subject, whisperOut_.val01.subject.c_str(), whisperOut_.val01.subject.length()>SUBJECT_SIZE?SUBJECT_SIZE:whisperOut_.val01.subject.length());
                                whisperStateOut.val01.value = whisperOut_.val01.value;
                                memset(whisperStateOut.val02.subject, '\0', SUBJECT_SIZE);
                                memcpy(whisperStateOut.val02.subject, whisperOut_.val02.subject.c_str(), whisperOut_.val02.subject.length()>SUBJECT_SIZE?SUBJECT_SIZE:whisperOut_.val02.subject.length());
                                whisperStateOut.val02.value = whisperOut_.val02.value;
                                memset(whisperStateOut.val03.subject, '\0', SUBJECT_SIZE);
                                memcpy(whisperStateOut.val03.subject, whisperOut_.val03.subject.c_str(), whisperOut_.val03.subject.length()>SUBJECT_SIZE?SUBJECT_SIZE:whisperOut_.val03.subject.length());
                                whisperStateOut.val03.value = whisperOut_.val03.value;
                                memset(whisperStateOut.val04.subject, '\0', SUBJECT_SIZE);
                                memcpy(whisperStateOut.val04.subject, whisperOut_.val04.subject.c_str(), whisperOut_.val04.subject.length()>SUBJECT_SIZE?SUBJECT_SIZE:whisperOut_.val04.subject.length());
                                whisperStateOut.val04.value = whisperOut_.val04.value;
                                memset(whisperStateOut.val05.subject, '\0', SUBJECT_SIZE);
                                memcpy(whisperStateOut.val05.subject, whisperOut_.val05.subject.c_str(), whisperOut_.val05.subject.length()>SUBJECT_SIZE?SUBJECT_SIZE:whisperOut_.val05.subject.length());
                                whisperStateOut.val05.value = whisperOut_.val05.value;
                                memset(whisperStateOut.val06.subject, '\0', SUBJECT_SIZE);
                                memcpy(whisperStateOut.val06.subject, whisperOut_.val06.subject.c_str(), whisperOut_.val06.subject.length()>SUBJECT_SIZE?SUBJECT_SIZE:whisperOut_.val06.subject.length());
                                whisperStateOut.val06.value = whisperOut_.val06.value;
                                memset(whisperStateOut.val07.subject, '\0', SUBJECT_SIZE);
                                memcpy(whisperStateOut.val07.subject, whisperOut_.val07.subject.c_str(), whisperOut_.val07.subject.length()>SUBJECT_SIZE?SUBJECT_SIZE:whisperOut_.val07.subject.length());
                                whisperStateOut.val07.value = whisperOut_.val07.value;
                                memset(whisperStateOut.val08.subject, '\0', SUBJECT_SIZE);
                                memcpy(whisperStateOut.val08.subject, whisperOut_.val08.subject.c_str(), whisperOut_.val08.subject.length()>SUBJECT_SIZE?SUBJECT_SIZE:whisperOut_.val08.subject.length());
                                whisperStateOut.val08.value = whisperOut_.val08.value;
                                memset(whisperStateOut.val09.subject, '\0', SUBJECT_SIZE);
                                memcpy(whisperStateOut.val09.subject, whisperOut_.val09.subject.c_str(), whisperOut_.val09.subject.length()>SUBJECT_SIZE?SUBJECT_SIZE:whisperOut_.val09.subject.length());
                                whisperStateOut.val09.value = whisperOut_.val09.value;
                                memset(whisperStateOut.val10.subject, '\0', SUBJECT_SIZE);
                                memcpy(whisperStateOut.val10.subject, whisperOut_.val10.subject.c_str(), whisperOut_.val10.subject.length()>SUBJECT_SIZE?SUBJECT_SIZE:whisperOut_.val10.subject.length());
                                whisperStateOut.val10.value = whisperOut_.val10.value;
                                memset(whisperStateOut.val11.subject, '\0', SUBJECT_SIZE);
                                memcpy(whisperStateOut.val11.subject, whisperOut_.val11.subject.c_str(), whisperOut_.val11.subject.length()>SUBJECT_SIZE?SUBJECT_SIZE:whisperOut_.val11.subject.length());
                                whisperStateOut.val11.value = whisperOut_.val11.value;
                                memset(whisperStateOut.val12.subject, '\0', SUBJECT_SIZE);
                                memcpy(whisperStateOut.val12.subject, whisperOut_.val12.subject.c_str(), whisperOut_.val12.subject.length()>SUBJECT_SIZE?SUBJECT_SIZE:whisperOut_.val12.subject.length());
                                whisperStateOut.val12.value = whisperOut_.val12.value;
                                memset(whisperStateOut.val13.subject, '\0', SUBJECT_SIZE);
                                memcpy(whisperStateOut.val13.subject, whisperOut_.val13.subject.c_str(), whisperOut_.val13.subject.length()>SUBJECT_SIZE?SUBJECT_SIZE:whisperOut_.val13.subject.length());
                                whisperStateOut.val13.value = whisperOut_.val13.value;
                                memset(whisperStateOut.val14.subject, '\0', SUBJECT_SIZE);
                                memcpy(whisperStateOut.val14.subject, whisperOut_.val14.subject.c_str(), whisperOut_.val14.subject.length()>SUBJECT_SIZE?SUBJECT_SIZE:whisperOut_.val14.subject.length());
                                whisperStateOut.val14.value = whisperOut_.val14.value;
                                memset(whisperStateOut.val15.subject, '\0', SUBJECT_SIZE);
                                memcpy(whisperStateOut.val15.subject, whisperOut_.val15.subject.c_str(), whisperOut_.val15.subject.length()>SUBJECT_SIZE?SUBJECT_SIZE:whisperOut_.val15.subject.length());
                                whisperStateOut.val15.value = whisperOut_.val15.value;
                                memset(whisperStateOut.val16.subject, '\0', SUBJECT_SIZE);
                                memcpy(whisperStateOut.val16.subject, whisperOut_.val16.subject.c_str(), whisperOut_.val16.subject.length()>SUBJECT_SIZE?SUBJECT_SIZE:whisperOut_.val16.subject.length());
                                whisperStateOut.val16.value = whisperOut_.val16.value;
                                memset(whisperStateOut.val17.subject, '\0', SUBJECT_SIZE);
                                memcpy(whisperStateOut.val17.subject, whisperOut_.val17.subject.c_str(), whisperOut_.val17.subject.length()>SUBJECT_SIZE?SUBJECT_SIZE:whisperOut_.val17.subject.length());
                                whisperStateOut.val17.value = whisperOut_.val17.value;
                                memset(whisperStateOut.val18.subject, '\0', SUBJECT_SIZE);
                                memcpy(whisperStateOut.val18.subject, whisperOut_.val18.subject.c_str(), whisperOut_.val18.subject.length()>SUBJECT_SIZE?SUBJECT_SIZE:whisperOut_.val18.subject.length());
                                whisperStateOut.val18.value = whisperOut_.val18.value;
                                memset(whisperStateOut.val19.subject, '\0', SUBJECT_SIZE);
                                memcpy(whisperStateOut.val19.subject, whisperOut_.val19.subject.c_str(), whisperOut_.val19.subject.length()>SUBJECT_SIZE?SUBJECT_SIZE:whisperOut_.val19.subject.length());
                                whisperStateOut.val19.value = whisperOut_.val19.value;
                                memset(whisperStateOut.val20.subject, '\0', SUBJECT_SIZE);
                                memcpy(whisperStateOut.val20.subject, whisperOut_.val20.subject.c_str(), whisperOut_.val20.subject.length()>SUBJECT_SIZE?SUBJECT_SIZE:whisperOut_.val20.subject.length());
                                whisperStateOut.val20.value = whisperOut_.val20.value;
                                memset(whisperStateOut.val21.subject, '\0', SUBJECT_SIZE);
                                memcpy(whisperStateOut.val21.subject, whisperOut_.val21.subject.c_str(), whisperOut_.val21.subject.length()>SUBJECT_SIZE?SUBJECT_SIZE:whisperOut_.val21.subject.length());
                                whisperStateOut.val21.value = whisperOut_.val21.value;
                                memset(whisperStateOut.val22.subject, '\0', SUBJECT_SIZE);
                                memcpy(whisperStateOut.val22.subject, whisperOut_.val22.subject.c_str(), whisperOut_.val22.subject.length()>SUBJECT_SIZE?SUBJECT_SIZE:whisperOut_.val22.subject.length());
                                whisperStateOut.val22.value = whisperOut_.val22.value;
                                memset(whisperStateOut.val23.subject, '\0', SUBJECT_SIZE);
                                memcpy(whisperStateOut.val23.subject, whisperOut_.val23.subject.c_str(), whisperOut_.val23.subject.length()>SUBJECT_SIZE?SUBJECT_SIZE:whisperOut_.val23.subject.length());
                                whisperStateOut.val23.value = whisperOut_.val23.value;
                                memset(whisperStateOut.val24.subject, '\0', SUBJECT_SIZE);
                                memcpy(whisperStateOut.val24.subject, whisperOut_.val24.subject.c_str(), whisperOut_.val24.subject.length()>SUBJECT_SIZE?SUBJECT_SIZE:whisperOut_.val24.subject.length());
                                whisperStateOut.val24.value = whisperOut_.val24.value;
                                memset(whisperStateOut.val25.subject, '\0', SUBJECT_SIZE);
                                memcpy(whisperStateOut.val25.subject, whisperOut_.val25.subject.c_str(), whisperOut_.val25.subject.length()>SUBJECT_SIZE?SUBJECT_SIZE:whisperOut_.val25.subject.length());
                                whisperStateOut.val25.value = whisperOut_.val25.value;
                                memset(whisperStateOut.val26.subject, '\0', SUBJECT_SIZE);
                                memcpy(whisperStateOut.val26.subject, whisperOut_.val26.subject.c_str(), whisperOut_.val26.subject.length()>SUBJECT_SIZE?SUBJECT_SIZE:whisperOut_.val26.subject.length());
                                whisperStateOut.val26.value = whisperOut_.val26.value;
                                memset(whisperStateOut.val27.subject, '\0', SUBJECT_SIZE);
                                memcpy(whisperStateOut.val27.subject, whisperOut_.val27.subject.c_str(), whisperOut_.val27.subject.length()>SUBJECT_SIZE?SUBJECT_SIZE:whisperOut_.val27.subject.length());
                                whisperStateOut.val27.value = whisperOut_.val27.value;
                                memset(whisperStateOut.val28.subject, '\0', SUBJECT_SIZE);
                                memcpy(whisperStateOut.val28.subject, whisperOut_.val28.subject.c_str(), whisperOut_.val28.subject.length()>SUBJECT_SIZE?SUBJECT_SIZE:whisperOut_.val28.subject.length());
                                whisperStateOut.val28.value = whisperOut_.val28.value;
                                memset(whisperStateOut.val29.subject, '\0', SUBJECT_SIZE);
                                memcpy(whisperStateOut.val29.subject, whisperOut_.val29.subject.c_str(), whisperOut_.val29.subject.length()>SUBJECT_SIZE?SUBJECT_SIZE:whisperOut_.val29.subject.length());
                                whisperStateOut.val29.value = whisperOut_.val29.value;
                                #endif

                                #if 0
                                #else
                                byte_size = sizeof(command) + sizeof(whisperStateOut);
                                u8_ptr = (char*)&byte_size;
                                memcpy(buffer, u8_ptr, sizeof(byte_size));
                                u8_ptr = (char*)&command;
                                memcpy(buffer+sizeof(byte_size), u8_ptr, sizeof(command));
                                u8_ptr = (char*)&whisperStateOut;
                                memcpy(buffer+sizeof(byte_size)+sizeof(command), u8_ptr, sizeof(whisperStateOut));
                                buffer[ACK_IDX] = ACK_ONE;
                                send(client_sock, buffer, sizeof(byte_size)+sizeof(command)+sizeof(whisperStateOut), 0);
                                #endif
                            } else {
                            }
                        break;
                        case SetSensor:
                            printf("[s] Receviced SetSensor, trail byte(%d)\n", trail_len-COMMAND_SIZE);
                            memcpy(&sensor, buffer+COMMAND_SIZE, sizeof(uint32_t));
                            printf("[s] sensor: %d\n", sensor);

                            now = ros::Time::now();
                            chatIn.header.stamp = now;

                            chatIn.sensor = sensor;

                            // chatIn_pub->publish(chatIn);
                        break;
                        case SetJoyMode:
                            printf("[s] Receviced SetJoyMode, trail byte(%d)\n", trail_len-COMMAND_SIZE);
                            #if 0
                            platformStateOut.mode = SetJoyMode;
                            #else
                            // recipelab 패키지에서 정의된 값임
                            #define Joy 5
                            platformStateOut.mode = Joy;
                            #endif

                            now = ros::Time::now();
                            chatIn.header.stamp = now;

                            chatIn.command = platformStateOut.mode;

                            // chatIn_pub->publish(chatIn);
                        break;
                        case SetPoseMode:
                            printf("[s] Receviced SetPoseMode, trail byte(%d)\n", trail_len-COMMAND_SIZE);
                            platformStateOut.mode = SetPoseMode;

                            now = ros::Time::now();
                            chatIn.header.stamp = now;

                            chatIn.command = platformStateOut.mode;

                            // chatIn_pub->publish(chatIn);
                        break;
                        case SetTwist:
                            printf("[s] Receviced SetTwist, trail byte(%d)\n", trail_len-COMMAND_SIZE);
                            
                            static geometry_msgs::Twist twist;
                            memcpy(&twist, buffer+COMMAND_SIZE, sizeof(twist));
                            chatIn.twist = twist;

                            now = ros::Time::now();
                            chatIn.header.stamp = now;

                            chatIn.command = platformStateOut.mode;

                            chatIn.sensor = sensor;

                            chatIn.jogParam.velParams.x.acc = jogVelParamsIn.x.acc;
                            chatIn.jogParam.velParams.y.acc = jogVelParamsIn.y.acc;
                            chatIn.jogParam.velParams.rz.acc = jogVelParamsIn.rz.acc;
                            chatIn.jogParam.velParams.x.vel = jogVelParamsIn.x.vel;
                            chatIn.jogParam.velParams.y.vel = jogVelParamsIn.y.vel;
                            chatIn.jogParam.velParams.rz.vel = jogVelParamsIn.rz.vel;
                            chatIn.jogParam.velParams.x.vmax =jogVelParamsIn.x.vmax;
                            chatIn.jogParam.velParams.y.vmax =jogVelParamsIn.y.vmax;
                            chatIn.jogParam.velParams.rz.vmax =jogVelParamsIn.rz.vmax;

                            chatIn_pub->publish(chatIn);
                        break;
                        case SetPTParam:
                            printf("[s] Receviced SetPTParam, trail byte(%d)\n", trail_len-COMMAND_SIZE);
                            
                            typedef struct _OrderList {
                                int num;
                                int order[20];
                            } OrderList;
                            static OrderList orderList;
                            
                            memcpy(&orderList.num, buffer+COMMAND_SIZE, sizeof(int));
                            printf("order num: %d\n", orderList.num);
                            memcpy(orderList.order, buffer+COMMAND_SIZE+sizeof(int), sizeof(int)*orderList.num);
                            for (int i=0; i<orderList.num; i++) {
                                printf("#%d,", orderList.order[i]);
                            }
                            printf("\n");

                            static PosList posList;
                            
                            memcpy(&posList.num, buffer+COMMAND_SIZE+sizeof(int)+sizeof(int)*orderList.num, sizeof(int));
                            printf("list num: %d\n", posList.num);
                            memcpy(posList.list, buffer+COMMAND_SIZE+sizeof(int)+sizeof(int)*orderList.num+sizeof(int), sizeof(PTParam)*posList.num);
                            for (int i=0; i<posList.num; i++) {
                                printf("%02d %02d %3.1f %3.1f %3.1f %3.1f %3.1f %3.1f %3.1f %3.1f %3.1f %3.1f %3.1f %3.1f\n",
                                    posList.list[i].num, posList.list[i].type,
                                    posList.list[i].x, posList.list[i].y, posList.list[i].z,
                                    posList.list[i].xvel, posList.list[i].yvel, posList.list[i].zvel,
                                    posList.list[i].xacc, posList.list[i].yacc, posList.list[i].zacc,
                                    posList.list[i].xmax, posList.list[i].ymax, posList.list[i].zmax);
                            }
                            static int temp;
                            memcpy(&temp, buffer+COMMAND_SIZE+sizeof(int)+sizeof(int)*orderList.num+sizeof(int)+sizeof(PTParam)*posList.num, sizeof(int));
                            printf("temp: %d", temp);

                            // now = ros::Time::now();
                            // chatIn.header.stamp = now;

                            // chatIn.command = platformStateOut.mode;

                            // chatIn.sensor = sensor;

                            // chatIn.jogParam.velParams.x.acc = jogVelParamsIn.x.acc;
                            // chatIn.jogParam.velParams.y.acc = jogVelParamsIn.y.acc;
                            // chatIn.jogParam.velParams.rz.acc = jogVelParamsIn.rz.acc;
                            // chatIn.jogParam.velParams.x.vel = jogVelParamsIn.x.vel;
                            // chatIn.jogParam.velParams.y.vel = jogVelParamsIn.y.vel;
                            // chatIn.jogParam.velParams.rz.vel = jogVelParamsIn.rz.vel;
                            // chatIn.jogParam.velParams.x.vmax =jogVelParamsIn.x.vmax;
                            // chatIn.jogParam.velParams.y.vmax =jogVelParamsIn.y.vmax;
                            // chatIn.jogParam.velParams.rz.vmax =jogVelParamsIn.rz.vmax;

                            // chatIn_pub->publish(chatIn);
                        break;
                        case SetErrorMode:
                            printf("[s] Receviced SetErrorMode, trail byte(%d)\n", trail_len-COMMAND_SIZE);
                            // platformStateOut.mode = SetErrorMode;
                            platformStateOut.mode = 14;

                            now = ros::Time::now();
                            chatIn.header.stamp = now;

                            chatIn.command = platformStateOut.mode;

                            chatIn_pub->publish(chatIn);
                        break;
                        case GetBattery:
                            // printf("[s] Receviced GetBattery, trail byte(%d)\n", trail_len-COMMAND_SIZE);
                            memcpy(&batteryState, buffer+COMMAND_SIZE, sizeof(StateInfo));
                            // printf("[s] period: %d\n", batteryState.period);

                            if (batteryState.period == 0) {
                                // 전체 송신할 바이트 계산
                                #if TCP_DUMMY_SEND
                                batteryStateOut.voltage += 1;
                                batteryStateOut.current += 2;
                                batteryStateOut.percentage += 3;
                                #else
                                batteryStateOut.voltage = chatOut_.batteryState.voltage;
                                batteryStateOut.current = chatOut_.batteryState.current;
                                batteryStateOut.percentage = chatOut_.batteryState.percentage;
                                #endif

                                #if 0
                                byte_size = sizeof(command) + sizeof(batteryStateOut);
                                u8_ptr = (char*)&byte_size;
                                send(client_sock, u8_ptr, sizeof(byte_size), 0);
                                u8_ptr = (char*)&command;
                                send(client_sock, u8_ptr, sizeof(command), 0);
                                u8_ptr = (char*)&batteryStateOut;
                                send(client_sock, u8_ptr, sizeof(batteryStateOut), 0);
                                #else
                                byte_size = sizeof(command) + sizeof(batteryStateOut);
                                u8_ptr = (char*)&byte_size;
                                memcpy(buffer, u8_ptr, sizeof(byte_size));
                                u8_ptr = (char*)&command;
                                memcpy(buffer+sizeof(byte_size), u8_ptr, sizeof(command));
                                u8_ptr = (char*)&batteryStateOut;
                                memcpy(buffer+sizeof(byte_size)+sizeof(command), u8_ptr, sizeof(batteryStateOut));
                                buffer[ACK_IDX] = ACK_ONE;
                                send(client_sock, buffer, sizeof(byte_size)+sizeof(command)+sizeof(batteryStateOut), 0);
                                #endif
                            } else {
                            }
                        break;
                        default:
                            printf("[s] unknown command\n");
                    }
                } else {
                }
            } else if (recv_len == 0) {
                // 참고자료
                // recv를 호출했을때 0이 리턴되거나 send를 호출했을때 -1로 에러리턴이면 상대편 연결이 종료된 것

                // 커넥션중일 때는 MSG_PEEK|MSG_DONTWAIT에서 아무것도 송신안할 경우 -1이 발생하였음
                // 커넥션을 강제해제했을 경우 0이 발생하였음
                // MSG_DONTWIT를 추가하지 않았을 경우에는 무한대기가 되었음
                // 우선 이케이스에는 0을 커넥션 종료로 처리함
                printf("[s] client disconnect error, recv: %d \n", recv_len);
                readWriteInfinite = 0;
                continue;
            } else {
                usleep(1000);
            }

            #if 0
            static PosStateOut posStateTest = {0.0, 10.0, 0.0, 0.0};
            #if 1
            #define SEND_PERIOD_MS	10
            if (SEND_PERIOD_MS <= (ts_now-ts_dummy)) {
                ts_dummy = ts_now;

                posStateTest.x += 0.1;
                posStateTest.y -= 0.1;
                posStateTest.rz = posStateTest.x + 1;
                posStateTest.ang = posStateTest.y + 2;
                if (posStateTest.x > 10.0) {
                    posStateTest.x = 0.0;
                    posStateTest.y = 10.0;
                }

                command = SetPos;
                // 전체 송신할 바이트 계산
                byte_size = sizeof(command) + sizeof(posStateTest);
                u8_ptr = (char*)&byte_size;
                send(client_sock, u8_ptr, sizeof(byte_size), 0);

                u8_ptr = (char*)&command;
                send(client_sock, u8_ptr, sizeof(command), 0);

                // printf("[s] count: %d\tdec: %d\toffset: %d\n", t.count, t.dec, t.offset);
                u8_ptr = (char*)&posStateTest;
                send(client_sock, u8_ptr, sizeof(posStateTest), 0);

                // printf("[s] timestamp now : %ld\n", ts_now);
            }
            #else
            posStateTest.x += 0.1;
            posStateTest.y -= 0.1;
            posStateTest.rz = posStateTest.x + 1;
            posStateTest.ang = posStateTest.y + 2;
            if (posStateTest.x > 10.0) {
                posStateTest.x = 0.0;
                posStateTest.y = 10.0;
            }

            command = SetPos;
            byte_size = sizeof(command) + sizeof(posStateTest);
            u8_ptr = (char*)&byte_size;
            memcpy(buffer, u8_ptr, sizeof(byte_size));
            u8_ptr = (char*)&command;
            memcpy(buffer+sizeof(byte_size), u8_ptr, sizeof(command));
            u8_ptr = (char*)&posStateTest;
            memcpy(buffer+sizeof(byte_size)+sizeof(command), u8_ptr, sizeof(posStateTest));
            send(client_sock, buffer, sizeof(byte_size)+sizeof(command)+sizeof(posStateTest), 0);

            usleep(10000);

            // printf("[s] usleep now : %ld\n", ts_now);
            #endif
            #endif

            // printf("[s] milli: %ld %ld\n", msecs_time, sizeof(msecs_time));

            if (totalState.period !=0 && totalState.period <= (ts_now-ts_total)) {
                ts_total = ts_now;

                // 전체 송신할 바이트 계산
                #if TCP_DUMMY_SEND
                totalStateOut.sensor.front += 1;
                totalStateOut.sensor.back += 2;
                totalStateOut.sensor.right_front += 3;
                totalStateOut.sensor.right_back += 4;

                totalStateOut.pos.x += 5;
                totalStateOut.pos.y += 6;
                totalStateOut.pos.rz += 7;
                totalStateOut.pos.ang += 8;

                totalStateOut.platform.mode += 8;
                totalStateOut.platform.state += 9;

                totalStateOut.battery.voltage += 10;
                totalStateOut.battery.current += 11;
                totalStateOut.battery.percentage += 12;
                #else
                totalStateOut.sensor.front = chatOut_.sensorState.front;
                totalStateOut.sensor.back = chatOut_.sensorState.back;
                totalStateOut.sensor.right_front = chatOut_.sensorState.right_front;
                totalStateOut.sensor.right_back = chatOut_.sensorState.right_back;

                totalStateOut.pos.x = chatOut_.posState.x;
                totalStateOut.pos.y = chatOut_.posState.y;
                totalStateOut.pos.rz = chatOut_.posState.rz;
                totalStateOut.pos.ang = chatOut_.posState.ang;

                totalStateOut.platform.mode = chatOut_.platformState.mode;
                totalStateOut.platform.state = chatOut_.platformState.state;

                totalStateOut.battery.voltage = chatOut_.batteryState.voltage;
                totalStateOut.battery.current = chatOut_.batteryState.current;
                totalStateOut.battery.percentage = chatOut_.batteryState.percentage;
                #endif

                command = GetTotal;

                byte_size = sizeof(command) + sizeof(totalStateOut);
                u8_ptr = (char*)&byte_size;
                memcpy(buffer, u8_ptr, sizeof(byte_size));
                u8_ptr = (char*)&command;
                memcpy(buffer+sizeof(byte_size), u8_ptr, sizeof(command));
                u8_ptr = (char*)&totalStateOut;
                memcpy(buffer+sizeof(byte_size)+sizeof(command), u8_ptr, sizeof(totalStateOut));
                buffer[ACK_IDX] = ACK_STR;
                send(client_sock, buffer, sizeof(byte_size)+sizeof(command)+sizeof(totalStateOut), 0);
            }

            if (platformState.period !=0 && platformState.period <= (ts_now-ts_platform)) {
                ts_platform = ts_now;

                // 전체 송신할 바이트 계산
                #if TCP_DUMMY_SEND
                platformStateOut.mode += 1;
                platformStateOut.state += 2;
                #else
                platformStateOut.mode = chatOut_.platformState.mode;
                platformStateOut.state = chatOut_.platformState.state;
                #endif

                command = GetPlatform;

                byte_size = sizeof(command) + sizeof(platformStateOut);
                u8_ptr = (char*)&byte_size;
                memcpy(buffer, u8_ptr, sizeof(byte_size));
                u8_ptr = (char*)&command;
                memcpy(buffer+sizeof(byte_size), u8_ptr, sizeof(command));
                u8_ptr = (char*)&platformStateOut;
                memcpy(buffer+sizeof(byte_size)+sizeof(command), u8_ptr, sizeof(platformStateOut));
                buffer[ACK_IDX] = ACK_STR;
                send(client_sock, buffer, sizeof(byte_size)+sizeof(command)+sizeof(platformStateOut), 0);
            }

            if (sensorState.period !=0 && sensorState.period <= (ts_now-ts_sensor)) {
                ts_sensor = ts_now;

                // 전체 송신할 바이트 계산
                #if TCP_DUMMY_SEND
                sensorStateOut.front += 1;
                sensorStateOut.back += 2;
                sensorStateOut.right_front += 3;
                sensorStateOut.right_back += 4;
                #else
                sensorStateOut.front = chatOut_.sensorState.front;
                sensorStateOut.back = chatOut_.sensorState.back;
                sensorStateOut.right_front = chatOut_.sensorState.right_front;
                sensorStateOut.right_back = chatOut_.sensorState.right_back;
                #endif

                command = GetSensor;

                byte_size = sizeof(command) + sizeof(sensorStateOut);
                u8_ptr = (char*)&byte_size;
                memcpy(buffer, u8_ptr, sizeof(byte_size));
                u8_ptr = (char*)&command;
                memcpy(buffer+sizeof(byte_size), u8_ptr, sizeof(command));
                u8_ptr = (char*)&sensorStateOut;
                memcpy(buffer+sizeof(byte_size)+sizeof(command), u8_ptr, sizeof(sensorStateOut));
                buffer[ACK_IDX] = ACK_STR;
                send(client_sock, buffer, sizeof(byte_size)+sizeof(command)+sizeof(sensorStateOut), 0);
            }

            if (posState.period !=0 && posState.period <= (ts_now-ts_position)) {
                ts_position = ts_now;

                // 전체 송신할 바이트 계산
                #if TCP_DUMMY_SEND
                posStateOut.x += 1;
                posStateOut.y += 2;
                posStateOut.rz += 3;
                posStateOut.ang += 4;
                #else
                posStateOut.x = chatOut_.posState.x;
                posStateOut.y = chatOut_.posState.y;
                posStateOut.rz = chatOut_.posState.rz;
                posStateOut.ang = chatOut_.posState.ang;
                #endif

                command = GetPos;

                byte_size = sizeof(command) + sizeof(posStateOut);
                u8_ptr = (char*)&byte_size;
                memcpy(buffer, u8_ptr, sizeof(byte_size));
                u8_ptr = (char*)&command;
                memcpy(buffer+sizeof(byte_size), u8_ptr, sizeof(command));
                u8_ptr = (char*)&posStateOut;
                memcpy(buffer+sizeof(byte_size)+sizeof(command), u8_ptr, sizeof(posStateOut));
                buffer[ACK_IDX] = ACK_STR;
                send(client_sock, buffer, sizeof(byte_size)+sizeof(command)+sizeof(posStateOut), 0);
            }

            if (batteryState.period !=0 && batteryState.period <= (ts_now-ts_battery)) {
                ts_battery = ts_now;

                // 전체 송신할 바이트 계산
                #if TCP_DUMMY_SEND
                batteryStateOut.voltage += 1;
                batteryStateOut.current += 2;
                batteryStateOut.percentage += 3;
                #else
                batteryStateOut.voltage = chatOut_.batteryState.voltage;
                batteryStateOut.current = chatOut_.batteryState.current;
                batteryStateOut.percentage = chatOut_.batteryState.percentage;
                #endif

                command = GetBattery;

                byte_size = sizeof(command) + sizeof(batteryStateOut);
                u8_ptr = (char*)&byte_size;
                memcpy(buffer, u8_ptr, sizeof(byte_size));
                u8_ptr = (char*)&command;
                memcpy(buffer+sizeof(byte_size), u8_ptr, sizeof(command));
                u8_ptr = (char*)&batteryStateOut;
                memcpy(buffer+sizeof(byte_size)+sizeof(command), u8_ptr, sizeof(batteryStateOut));
                buffer[ACK_IDX] = ACK_STR;
                send(client_sock, buffer, sizeof(byte_size)+sizeof(command)+sizeof(batteryStateOut), 0);
            }

            if (whisperState.period !=0 && whisperState.period <= (ts_now-ts_whisper)) {
                ts_whisper = ts_now;

                // 전체 송신할 바이트 계산
                #if TCP_DUMMY_SEND
                whisperOut_.val00.subject = "val00val00val00val00";
                memset(whisperStateOut.val00.subject, '\0', SUBJECT_SIZE);
                memcpy(whisperStateOut.val00.subject, whisperOut_.val00.subject.c_str(), whisperOut_.val00.subject.length()>SUBJECT_SIZE?SUBJECT_SIZE:whisperOut_.val00.subject.length());
                whisperStateOut.val00.value += 1;
                whisperOut_.val19.subject = "val19val19val19val19";
                memset(whisperStateOut.val19.subject, '\0', SUBJECT_SIZE);
                memcpy(whisperStateOut.val19.subject, whisperOut_.val19.subject.c_str(), whisperOut_.val19.subject.length()>SUBJECT_SIZE?SUBJECT_SIZE:whisperOut_.val19.subject.length());
                whisperStateOut.val19.value += 2;
                #else
                memset(whisperStateOut.val00.subject, '\0', SUBJECT_SIZE);
                memcpy(whisperStateOut.val00.subject, whisperOut_.val00.subject.c_str(), whisperOut_.val00.subject.length()>SUBJECT_SIZE?SUBJECT_SIZE:whisperOut_.val00.subject.length());
                whisperStateOut.val00.value = whisperOut_.val00.value;
                memset(whisperStateOut.val01.subject, '\0', SUBJECT_SIZE);
                memcpy(whisperStateOut.val01.subject, whisperOut_.val01.subject.c_str(), whisperOut_.val01.subject.length()>SUBJECT_SIZE?SUBJECT_SIZE:whisperOut_.val01.subject.length());
                whisperStateOut.val01.value = whisperOut_.val01.value;
                memset(whisperStateOut.val02.subject, '\0', SUBJECT_SIZE);
                memcpy(whisperStateOut.val02.subject, whisperOut_.val02.subject.c_str(), whisperOut_.val02.subject.length()>SUBJECT_SIZE?SUBJECT_SIZE:whisperOut_.val02.subject.length());
                whisperStateOut.val02.value = whisperOut_.val02.value;
                memset(whisperStateOut.val03.subject, '\0', SUBJECT_SIZE);
                memcpy(whisperStateOut.val03.subject, whisperOut_.val03.subject.c_str(), whisperOut_.val03.subject.length()>SUBJECT_SIZE?SUBJECT_SIZE:whisperOut_.val03.subject.length());
                whisperStateOut.val03.value = whisperOut_.val03.value;
                memset(whisperStateOut.val04.subject, '\0', SUBJECT_SIZE);
                memcpy(whisperStateOut.val04.subject, whisperOut_.val04.subject.c_str(), whisperOut_.val04.subject.length()>SUBJECT_SIZE?SUBJECT_SIZE:whisperOut_.val04.subject.length());
                whisperStateOut.val04.value = whisperOut_.val04.value;
                memset(whisperStateOut.val05.subject, '\0', SUBJECT_SIZE);
                memcpy(whisperStateOut.val05.subject, whisperOut_.val05.subject.c_str(), whisperOut_.val05.subject.length()>SUBJECT_SIZE?SUBJECT_SIZE:whisperOut_.val05.subject.length());
                whisperStateOut.val05.value = whisperOut_.val05.value;
                memset(whisperStateOut.val06.subject, '\0', SUBJECT_SIZE);
                memcpy(whisperStateOut.val06.subject, whisperOut_.val06.subject.c_str(), whisperOut_.val06.subject.length()>SUBJECT_SIZE?SUBJECT_SIZE:whisperOut_.val06.subject.length());
                whisperStateOut.val06.value = whisperOut_.val06.value;
                memset(whisperStateOut.val07.subject, '\0', SUBJECT_SIZE);
                memcpy(whisperStateOut.val07.subject, whisperOut_.val07.subject.c_str(), whisperOut_.val07.subject.length()>SUBJECT_SIZE?SUBJECT_SIZE:whisperOut_.val07.subject.length());
                whisperStateOut.val07.value = whisperOut_.val07.value;
                memset(whisperStateOut.val08.subject, '\0', SUBJECT_SIZE);
                memcpy(whisperStateOut.val08.subject, whisperOut_.val08.subject.c_str(), whisperOut_.val08.subject.length()>SUBJECT_SIZE?SUBJECT_SIZE:whisperOut_.val08.subject.length());
                whisperStateOut.val08.value = whisperOut_.val08.value;
                memset(whisperStateOut.val09.subject, '\0', SUBJECT_SIZE);
                memcpy(whisperStateOut.val09.subject, whisperOut_.val09.subject.c_str(), whisperOut_.val09.subject.length()>SUBJECT_SIZE?SUBJECT_SIZE:whisperOut_.val09.subject.length());
                whisperStateOut.val09.value = whisperOut_.val09.value;
                memset(whisperStateOut.val10.subject, '\0', SUBJECT_SIZE);
                memcpy(whisperStateOut.val10.subject, whisperOut_.val10.subject.c_str(), whisperOut_.val10.subject.length()>SUBJECT_SIZE?SUBJECT_SIZE:whisperOut_.val10.subject.length());
                whisperStateOut.val10.value = whisperOut_.val10.value;
                memset(whisperStateOut.val11.subject, '\0', SUBJECT_SIZE);
                memcpy(whisperStateOut.val11.subject, whisperOut_.val11.subject.c_str(), whisperOut_.val11.subject.length()>SUBJECT_SIZE?SUBJECT_SIZE:whisperOut_.val11.subject.length());
                whisperStateOut.val11.value = whisperOut_.val11.value;
                memset(whisperStateOut.val12.subject, '\0', SUBJECT_SIZE);
                memcpy(whisperStateOut.val12.subject, whisperOut_.val12.subject.c_str(), whisperOut_.val12.subject.length()>SUBJECT_SIZE?SUBJECT_SIZE:whisperOut_.val12.subject.length());
                whisperStateOut.val12.value = whisperOut_.val12.value;
                memset(whisperStateOut.val13.subject, '\0', SUBJECT_SIZE);
                memcpy(whisperStateOut.val13.subject, whisperOut_.val13.subject.c_str(), whisperOut_.val13.subject.length()>SUBJECT_SIZE?SUBJECT_SIZE:whisperOut_.val13.subject.length());
                whisperStateOut.val13.value = whisperOut_.val13.value;
                memset(whisperStateOut.val14.subject, '\0', SUBJECT_SIZE);
                memcpy(whisperStateOut.val14.subject, whisperOut_.val14.subject.c_str(), whisperOut_.val14.subject.length()>SUBJECT_SIZE?SUBJECT_SIZE:whisperOut_.val14.subject.length());
                whisperStateOut.val14.value = whisperOut_.val14.value;
                memset(whisperStateOut.val15.subject, '\0', SUBJECT_SIZE);
                memcpy(whisperStateOut.val15.subject, whisperOut_.val15.subject.c_str(), whisperOut_.val15.subject.length()>SUBJECT_SIZE?SUBJECT_SIZE:whisperOut_.val15.subject.length());
                whisperStateOut.val15.value = whisperOut_.val15.value;
                memset(whisperStateOut.val16.subject, '\0', SUBJECT_SIZE);
                memcpy(whisperStateOut.val16.subject, whisperOut_.val16.subject.c_str(), whisperOut_.val16.subject.length()>SUBJECT_SIZE?SUBJECT_SIZE:whisperOut_.val16.subject.length());
                whisperStateOut.val16.value = whisperOut_.val16.value;
                memset(whisperStateOut.val17.subject, '\0', SUBJECT_SIZE);
                memcpy(whisperStateOut.val17.subject, whisperOut_.val17.subject.c_str(), whisperOut_.val17.subject.length()>SUBJECT_SIZE?SUBJECT_SIZE:whisperOut_.val17.subject.length());
                whisperStateOut.val17.value = whisperOut_.val17.value;
                memset(whisperStateOut.val18.subject, '\0', SUBJECT_SIZE);
                memcpy(whisperStateOut.val18.subject, whisperOut_.val18.subject.c_str(), whisperOut_.val18.subject.length()>SUBJECT_SIZE?SUBJECT_SIZE:whisperOut_.val18.subject.length());
                whisperStateOut.val18.value = whisperOut_.val18.value;
                memset(whisperStateOut.val19.subject, '\0', SUBJECT_SIZE);
                memcpy(whisperStateOut.val19.subject, whisperOut_.val19.subject.c_str(), whisperOut_.val19.subject.length()>SUBJECT_SIZE?SUBJECT_SIZE:whisperOut_.val19.subject.length());
                whisperStateOut.val19.value = whisperOut_.val19.value;
                memset(whisperStateOut.val20.subject, '\0', SUBJECT_SIZE);
                memcpy(whisperStateOut.val20.subject, whisperOut_.val20.subject.c_str(), whisperOut_.val20.subject.length()>SUBJECT_SIZE?SUBJECT_SIZE:whisperOut_.val20.subject.length());
                whisperStateOut.val20.value = whisperOut_.val20.value;
                memset(whisperStateOut.val21.subject, '\0', SUBJECT_SIZE);
                memcpy(whisperStateOut.val21.subject, whisperOut_.val21.subject.c_str(), whisperOut_.val21.subject.length()>SUBJECT_SIZE?SUBJECT_SIZE:whisperOut_.val21.subject.length());
                whisperStateOut.val21.value = whisperOut_.val21.value;
                memset(whisperStateOut.val22.subject, '\0', SUBJECT_SIZE);
                memcpy(whisperStateOut.val22.subject, whisperOut_.val22.subject.c_str(), whisperOut_.val22.subject.length()>SUBJECT_SIZE?SUBJECT_SIZE:whisperOut_.val22.subject.length());
                whisperStateOut.val22.value = whisperOut_.val22.value;
                memset(whisperStateOut.val23.subject, '\0', SUBJECT_SIZE);
                memcpy(whisperStateOut.val23.subject, whisperOut_.val23.subject.c_str(), whisperOut_.val23.subject.length()>SUBJECT_SIZE?SUBJECT_SIZE:whisperOut_.val23.subject.length());
                whisperStateOut.val23.value = whisperOut_.val23.value;
                memset(whisperStateOut.val24.subject, '\0', SUBJECT_SIZE);
                memcpy(whisperStateOut.val24.subject, whisperOut_.val24.subject.c_str(), whisperOut_.val24.subject.length()>SUBJECT_SIZE?SUBJECT_SIZE:whisperOut_.val24.subject.length());
                whisperStateOut.val24.value = whisperOut_.val24.value;
                memset(whisperStateOut.val25.subject, '\0', SUBJECT_SIZE);
                memcpy(whisperStateOut.val25.subject, whisperOut_.val25.subject.c_str(), whisperOut_.val25.subject.length()>SUBJECT_SIZE?SUBJECT_SIZE:whisperOut_.val25.subject.length());
                whisperStateOut.val25.value = whisperOut_.val25.value;
                memset(whisperStateOut.val26.subject, '\0', SUBJECT_SIZE);
                memcpy(whisperStateOut.val26.subject, whisperOut_.val26.subject.c_str(), whisperOut_.val26.subject.length()>SUBJECT_SIZE?SUBJECT_SIZE:whisperOut_.val26.subject.length());
                whisperStateOut.val26.value = whisperOut_.val26.value;
                memset(whisperStateOut.val27.subject, '\0', SUBJECT_SIZE);
                memcpy(whisperStateOut.val27.subject, whisperOut_.val27.subject.c_str(), whisperOut_.val27.subject.length()>SUBJECT_SIZE?SUBJECT_SIZE:whisperOut_.val27.subject.length());
                whisperStateOut.val27.value = whisperOut_.val27.value;
                memset(whisperStateOut.val28.subject, '\0', SUBJECT_SIZE);
                memcpy(whisperStateOut.val28.subject, whisperOut_.val28.subject.c_str(), whisperOut_.val28.subject.length()>SUBJECT_SIZE?SUBJECT_SIZE:whisperOut_.val28.subject.length());
                whisperStateOut.val28.value = whisperOut_.val28.value;
                memset(whisperStateOut.val29.subject, '\0', SUBJECT_SIZE);
                memcpy(whisperStateOut.val29.subject, whisperOut_.val29.subject.c_str(), whisperOut_.val29.subject.length()>SUBJECT_SIZE?SUBJECT_SIZE:whisperOut_.val29.subject.length());
                whisperStateOut.val29.value = whisperOut_.val29.value;
                #endif

                command = GetWhisper;

                byte_size = sizeof(command) + sizeof(whisperStateOut);
                u8_ptr = (char*)&byte_size;
                memcpy(buffer, u8_ptr, sizeof(byte_size));
                u8_ptr = (char*)&command;
                memcpy(buffer+sizeof(byte_size), u8_ptr, sizeof(command));
                u8_ptr = (char*)&whisperStateOut;
                memcpy(buffer+sizeof(byte_size)+sizeof(command), u8_ptr, sizeof(whisperStateOut));
                buffer[ACK_IDX] = ACK_STR;
                send(client_sock, buffer, sizeof(byte_size)+sizeof(command)+sizeof(whisperStateOut), 0);
            }
        }
        printf("[s] tcp read/write end\n");

        ret = close(client_sock);
        printf("[s] client socket closed, ret: %d\n", ret);

        ret = close(sock);
        printf("[s] socket closed, ret: %d\n", ret);
        // 소켓을 정상적으로 너무 빨리 닫고 재 열기할 경우
        // 해당 포트가 이미 사용중이라는 표시가 나타날 수 있음
        usleep(500000);
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "chatterbox");
    ros::NodeHandle nh("~");

    ros::Subscriber chatOut_sub = nh.subscribe("/chatterbox/chatOut_topic", 100, chatOutCallBack);
    ros::Subscriber whisperOut_sub = nh.subscribe("/chatterbox/whisperOut_topic", 100, whisperOutCallBack);

    ros::Publisher chatIn_pub = nh.advertise<chatterbox::ChatIn>("chatIn_topic", 100);

    int thread_rate = 200;
    boost::thread hThread(fThread, &thread_rate, &chatIn_pub);

    ros::Rate main_rate(1000);

    double time_cur = ros::Time::now().toSec();
    double time_pre = time_cur;
    double time_diff;

    while(ros::ok())
    {
        time_cur = ros::Time::now().toSec();
        time_diff = time_cur - time_pre;
#define PERIOD  0.1
        if ( time_diff > PERIOD) {
            time_pre = time_cur;
        }

        ros::spinOnce();

        main_rate.sleep();
    }

    printf("[s] hThread join\n");
    hThread.join();
    printf("[s] hThread joined\n");
}
