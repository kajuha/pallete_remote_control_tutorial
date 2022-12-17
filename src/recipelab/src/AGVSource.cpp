/*======================================================================
	File name	:	AGVSource.c                    
                    
	Originator	:	AGV Control

	Target		:	ROS 

	Version		:	1.00

	Eigen 참고 사이트 : https://sunggoo.tistory.com/51?category=863796
	Matrix3f A ;	// float
	Matrix4d B ;	// double
	// 20 x75 matrix of type float
	Matrix <float , 20 , 75 > M2 ;
	// Set each coefficient to a uniform random value in the range
	[ -1 , 1]
	A = Matrix3f :: Random () ;
	// Set B to the identity matrix
	B = Matrix4d :: Identity () ;
	// Set all elements to zero
	A = Matrix3f :: Zero () ;
	// Set all elements to ones
	A = Matrix3f :: Ones () ;
	// Set all elements to a constant value
	B = Matrix4d :: Constant (4.5) ;
	// Transposition
	cout << M1 . transpose () << endl ;
	// Inversion ( # include <Eigen /Dense > )
	// Generates NaNs if the matrix is not invertible
	cout << M1 . inverse () << endl ;
	cout << A (1 , 2) ;
======================================================================*/

/*======================================================================
	History		:
		2021-04-20,		Version 1.00
======================================================================*/

#include "AGVConfig.h"

__uint16_t CalcCRCbyAlgorithm(__uint16_t* pDataBuffer, __uint16_t usDataLen)
{	
	const __uint16_t POLYNOMIAL = 0xA001;
	__uint16_t wCrc;
	__int16_t iByte, iBit;
	wCrc = 0xffff;
	for (iByte = 0; iByte < usDataLen; iByte++)
	{
		wCrc ^= *(pDataBuffer + iByte);		
		for (iBit = 0; iBit <= 7; iBit++)
		{
			if (wCrc & 0x0001)
			{
				wCrc >>= 1;
				wCrc ^= POLYNOMIAL;
			}
			else
			{
				wCrc >>= 1;
			}
		}
	}
	if(((wCrc>>8)&0xff)==0xFA)	  wCrc=0x01AF;
	if(((wCrc>>8)&0xff)==0xFE)	  wCrc=0x01EF;	
	
	return wCrc;
}
void MemCopy(__uint16_t *SourceAddr, __uint16_t* SourceEndAddr, __uint16_t* DestAddr)
{
    while(SourceAddr < SourceEndAddr)
    {
        *DestAddr++ = *SourceAddr++;
    }
    return;
}
double PosProfile(double fInit_Pos,double fFinal_Pos,double fv_in,double fa_in,PROFILE_JOINT *fpm)
{
	double t;			// 시간의 흐름
	double w;
	double w_dot;
	double fs;
	t = fpm->tcounter;
	switch(fpm->StartFlag)
	{
		case PROFILEInit:
			if((fv_in<0.0001) || (fa_in<0.0001))	// 20210607 설정 가속도 또는 속도값이 0.0이면  입력값이 에러이면 동작안해야함.
			{
				fpm->Init_Pos 		= fInit_Pos;
				fpm->Final_Pos 		= fpm->Init_Pos;
				//추후에 고장진단에서 아래값을 가지고 판단함.
				fpm->Comd_Vel		= fv_in;     // AGV 속도 설정
				fpm->Design_Accel	= fa_in;     // AGV 가속도 설정
				fpm->Ts = 0.0;
				fpm->T	= 0.0;
				fpm->tcounter = 0.0;
				t = fpm->tcounter;		// 이 변수를 다시 한번 써줘야 한다.
				fpm->Output_Pos	= fpm->Final_Pos;   // 위치 이동이 너무 작을 경우 skip final_pos를 바로 출력하고
				fpm->StartFlag = PROFILEStop;
				printf("AGVSource.cpp line %d  --> if((fv_in<0.0001) || (fa_in<0.0001)) 입력값에러 \n", __LINE__);
				break;
			}
			else
			{
				fpm->Init_Pos 		= fInit_Pos;
				fpm->Final_Pos 		= fFinal_Pos;
				fpm->Comd_Vel		= fv_in;     // AGV 속도 설정
				fpm->Design_Accel	= fa_in;     // AGV 가속도 설정
				printf("AGVSource.cpp line %d  --> 정상적 입력값 (%f) \n", __LINE__ ,(fpm->Final_Pos - fpm->Init_Pos) );
				fpm->Length_Pos = sqrt((fpm->Final_Pos - fpm->Init_Pos)*(fpm->Final_Pos - fpm->Init_Pos));
				if(fpm->Design_Accel > fpm->Max_Accel)	fpm->Design_Accel = fpm->Max_Accel;
				// CASE가 두가지로 나뉘어진다.
				// 1. 위치이동이 너무 작을 경우
				if(fpm->Length_Pos <= MIN_POS)		//위치 이동이 너무 작을경우(0.001mm , 0.06deg 이하) 속도프로파일 적용하지 않고 final_pos를 출력한다.
				{
					fpm->Ts = 0.0;
					fpm->T	= 0.0;
					fpm->tcounter = 0.0;
					t = fpm->tcounter;		// 이 변수를 다시 한번 써줘야 한다.
					fpm->Output_Pos	= fpm->Final_Pos;   // 위치 이동이 너무 작을 경우 skip final_pos를 바로 출력하고
					fpm->StartFlag = PROFILEStop;
					printf("AGVSource.cpp line %d --> 1. 위치이동이 너무 작음 \n", __LINE__);
					break;
				}
				// 2. 정상 위치 이동일경우
				else
				{		
					// CASE가 다시 나뉘어진다. 설정 가속도가 너무 작을 경우
					if(fpm->Length_Pos <= (fpm->Comd_Vel*fpm->Comd_Vel/fpm->Design_Accel))	//설정 가속도가 너무 작을경우 사다리꼴 프로파일이 아닌 삼각형 프로파일을 그린다.
					{
						double T_Ts_ratio = 100.0;		//전체 시간 T 에서 Ts가 차지하는 % 이다. 100% 가 되면 Ts = T/2가 된다.
						fpm->Design_Accel = fpm->Comd_Vel * fpm->Comd_Vel * 100.0/(T_Ts_ratio*fpm->Length_Pos);
					}
					fpm->Ts = fpm->Comd_Vel/fpm->Design_Accel;
					fpm->T	= (fpm->Comd_Vel*fpm->Comd_Vel + fpm->Design_Accel*fpm->Length_Pos)/(fpm->Design_Accel*fpm->Comd_Vel);
					fpm->tcounter = 0.0;
					t = fpm->tcounter;		// 이 변수를 다시 한번 써줘야 한다.
					fpm->StartFlag = PROFILERun;
					printf("AGVSource.cpp line %d --> 2. 정상 위치 이동일경우) \n", __LINE__);
				}
			//LBox->state.AgvRunState   = RUNNING ;
			}
				
//			break;
		case PROFILERun:
			if(t <= fpm->Ts)
			{
				w = fpm->Design_Accel*t*t/2;
		        w_dot = fpm->Design_Accel*t;
		//        w_dot_dot = fpm->Design_Accel;
			}
			else if(((fpm->Ts) < t) && (t <= (fpm->T)-(fpm->Ts)))
			{
				w = fpm->Comd_Vel*t - (fpm->Comd_Vel*fpm->Comd_Vel)/(2*fpm->Design_Accel);
		        w_dot = fpm->Comd_Vel;
			}
			else if((fpm->T-fpm->Ts < t) && (t <= fpm->T))
			{
				w = -fpm->Design_Accel*(t-fpm->T)*(t-fpm->T)/2 + fpm->Comd_Vel*fpm->T - (fpm->Comd_Vel*fpm->Comd_Vel)/fpm->Design_Accel;
		        w_dot = -fpm->Design_Accel*(t-fpm->T);
			}
			else
			{
				t=fpm->T;
				w = -fpm->Design_Accel*(t-fpm->T)*(t-fpm->T)/2 + fpm->Comd_Vel*fpm->T - (fpm->Comd_Vel*fpm->Comd_Vel)/fpm->Design_Accel;
		        w_dot = -fpm->Design_Accel*(t-fpm->T);
		        fpm->StartFlag = PROFILEStop;				
			} 
			fs = w/fpm->Length_Pos;
			fpm->Output_Pos = fpm->Init_Pos + fs*(fpm->Final_Pos - fpm->Init_Pos);	
			fpm->tcounter += Ts_main;   // Ts_main 는 샘플링 시간임. 0.01초 이면 0.01마다 제어주기가 옴.
			//LBox->state.AgvRunState   = RUNNING ;
			break;
		case PROFILEStop:
		default:
			// 여기서 문제가 발생할 수 있다. 밑에 부분을 주석처리한다. 
			//fpm->Init_Pos 	= 0.0;
			//fpm->Final_Pos 	= 0.0;
			//fpm->Comd_Vel	= 0.0;
			fpm->T			= 0.0;
			fpm->tcounter 	= 0.0;
			fpm->Output_Pos	= fpm->Final_Pos;	
			//LBox->state.AgvRunState   = READY ;					
			break;
	}
	return fpm->Output_Pos;
}
void PosProfileCartecian(double* fInit_Pos,double* fFinal_Pos,double fv_in,double fa_in,PROFILE_CART *fpm)
{
	double t;			// 시간의 흐름
	double w;
	double w_dot;
	double fs;
	double fs_dot;
	double LengthXY[2];
	t = fpm->tcounter;
	switch(fpm->StartFlag)
	{
		case PROFILEInit:
			fpm->Init_Pos[0] 	= *(fInit_Pos+0);
			fpm->Init_Pos[1] 	= *(fInit_Pos+1);
			fpm->Final_Pos[0]	= *(fFinal_Pos+0);
			fpm->Final_Pos[1]	= *(fFinal_Pos+1);

			fpm->Comd_Vel		= fv_in;
			fpm->Design_Accel	= fa_in;
			LengthXY[0] 		= (fpm->Final_Pos[0] - fpm->Init_Pos[0])*(fpm->Final_Pos[0] - fpm->Init_Pos[0]);
			LengthXY[1] 		= (fpm->Final_Pos[1] - fpm->Init_Pos[1])*(fpm->Final_Pos[1] - fpm->Init_Pos[1]);
			 
			fpm->Length_Pos = sqrt(LengthXY[0] + LengthXY[1]);
			if(fpm->Design_Accel > fpm->Max_Accel)	fpm->Design_Accel = fpm->Max_Accel;
			if(fpm->Length_Pos <= (fpm->Comd_Vel*fpm->Comd_Vel/fpm->Design_Accel))	fpm->Comd_Vel = sqrt(fpm->Design_Accel*fpm->Length_Pos);
			fpm->Ts = fpm->Comd_Vel/fpm->Design_Accel;
			fpm->T	= (fpm->Comd_Vel*fpm->Comd_Vel + fpm->Design_Accel*fpm->Length_Pos)/(fpm->Design_Accel*fpm->Comd_Vel);
			fpm->tcounter = 0.0;
			t = fpm->tcounter;		// 이 변수를 다시 한번 써줘야 한다.
			fpm->StartFlag = PROFILERun;
//			break;
		case PROFILERun:
			if(t <= fpm->Ts)
			{
				w = fpm->Design_Accel*t*t/2;
		        w_dot = fpm->Design_Accel*t;
			}
			else if((fpm->Ts < t) && (t <= fpm->T-fpm->Ts))
			{
				w = fpm->Comd_Vel*t - (fpm->Comd_Vel*fpm->Comd_Vel)/(2*fpm->Design_Accel);
		        w_dot = fpm->Comd_Vel;
			}
			else if((fpm->T-fpm->Ts < t) && (t <= fpm->T))
			{
				w = -fpm->Design_Accel*(t-fpm->T)*(t-fpm->T)/2 + fpm->Comd_Vel*fpm->T - (fpm->Comd_Vel*fpm->Comd_Vel)/fpm->Design_Accel;
		        w_dot = -fpm->Design_Accel*(t-fpm->T);
			}
			else
			{
				t=fpm->T;
				w = -fpm->Design_Accel*(t-fpm->T)*(t-fpm->T)/2 + fpm->Comd_Vel*fpm->T - (fpm->Comd_Vel*fpm->Comd_Vel)/fpm->Design_Accel;
		        w_dot = -fpm->Design_Accel*(t-fpm->T);
		        fpm->StartFlag = PROFILEStop;
				
			} 
			fs 		= w/fpm->Length_Pos;
			fs_dot 	= w_dot/fpm->Length_Pos;
			fpm->Output_Pos[0] = fpm->Init_Pos[0] + fs*(fpm->Final_Pos[0] - fpm->Init_Pos[0]);
			fpm->Output_Pos[1] = fpm->Init_Pos[1] + fs*(fpm->Final_Pos[1] - fpm->Init_Pos[1]);
			fpm->tcounter += Ts_main;
			break;
		case PROFILEStop:
		default:			// 여기서 문제가 발생할 수 있다.
//			fpm->Init_Pos[0] 	= 0.0;
//			fpm->Final_Pos 		= 0.0;
			fpm->Comd_Vel		= 0.0;
			fpm->T				= 0.0;
			fpm->tcounter 		= 0.0;
			fpm->Output_Pos[0]	= *(fFinal_Pos+0);
			fpm->Output_Pos[1]	= *(fFinal_Pos+1);
			fpm->Output_Pos[2]	= *(fFinal_Pos+2);
			break;
	}
}
double VelProfile(double fInit_Vel,double fFinal_Vel,double fa_in,PROFILE_JOINT *fpm)
{
	// 속도 프로파일의 경우 속도 프로파일 중간에 다시 명령이 들어가도 재계산되어 동작이 가능하도록 설정되어 있음.
	double t;			// 시간의 흐름
	double w;
	double w_dot;
	double fs;
	static __uint32_t bufcount = 0;
	t = fpm->tcounter;
	switch(fpm->StartFlag)
	{
		case PROFILEInit:
			bufcount = 0;
			if(fa_in<0.0001)		// 20210607 AGV 설정 가속도 값이 0.0000001 일경우 더블임으로 0.0으로 정확하게 나오지 않음. 에러처리
			{
				fpm->Init_Vel 		= fInit_Vel;	// AGV 현재 속도
				fpm->Final_Vel 		= fpm->Init_Vel;	// AGV 목표 속도
				//추후에 고장진단에서 아래값을 가지고 판단함.
				fpm->Design_Accel	= fa_in;     	// AGV 가속도 설정
				//printf("fpm->Init_Vel: %f , fpm->Final_Vel : %f\n", fpm->Init_Vel , fpm->Final_Vel);
				// printf("%u -- Init_Vel: %f , Final_Vel: %f , Design_Accel: %f \n",bufcount, fpm->Init_Vel , fpm->Final_Vel,fpm->Design_Accel);
				fpm->Ts = 0.0;
				fpm->T	= 0.0;
				fpm->tcounter = 0.0;
				t = fpm->tcounter;		// 이 변수를 다시 한번 써줘야 한다.
				if(fpm->Final_Vel<0.0000001){	fpm->Output_Vel	= 0.0; fpm->StartFlag = PROFILEStop;}
				else						{	fpm->Output_Vel	= fpm->Final_Vel; fpm->StartFlag = PROFILERunning;}
				break;
			}
			else
			{
				fpm->Init_Vel 		= fInit_Vel;	// AGV 현재 속도
				fpm->Final_Vel 		= fFinal_Vel;	// AGV 목표 속도
				fpm->Design_Accel	= fa_in;     	// AGV 가속도 설정
				//printf("fpm->Init_Vel: %f , fpm->Final_Vel : %f\n", fpm->Init_Vel , fpm->Final_Vel);
				// printf("%u -- Init_Vel: %f , Final_Vel: %f , Design_Accel: %f \n",bufcount, fpm->Init_Vel , fpm->Final_Vel,fpm->Design_Accel);
				fpm->Length_Vel = (fpm->Final_Vel - fpm->Init_Vel);
				if(abs(fpm->Length_Vel) <= MIN_VEL)		//속도 이동이 너무 작을경우(0.0mm , 0.0deg 이하) 속도프로파일 적용하지 않고 final_pos를 출력한다.
				{
					fpm->Ts = 0.0;
					fpm->T	= 0.0;
					fpm->tcounter = 0.0;
					t = fpm->tcounter;		// 이 변수를 다시 한번 써줘야 한다.
					fpm->Output_Vel	= fpm->Final_Vel;   // 속도 이동이 너무 작을 경우 skip final_vel를 바로 출력하고
					fpm->StartFlag = PROFILEStop;
					break;
				}
				else
				{
					if(fpm->Design_Accel > fpm->Max_Accel)	fpm->Design_Accel = fpm->Max_Accel;
					fpm->T = abs(fpm->Length_Vel)/fpm->Design_Accel;
					if(fpm->Length_Vel >= 0.0) 	fpm->Design_Accel = fpm->Design_Accel;
					else						fpm->Design_Accel = -fpm->Design_Accel;
					fpm->tcounter = 0.0;
					t = fpm->tcounter;		// 이 변수를 다시 한번 써줘야 한다.
					//printf(" %f ,  %f , %f , %f\n", fpm->Init_Vel , fpm->Final_Vel,fpm->Design_Accel,t);
					fpm->StartFlag = PROFILERun;
				}
				// printf("%u -- Length_Vel: %f , Design_Accel: %f \n",bufcount, fpm->Length_Vel ,fpm->Design_Accel);
				// printf("%u -- fpm->StartFlag : %X, t : %f , fpm->T : %f\n", bufcount, fpm->StartFlag, t, fpm->T);
			}
			
//			break;
		case PROFILERun:
			if(t <= fpm->T)
			{
		        w_dot = fpm->Init_Vel + fpm->Design_Accel*t;
				//printf("1 %f ,  %f , %f , %f , %f , %f , %f , %f\n", fpm->Init_Vel , fpm->Final_Vel,fpm->Design_Accel,t,w_dot , Ts_main ,t, fpm->T);
			}
			else
			{
				t=fpm->T;
		        w_dot = fpm->Init_Vel + fpm->Design_Accel*t;
		        fpm->StartFlag = PROFILERunning;
				//printf("2 %f ,  %f , %f , %f , %f , %f , %f , %f\n", fpm->Init_Vel , fpm->Final_Vel,fpm->Design_Accel,t,w_dot , Ts_main ,t, fpm->T);			
			} 
			fpm->Output_Vel = w_dot;	
			fpm->tcounter += Ts_main;   // Ts_main 는 샘플링 시간임. 0.01초 이면 0.01마다 제어주기가 옴.
			//printf(" %f ,  %f , %f , %f\n", fpm->Init_Vel , fpm->Final_Vel,fpm->Design_Accel,t);
			break;
		case PROFILERunning:
			//printf("3 %f ,  %f , %f , %f , %f , %f , %f , %f\n", fpm->Init_Vel , fpm->Final_Vel,fpm->Design_Accel,t,w_dot , Ts_main ,t, fpm->T);
			if(abs(fpm->Output_Vel) <= 0.0000001)	fpm->StartFlag = PROFILEStop;
			break;
		case PROFILEStop:
		default:
			fpm->Init_Vel 	= 0.0;
			fpm->Final_Vel 	= 0.0;
			fpm->T			= 0.0;
			fpm->tcounter 	= 0.0;
			fpm->Output_Vel	= 0.0;
			break;
	}
	// if(bufcount <10)
	// {
	// 	bufcount++;
	// 	 printf("%u -- Init_Vel: %f , Final_Vel: %f , Design_Accel: %f \n",bufcount, fpm->Init_Vel , fpm->Final_Vel,fpm->Design_Accel);
	// 	 printf("%u -- Length_Vel: %f , Design_Accel: %f Output_Vel : %f\n",bufcount, fpm->Length_Vel ,fpm->Design_Accel,fpm->Output_Vel);
	// 	 printf("%u -- fpm->StartFlag : %X, t : %f , fpm->T : %f\n", bufcount, fpm->StartFlag, t, fpm->T);
	// }
	return fpm->Output_Vel;

}
void AGVLocationRecognition(double fdl1, double fdl2, double fdl3, double fdl4, AGV1AxisSensorBaseOdometry *fAGVodom)
{
    //agvodom.dl1 = range_rf_.range;
    //agvodom.dl2 = range_rb_.range;
    //agvodom.dl3 = range_br_.range;
    //agvodom.dl4 = range_fr_.range;
    fAGVodom->dl1 = fdl1;           // TF40 Lidar Right Front
    fAGVodom->dl2 = fdl2;           // TF40 Lidar Right Back
    fAGVodom->dl3 = fdl3;           // TF02 Lidar Back Right
    fAGVodom->dl4 = fdl4;           // TF02 Lidar Front Right
    fAGVodom->AGVLocationRecognition_ForwardKinematics();
//    fOdom1AxisLidarBase->odom.feed.pos.x = fAGVodom->x1;                    // 기구학 계산을 거쳐서 나온 AGV Odometry Px
//    fOdom1AxisLidarBase->odom.feed.pos.y = fAGVodom->y1;                    // 기구학 계산을 거쳐서 나온 AGV Odometry Py
//    fOdom1AxisLidarBase->odom.feed.pos.rz = fAGVodom->theta;              // 기구학 계산을 거쳐서 나온 AGV Odometry Yaw
}




