#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/BatteryState.h>

#include <unistd.h>
#include <pthread.h>
#include <semaphore.h>
#include <fcntl.h>
#include <termios.h>
#include <sys/time.h>
#include <stdlib.h>

#include <iostream>
#include <queue>

#include "qucc.h"

#include <string.h>
#define __FILENAME__ (strrchr(__FILE__, '/') ? strrchr(__FILE__, '/') + 1 : __FILE__)

using namespace std;

Qucc qucc;

int main(int argc, char* argv[])
{
	ros::init(argc, argv, "battery");
	ros::NodeHandle nh("~");

	std::string serial_port;
	int baud_rate;
	int slave_num;
	std::string bms_model;

	printf("CMAKE BUILD BMS: %d\n", BMS);

	// ros::param::get("~serial_port", serial_port);
	// ros::param::get("~baud_rate", baud_rate);
	// ros::param::get("~bms_model", bms_model);
	nh.getParam("serial_port", serial_port);
	nh.getParam("baud_rate", baud_rate);
	nh.getParam("bms_model", bms_model);

	printf("FILE NAME : %s\n", __FILENAME__);
	printf("BMS MODEL : %s\n", bms_model.c_str());

	ros::Publisher battery_pub = nh.advertise<sensor_msgs::BatteryState>("battery", 1000, true);

	sensor_msgs::BatteryState batteryState;

    char real_name[NAME_MAX] = {'\0', };

	realpath(serial_port.c_str(), real_name);

	printf("%s->%s %d\n", serial_port.c_str(), real_name, baud_rate);

	qucc = Qucc(real_name, baud_rate);

	if (qucc.initSerial() == false) {
		return 0;
	}

	ros::Rate r(1000);

	#define STEP_TIME 1.0
	double time_cur = ros::Time::now().toSec();
	double time_pre = time_cur;
	double time_diff;

	while (ros::ok())
	{
		qucc.receiveQuccState();

		if (qucc.isParsed) {
			qucc.isParsed = false;

			batteryState.header.stamp = ros::Time::now();
			batteryState.voltage = qucc._quccInfo.voltage_v;
			batteryState.current = qucc._quccInfo.current_a;
			batteryState.capacity = qucc._quccInfo.remaining_capacity_ah;
			batteryState.design_capacity = qucc._quccInfo.norminal_capacity_ah;
			batteryState.percentage = qucc._quccInfo.remaining_capacity_percent / 100.0;
			batteryState.power_supply_technology = batteryState.POWER_SUPPLY_TECHNOLOGY_LIFE;
			battery_pub.publish(batteryState);

			#if 1
			printf("%lf: %6.2f V, %6.2f A, %6.2f %%\n", time_cur, qucc._quccInfo.voltage_v, qucc._quccInfo.current_a, qucc._quccInfo.remaining_capacity_percent);
			#endif
		}

		time_cur = ros::Time::now().toSec();
		time_diff = time_cur - time_pre;
		if ( time_diff > STEP_TIME ) {
			qucc.sendQuccCmd();

			time_pre = time_cur;
		}

		ros::spinOnce();

		r.sleep();
	}

	qucc.closeSerial();

	return 0;
}