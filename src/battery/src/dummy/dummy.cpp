#include <ros/ros.h>
#include <sensor_msgs/BatteryState.h>

#include <random>

#define __FILENAME__ (strrchr(__FILE__, '/') ? strrchr(__FILE__, '/') + 1 : __FILE__)

using namespace std;

int main(int argc, char* argv[])
{
	ros::init(argc, argv, "battery");
	ros::NodeHandle nh("~");
	
	std::string bms_model;

	// ros::param::get("~bms_model", bms_model);
	nh.getParam("bms_model", bms_model);
	
	printf("FILE NAME : %s\n", __FILENAME__);
	printf("BMS MODEL : %s\n", bms_model.c_str());

	ros::Publisher battery_pub = nh.advertise<sensor_msgs::BatteryState>("battery", 1000, true);

	sensor_msgs::BatteryState batteryState;

	ros::Rate r(1000);

	#define STEP_TIME 1.0
	double time_cur = ros::Time::now().toSec();
	double time_pre = time_cur;
	double time_diff;

	std::random_device rd;
	std::mt19937 gen(rd());
	std::uniform_int_distribution<int> dis(0, 100);

	while (ros::ok())
	{
		batteryState.header.stamp = ros::Time::now();
#define DEFAULT_VOLTAGE 24.0
#define DEFAULT_CURRENT 2.0
#define DEFAULT_PERCENTAGE 80.0
		batteryState.voltage = DEFAULT_VOLTAGE + dis(gen)/100.0;
		batteryState.current = DEFAULT_CURRENT + dis(gen)/100.0;
		batteryState.capacity = 0.0;
		batteryState.design_capacity = 0.0;
		batteryState.percentage = (DEFAULT_PERCENTAGE + dis(gen)/10.0)/100.0;
		batteryState.power_supply_technology = batteryState.POWER_SUPPLY_TECHNOLOGY_LIFE;
		battery_pub.publish(batteryState);

		time_cur = ros::Time::now().toSec();
		time_diff = time_cur - time_pre;
		if ( time_diff > STEP_TIME ) {
			time_pre = time_cur;
		}

		ros::spinOnce();

		r.sleep();
	}

	return 0;
}