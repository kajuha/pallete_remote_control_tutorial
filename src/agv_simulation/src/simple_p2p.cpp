#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <visualization_msgs/Marker.h>
#include <tf/tf.h>

#define pi 3.141592

float deg2rad(float deg)
{
    return deg * pi / 180.0;
}

float rad2deg(float rad)
{
    return rad * 180.0 / pi;
}

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "simple_navigation_goals");
    ros::NodeHandle n;
    ros::Rate r(1);
    ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("WAYPOINTS", 1);
    
    MoveBaseClient ac("move_base", true);
    // Forward
    const int num_ckp_forward = 6;
    float x_coord_fw[num_ckp_forward*2]     = { 15.0,  15.0,  15.0, 15.0,   8.0,  8.0,  15.0,  15.0,  15.0,  15.0,   0.0,   0.0};
    float y_coord_fw[num_ckp_forward*2]     = {  0.0,   0.0,  -8.0, -8.0,  -8.0, -8.0,  -8.0,  -8.0, -15.0, -15.0, -15.0, -15.0};
    float orientation_fw[num_ckp_forward*2] = {  0.0, -pi/2, -pi/2,  -pi,   -pi,  0.0,   0.0, -pi/2, -pi/2,   -pi,   -pi,   0.0};

    // Return to origin
    const int num_ckp_rtn = 3;

    float x_coord_rtn[num_ckp_rtn*2]     = { 15.0,  15.0, 15.0, 15.0, 0.0, 0.0};
    float y_coord_rtn[num_ckp_rtn*2]     = {-15.0, -15.0,  0.0,  0.0, 0.0, 0.0};
    float orientation_rtn[num_ckp_rtn*2] = {  0.0,  pi/2, pi/2,   pi,  pi, 0.0};

    while(!ac.waitForServer(ros::Duration(5.0)))
    {
        ROS_INFO("Waiting for the move_base action server to come up");
    }

    move_base_msgs::MoveBaseGoal goal;
    tf::Quaternion q;

    for(int ckp_i = 0; ckp_i < num_ckp_forward*2; ckp_i++)
    {
        visualization_msgs::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = ros::Time::now();
        marker.ns = "waypoint";
        marker.id = ckp_i;
        marker.type = visualization_msgs::Marker::SPHERE;
        marker.action = visualization_msgs::Marker::ADD;

        goal.target_pose.header.frame_id = "map";
        goal.target_pose.header.stamp = ros::Time::now();

        goal.target_pose.pose.position.x = x_coord_fw[ckp_i];
        goal.target_pose.pose.position.y = y_coord_fw[ckp_i];
        goal.target_pose.pose.position.z = 0.0;

        q.setRPY(0.0, 0.0, orientation_fw[ckp_i]);
        q = q.normalize();

        goal.target_pose.pose.orientation.x = q.x();
        goal.target_pose.pose.orientation.y = q.y();
        goal.target_pose.pose.orientation.z = q.z();
        goal.target_pose.pose.orientation.w = q.w();

        marker.lifetime = ros::Duration();
        marker.pose.position.x = x_coord_fw[ckp_i];
        marker.pose.position.y = y_coord_fw[ckp_i];
        marker.pose.position.z = 0.0;
        marker.pose.orientation.x = q.x();
        marker.pose.orientation.y = q.y();
        marker.pose.orientation.z = q.z();
        marker.pose.orientation.w = q.w();

        marker.scale.x = 0.4;
        marker.scale.y = 0.4;
        marker.scale.z = 0.4;
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;
        marker.color.a = 0.8;
        marker_pub.publish(marker);

        ROS_INFO("Sending goal: x> %f, y> %f, th> %f", x_coord_fw[ckp_i], y_coord_fw[ckp_i], orientation_fw[ckp_i]);

        ac.sendGoal(goal);

        ac.waitForResult();

        if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
            ROS_INFO("Temporory Goal arrived (Forward)");
        else
        {
            ROS_INFO("Error occured");
            break;
        }
    }
    
    for(int ckp_i = 0; ckp_i < num_ckp_rtn*2; ckp_i++)
    {
        goal.target_pose.header.frame_id = "map";
        goal.target_pose.header.stamp = ros::Time::now();

        goal.target_pose.pose.position.x = x_coord_rtn[ckp_i];
        goal.target_pose.pose.position.y = y_coord_rtn[ckp_i];
        goal.target_pose.pose.position.z = 0.0;

        q.setRPY(0.0, 0.0, orientation_rtn[ckp_i]);
        q = q.normalize();

        goal.target_pose.pose.orientation.x = q.x();
        goal.target_pose.pose.orientation.y = q.y();
        goal.target_pose.pose.orientation.z = q.z();
        goal.target_pose.pose.orientation.w = q.w();

        visualization_msgs::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = ros::Time::now();
        marker.ns = "waypoint";
        marker.id = ckp_i;
        marker.type = visualization_msgs::Marker::SPHERE;
        marker.action = visualization_msgs::Marker::ADD;

        marker.lifetime = ros::Duration();
        marker.pose.position.x = x_coord_rtn[ckp_i];
        marker.pose.position.y = y_coord_rtn[ckp_i];
        marker.pose.position.z = 0.0;
        marker.pose.orientation.x = q.x();
        marker.pose.orientation.y = q.y();
        marker.pose.orientation.z = q.z();
        marker.pose.orientation.w = q.w();

        marker.scale.x = 0.4;
        marker.scale.y = 0.4;
        marker.scale.z = 0.4;
        marker.color.r = 0.7;
        marker.color.g = 0.4;
        marker.color.b = 0.7;
        marker.color.a = 0.8;
        marker_pub.publish(marker);

        ROS_INFO("Sending goal: x> %f, y> %f, th> %f", x_coord_rtn[ckp_i], y_coord_rtn[ckp_i], orientation_rtn[ckp_i]);

        ac.sendGoal(goal);

        ac.waitForResult();

        if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
            ROS_INFO("Temporory Goal arrived (Retrun to origin)");
        else
        {
            ROS_INFO("Error occured");
            break;
        }
    }

    return 0;
}