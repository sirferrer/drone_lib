#include "headers/servo.h"

int main(int argc, char **argv)
{
    // Initialise node
    ros::init(argc, argv, "servo_node");
    ros::NodeHandle nh;
    ros::Rate rate = ros::Rate(25);

    // Subscribe to RC
    ros::Publisher servo_pub = nh.advertise<mavros_msgs::OverrideRCIn>("rc/override", 10);

    mavros_msgs::OverrideRCIn rc_msg;
    ROS_INFO("init done");
    rc_msg.channels[6] = 1200;
    ROS_INFO("set value");

    ros::ServiceClient set_mode_client;
    set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");

    mavros_msgs::SetMode mode;
    mode.request.custom_mode = "OFFBOARD";

    for (int i; i < 500; i++)
    {
        servo_pub.publish(rc_msg);
        set_mode_client.call(mode);
        ros::spinOnce();
        rate.sleep();
    }

    for (int i = 1; i < 400; i++)
    {
        servo_pub.publish(rc_msg);
        ros::spinOnce();
        rate.sleep();
    }

    for (int i = 1; i < 400; i++)
    {
        rc_msg.channels[6] = 1900;
        servo_pub.publish(rc_msg);
        ros::spinOnce();
        rate.sleep();
    }
}