#include "headers/gdpdrone.h"

int main(int argc, char **argv)
{
    // Initialise node
    ros::init(argc, argv, "drone_node");

    // Create drone object, this sets everything up
    GDPdrone drone;

    // Set the rate. Default working frequency is 25 Hz
    float loop_rate = 25.0;
    ros::Rate rate = ros::Rate(loop_rate);

    // Initialise and Arm
    drone.Commands.await_Connection();
    drone.Commands.set_Offboard();
    drone.Commands.set_Armed();

    // MISSION STARTS HERE:
    // Request takeoff at 1m altitude. At 25Hz = 10 seconds
    float altitude = 5;
    int time_takeoff = 300;
    drone.Commands.request_Takeoff(altitude, time_takeoff);

    // Go one meter up and stay there. Total time 10 seconds
    ROS_INFO("Command 1");
    for (int count = 1; count < 125; count++)
    {
        drone.Commands.move_Position_Local(-2, 0, 5, 0, "BODY_OFFSET");
        ros::spinOnce();
        rate.sleep();
    }

    // Go one meter up and stay there. Total time 10 seconds
    ROS_INFO("Command 2");
    for (int count = 1; count < 125; count++)
    {
        drone.Commands.move_Position_Local(2, 0, 5, 0, "BODY_OFFSET");
        ros::spinOnce();
        rate.sleep();
    }

    // Go one meter up and stay there. Total time 10 seconds
    ROS_INFO("Command 3");
    for (int count = 1; count < 125; count++)
    {
        drone.Commands.move_Position_Local(0, 2, 5, 0, "BODY_OFFSET");
        ros::spinOnce();
        rate.sleep();
    }

    // Go one meter up and stay there. Total time 10 seconds
    ROS_INFO("Command 4");
    for (int count = 1; count < 125; count++)
    {
        drone.Commands.move_Position_Local(0, -2, 5, 0, "BODY_OFFSET");
        ros::spinOnce();
        rate.sleep();
    }

    // Land and disarm
    drone.Commands.request_LandingAuto();

    // Exit
    return 0;
}