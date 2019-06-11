#include "headers/gdpdrone.h"
#include <fstream>
#include <string.h>
#include <iterator>
#include <random>
#include "headers/jakelibrary.h"
#include <cmath>

void storePosition(struct MissionPositions STEP, float x, float y, float z, float yaw);
bool detectObstacle(float right, float centre, float left, float range);
float degtorad(float a);
float radtodeg(float a);
float distanceToObstacle(float right, float centre, float left, float range);
float orientationToObstacle(float right, float centre, float left, float range, float angle_max, float angle_min);
bool detectIfOverObstacle(float a, float b);
float propControl(float aNow, float aOld, float b);

struct MissionPositions
{
    float x, y, z;
    float yaw;
};

struct positions
{
    MissionPositions InitialCollection;
    MissionPositions FinalCollection;
    MissionPositions InitialObstacleAvoidance;
    MissionPositions FinalObstacleAvoidance;
    MissionPositions InitialTracking;
    MissionPositions FinalTracking;
};

struct Avoidance
{
    float horizontal;
    float vertical;
    float orientation;
};

struct obstacledistance
{
    std::vector<float> left = {0.0};
    std::vector<float> centre = {0.0};
    std::vector<float> right = {0.0};
    std::vector<float> xPosition = {0.0};
    std::vector<float> yPosition = {0.0};
};

positions POSITIONS;
Avoidance avoidance;
obstacledistance ObstacleDistance;

int main(int argc, char **argv)
{
    // Initialise node
    ros::init(argc, argv, "mission_full_novision_node");

    // Create drone object, this sets everything up
    GDPdrone drone;

    // Set the rate. Default working frequency is 25 Hz
    float loop_rate = 10.0;
    ros::Rate rate = ros::Rate(loop_rate);

    // Initialise and Arm
    drone.Commands.await_Connection();
    drone.Commands.set_Offboard();
    drone.Commands.set_Armed();

    // MISSION STARTS HERE:
    // Request takeoff at 1m altitude. At 25Hz = 10 seconds
    float altitude = 4.0;
    int time_takeoff = 50;
    drone.Commands.request_Takeoff(altitude, time_takeoff);



    ///<-------------------------------------------- STEP 3 ---- AMBULANCE PN TRACKING ---------------------------------------_>




    float setAltitude = 5.77f;

    // Start Mission; Initialize the relative velocities
    InitialiseJakeCode(drone.Data.target_position_relative.point.x, drone.Data.target_position_relative.point.y, drone.Data.target_position_relative.point.z);

    // Command 1, set drone velocity to the calculated initial velocity in 1 second.
    ROS_INFO("Initialising drone velocity");

    // Change this to a while loop comparing measured drone velocity and commanded drone velocity
    drone.Commands.Initialise_Velocity_for_AccelCommands(droneVel[1], droneVel[0], -droneVel[2]);

    float initial_yaw = atan2(droneVel[0], droneVel[1]) * 180.0 / M_PI;
    // turn drone to point in that direction
    ROS_INFO("Turning to direction of inital velocity: %f degrees", initial_yaw); // assume 10 degrees / second?
    for (int count = 1; count < 60; count++)
    {
        drone.Commands.move_Position_Local(0.0f, 0.0f, 0.0f, initial_yaw, "LOCAL_OFFSET", count);
        ros::spinOnce();
        rate.sleep();
    }

    // current yaw angle given by the accelerations
    float yawAcceleration;

    // buffer which stores actual and previous values of actual yaw
    boost::circular_buffer<float> yawError(2);
    yawError[0] = 0.0;
    yawError[1] = 0.0;

    // Kp declaration
    std::vector<float> kp;
    float kpCurrent;

    do
    {
        ROS_INFO("%f");
        droneAccComp(relPos, relVel, droneAcc);
        accFix = altitudeFix(drone.Data.target_position_relative.point.z, setAltitude);
        ROS_INFO("Accelerations needed: x: %f, y: %f, z: %f", droneAcc[1], droneAcc[0], droneAcc[2]);

        yawAcceleration = atan2(droneAcc[1], droneAcc[0]) * 180 / M_PI;
        yawError.push_back(yawAcceleration);
        kp.push_back(propControl(yawError[0], yawError[1], drone.Data.compass_heading.data));
        drone.Commands.move_Acceleration_Local_Trick(droneAcc[1], droneAcc[0], accFix, "LOCAL_OFFSET", loop_rate);
        


        for (int i = 0; i < 3; ++i)
        {
            relPosOld[i] = relPos[i];
        }

        relPos[0] = drone.Data.target_position_relative.point.y;
        relPos[1] = drone.Data.target_position_relative.point.x;
        relPos[2] = 0;

        gpsdistance = norm(relPos);

        ROS_INFO("Distance to target: %f", gpsdistance);

        velFromGPS(relPos, relPosOld, loop_rate, relVel);


        ros::spinOnce();
        rate.sleep();



    }
    while (gpsdistance > switchDist);







////<----------------------------------------------------GPS Landing------------------------------------------------------>
     ROS_INFO("Track Ambulance");
    for (int count = 1; count < 200; count++)
    {
        float yaw_angle = drone.Data.CalculateYawAngleToTarget();
        std::cout << "Yaw angle is: " << yaw_angle << " deg" << std::endl;
        // drone.Commands.move_Position_Global(drone.Data.gps_raw.latitude, drone.Data.gps_raw.longitude, drone.Data.gps_raw.altitude + 5.0f, yaw_angle, "BODY");
        drone.Commands.move_Velocity_Local_Gerald(1.50, yaw_angle, "BODY");
        ros::spinOnce();
        rate.sleep();
    }

    // Land and disarm
    ROS_INFO("Landing Now");
    drone.Commands.request_LandingAuto();


    return 0;
}














////<-------------------------------Function definitions----------------------->
void storePosition(struct MissionPositions STEP, float x, float y, float z, float yaw)
{
    STEP.x = x;
    STEP.y = y;
    STEP.z = z;
    STEP.yaw = yaw;
}

bool detectObstacle(float right, float centre, float left, float range)
{
    bool isObstacle;
    if ((right < range) || (centre < range) || (left < range))
        isObstacle = 1;
    else
        isObstacle = 0;
    return isObstacle;
}

float degtorad(float a)
{
    return a * M_PI / 180;
}

float radtodeg(float a)
{
    return a * 180 / M_PI;
}

float distanceToObstacle(float right, float centre, float left, float range)
{
    float distance;
    if (isfinite(right) == 1 && isfinite(centre) == 1 && isfinite(left) == 1)
        distance = std::max(std::max(right, centre), left);
    else if (isfinite(right) == 0)
        distance = std::max(centre, left);
    else if (isfinite(left) == 0)
        distance = std::max(centre, right);
    else if (isfinite(centre) == 0)
        distance = std::max(left, right);
    else if (isfinite(right) == 0 && isfinite(centre) == 0)
        distance = left;
    else if (isfinite(right) == 0 && isfinite(left) == 0)
        distance = centre;
    else if (isfinite(centre) == 0 && isfinite(left) == 0)
        distance = left;
    else if (isfinite(right) == 0 && isfinite(centre) == 0 && isfinite(left) == 0)
        distance = range * 2;
    return distance;
}

float orientationToObstacle(float right, float centre, float left, float range, float angle_max, float angle_min)
{
    float distance, angleOr;
    if (isfinite(right) == 1 && isfinite(centre) == 1 && isfinite(left) == 1)
        distance = std::max(std::max(right, centre), left);
    else if (isfinite(right) == 0)
        distance = std::max(centre, left);
    else if (isfinite(left) == 0)
        distance = std::max(centre, right);
    else if (isfinite(centre) == 0)
        distance = std::max(left, right);
    else if (isfinite(right) == 0 && isfinite(centre) == 0)
        distance = left;
    else if (isfinite(right) == 0 && isfinite(left) == 0)
        distance = centre;
    else if (isfinite(centre) == 0 && isfinite(left) == 0)
        distance = left;
    else if (isfinite(right) == 0 && isfinite(centre) == 0 && isfinite(left) == 0)
        distance = range * 2;

    if (distance == right)
        angleOr = angle_max;
    else if (distance == left)
        angleOr = angle_min;
    else if (distance == centre)
        angleOr = 0;

    return angleOr;
}

bool detectIfOverObstacle(float a, float b)
{
    if (a < 0.5 * b)
        return 1;
    else
        return 0;
}

float propControl(float aNow, float aOld, float b)
{
    float error = aNow;
    float diff = aNow - b;

    return error / diff;
}