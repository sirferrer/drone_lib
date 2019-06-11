#include "Dstar.h"
#include "headers/gdpdrone.h"
#include "iostream"

int approx(float a)
{
    int A = (int)floor(a * 10);
    A -= A % 6;
    return A;
}

int main(int argc, char **argv)
{
    // Initialise node
    ros::init(argc, argv, "lucian_node");

    GDPdrone drone;

    // Set the rate. Default working frequency is 25 Hz
    float loop_rate = 10.0;
    ros::Rate rate = ros::Rate(loop_rate);

    // Initialise and Arm
    drone.Commands.await_Connection();
    drone.Commands.set_Offboard();
    drone.Commands.set_Armed();

    Dstar *dstar = new Dstar();
    list<state> mypath;

    // MISSION STARTS HERE:
    // Request takeoff at 1m altitude. At 25Hz = 10 seconds
    float altitude = 1.0;
    int time_takeoff = 100;
    float k = 0.8;
    drone.Commands.request_Takeoff(altitude, time_takeoff);

    float fin=15.0;
    float pitch = 4.35;
    int xobst, yobst;
    int Count;

    //float fin=drone.Data.local_pose.pose.position.x;

    if (!(drone.Data.lidar.ranges[3] == INFINITY && drone.Data.lidar.ranges[4] == INFINITY && drone.Data.lidar.ranges[5] == INFINITY))
    {
        while (!(drone.Data.lidar.ranges[3] == INFINITY && drone.Data.lidar.ranges[4] == INFINITY && drone.Data.lidar.ranges[5] == INFINITY))
        {
            ROS_INFO("CE MA?");
            xobst = approx(drone.Data.lidar.ranges[1] * cos(pitch * M_PI / 180));
            yobst = xobst;
            Count = 0;
            while (Count != 20)
            {
                drone.Commands.move_Position_Local(0, 0, k, 0, "BODY_OFFSET", Count);
                Count++;
                ros::spinOnce();
                rate.sleep();
            }
            if (drone.Data.local_pose.pose.position.x > fin)
                {
                    drone.Commands.request_LandingAuto();
                    break;
                }
        }
    }
    else
    {
        while (drone.Data.lidar.ranges[3] == INFINITY && drone.Data.lidar.ranges[4] == INFINITY && drone.Data.lidar.ranges[5] == INFINITY && drone.Data.lidar.ranges[1] > k)
        {
            drone.Commands.move_Velocity_Local(0, 0.5, 0, 0, "BODY_OFFSET");
            ros::spinOnce();
            rate.sleep();
            xobst = approx(drone.Data.lidar.ranges[1] * cos(pitch * M_PI / 180)); //this should be out of HERE!
            if (drone.Data.local_pose.pose.position.x > fin)
            {
                drone.Commands.request_LandingAuto();
                break;
            }
        }

        if (drone.Data.lidar.ranges[1] <= k)
        {
            Count = 0;

            while (Count != 20)
            {
                drone.Commands.move_Position_Local(0, 0, k, 0, "BODY_OFFSET", Count);
                Count++;
                ros::spinOnce();
                rate.sleep();
            }
        }
        yobst = approx(drone.Data.lidar.ranges[1] * cos(pitch * M_PI / 180)); //* ((45 - pitch) * M_PI / 180));
    }

    ROS_INFO("xobst [%i]", xobst);
    ROS_INFO("yobst [%i]", yobst);

    int distance = 400; //gps distance in m between goal and where I am
    int resolution = 6; //Resolution is 2/10 m;
    int Altitude = (int)altitude;

    dstar->init(0, Altitude, distance / resolution, 0);
    for (int i = -distance / resolution; i <= distance / resolution; i++)
        dstar->updateCell(i, -1, -1); //set the ground to be non-traversable

    for (int i = 0; i <= yobst / resolution + 1; i++)
        dstar->updateCell(xobst / resolution, i, -1);

    int xst = 0, yst = 0;

    while (!(xst == distance / resolution && yst == 0))
    {
        dstar->replan();           // plan a path
        mypath = dstar->getPath(); // retrieve path
        ROS_INFO("xst[%i]", xst);
        ROS_INFO("yst[%i]", yst);
        for (auto &v : mypath)
        {
            dstar->updateStart(v.x, v.y); // move start to new path point
            ROS_INFO("v.x [%i]", v.x);
            ROS_INFO("v.y [%i]", v.y);
            Count = 0;

            while (Count != 20)
            {
                drone.Commands.move_Position_Local(0, (v.x - xst) * k, (v.y - yst) * k, 0, "BODY_OFFSET", Count);
                Count++;
                ros::spinOnce();
                rate.sleep();
            }
            xst = v.x;
            yst = v.y;
            if (drone.Data.local_pose.pose.position.x > fin)
                {
                    drone.Commands.request_LandingAuto();
                    break;
                }
            if (v.x == xobst / resolution && v.y == (yobst / resolution + 2))
            {
                ROS_INFO("COITE");
                if (!(drone.Data.lidar.ranges[3] == INFINITY && drone.Data.lidar.ranges[4] == INFINITY && drone.Data.lidar.ranges[5] == INFINITY))
                {
                    while (!(drone.Data.lidar.ranges[3] == INFINITY && drone.Data.lidar.ranges[4] == INFINITY && drone.Data.lidar.ranges[5] == INFINITY))
                    {
                        xobst = approx(drone.Data.lidar.ranges[1] * cos(pitch * M_PI / 180));
                        yobst = xobst;
                        Count = 0;
                        while (Count != 20)
                        {
                            drone.Commands.move_Position_Local(0, 0, k, 0, "BODY_OFFSET", Count);
                            Count++;
                            ros::spinOnce();
                            rate.sleep();
                        }
                        if (drone.Data.local_pose.pose.position.x > fin)
                        {
                            drone.Commands.request_LandingAuto();
                            break;
                        }
                    }
                }
                else
                {
                    while (drone.Data.lidar.ranges[3] == INFINITY && drone.Data.lidar.ranges[4] == INFINITY && drone.Data.lidar.ranges[5] == INFINITY && drone.Data.lidar.ranges[1] > 0.4)
                    {
                        drone.Commands.move_Velocity_Local(0, 0.5, 0, 0, "BODY_OFFSET");
                        ros::spinOnce();
                        rate.sleep();
                        xobst = approx(drone.Data.lidar.ranges[1] * cos(pitch * M_PI / 180)); //this should be out of HERE!
                        if (drone.Data.local_pose.pose.position.x > fin)
                        {
                            drone.Commands.request_LandingAuto();
                            break;
                        }
                    }
                    /*for (int j = 0; j < 10; j++)
                        {
                            drone.Commands.move_Position_Local(0, 0, 0, 0, "BODY_OFFSET", j);
                            ros::spinOnce();
                            rate.sleep();
                        }*/
                    if (drone.Data.lidar.ranges[1] <= 0.4)
                    {
                        ROS_INFO("AM");
                        Count = 0;

                        while (Count != 20)
                        {
                            drone.Commands.move_Position_Local(0, 0, k, 0, "BODY_OFFSET", Count);
                            Count++;
                            ros::spinOnce();
                            rate.sleep();
                        }
                    }
                    yobst = approx(drone.Data.lidar.ranges[1] * cos(pitch * M_PI / 180)); //* ((45 - pitch) * M_PI / 180));
                }

                ROS_INFO("xobst [%i]", xobst);
                ROS_INFO("yobst [%i]", yobst);

                int Count;
                int distance = 100; //gps distance in m between goal and where I am
                int resolution = 5; //Resolution is 2/10 m;
                int Altitude = (int)altitude;

                dstar->init(0, Altitude, distance / resolution, 0);
                for (int i = -distance / resolution; i <= distance / resolution; i++)
                    dstar->updateCell(i, -1, -1); //set the ground to be non-traversable

                for (int i = 0; i <= yobst / resolution + 1; i++)
                    dstar->updateCell(xobst / resolution, i, -1);
                xst = 0;
                yst = 0;
                //ROS_INFO("AM COAIELE MARI");
                break;
            }
            ros::spinOnce();
            rate.sleep();
        }
    }

    // Land and disarm
    drone.Commands.request_LandingAuto();
    return 0;
}
