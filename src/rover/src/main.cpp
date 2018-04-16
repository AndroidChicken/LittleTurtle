#include <unistd.h>             //Needed for I2C port
#include <fcntl.h>              //Needed for I2C port
#include <sys/ioctl.h>          //Needed for I2C port
#include <linux/i2c-dev.h>      //Needed for I2C port

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <std_msgs/String.h>
#include <std_msgs/Int8MultiArray.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <nav_msgs/Odometry.h>
#include <sstream>
#include <unistd.h>
#include "astar.h"
#include "bno055.h"

#ifndef M_PI
#define M_PI 3.14159265359
#endif

const float HeadingCvt = M_PI*2.0f/(5887.0f - 128.0);
int i2cFile = 0;
AStar astar;

//*****************************************************************************
// Handle twist message
//*****************************************************************************
void twistMessage(const geometry_msgs::Twist& twist)
{
    int8_t velLeft = static_cast<int8_t>(twist.linear.x);
    int8_t velRight = static_cast<int8_t>(twist.linear.x);

    // compute difference in wheel velocity to achive angular velociy
    // Just a simple scalar conversion for now
    velLeft += twist.angular.z *10;
    velRight -= twist.angular.z*10;

    astar.setMotor(i2cFile, velLeft, velRight);
}


//*****************************************************************************
// Convert IMU x axis to heading in radians
//*****************************************************************************
float toRads(int16_t heading)
{
    return (float)(heading - 128)*HeadingCvt;
}


//*****************************************************************************
//*****************************************************************************
int main(int argc, char** argv)
{
    ros::init(argc, argv, "rover");
    ros::NodeHandle nh;
    AStar::State astarState = {0};
    AStar::State prevState = {0};
    BNO055 imu(BNO055_ADDRESS_A);

    ROS_INFO("Opening i2c port");

    // make sure we can talk to the AStar
    char *filename = (char*)"/dev/i2c-1";
    if ((i2cFile = open(filename, O_RDWR)) < 0)
    {
        ROS_INFO("Failed to open i2c device\n");
        return  -1;
    }

    ROS_INFO("Initializing BNO055 imu");

    // Init the imu
    if (!imu.begin(i2cFile, BNO055::OPERATION_MODE_NDOF))
    {
        ROS_INFO("Failed to init the IMU");
        return -1;
    }

    ROS_INFO("Waiting for imu calibration");
    while(!imu.isFullyCalibrated())
    {
        usleep(100000);
    }
    ROS_INFO("imu calibrated");


    ros::Subscriber sb = nh.subscribe("rover/cmd_vel", 50, &twistMessage);

    ros::Publisher posePub = nh.advertise<geometry_msgs::Pose>("rover/pose", 50);
    ros::Publisher calPub = nh.advertise<std_msgs::Int8MultiArray>("rover/calibration", 50);
    ros::Publisher odomPub = nh.advertise<nav_msgs::Odometry>("odom", 50);

    tf::TransformBroadcaster tfBroadcaster;

    #define LOOP_RATE 60
    ros::Rate loop_rate(LOOP_RATE); // hz
    uint16_t loopCount = 0;

    nav_msgs::Odometry odom;
    BNO055::Vector imuVect;
    astar.readState(i2cFile, prevState);
     
    while (ros::ok())
    {
        ros::spinOnce();

        // get direction
        imu.getVector(&imuVect);

        // Get astar state which includes odometers
        astar.readState(i2cFile, astarState);

        // Compute change in position
        float distance = (astarState.left - prevState.left + astarState.right - prevState.right)/2.0f;
        prevState = astarState;

        float deltaX = distance * cos(imuVect.x);
        float deltaY = distance * sin(imuVect.x);

        odom.pose.pose.position.x += deltaX;
        odom.pose.pose.position.y += deltaY;
        odom.pose.pose.position.z = 0.0f;
        odom.pose.pose.orientation = tf::createQuaternionMsgFromYaw(imuVect.x);

        // publish pose
        odomPub.publish(odom);

        // Send tranform for laser
        tfBroadcaster.sendTransform(
            tf::StampedTransform(
                tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0, 0, .01)),
                ros::Time::now(),
                "base_link",
                "base_laser"));

        loop_rate.sleep();
        loopCount++;
    }

    // Send opometry transform
    astar.setMotor(i2cFile, 0, 0);

    return 0;
}
