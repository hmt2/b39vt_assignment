#include <iostream>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>

class RobotDriver
{
private:
    //The node handle we'll be using
    ros::NodeHandle nh_;
    //We will be publishing to the "/base_controller/command" topic to issue commands
    ros::Publisher cmd_vel_pub_;
    ros::Subscriber scanSub;

public:
    //ROS node initialization
    RobotDriver(ros::NodeHandle &nh)
    {
        nh_ = nh;
        //set up the publisher for the cmd_vel topic
        cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 1);
        //set up the subscriber to laserscan
        scanSub = nh_.subscribe<sensor_msgs::LaserScan>("/scan",1, &RobotDriver::processLaserScan, this);
    }


    void processLaserScan(const sensor_msgs::LaserScanConstPtr& scan)
    {
        geometry_msgs::Twist base_cmd;

        int inf = 0;
        float av = 0;

        for(int i = 235; i<276; i++)
        {
            if(scan->ranges[i] == INFINITY || isnan(scan->ranges[i]))
            {
                inf++;
                av = av + 0;
            }
            else
            {
                av = av + scan->ranges[i];
            }
        }

        av = av/(40-inf);
        printf("%f\n", av);

        if(av<0.2)
        {
            printf("x");
            base_cmd.linear.x = 0;
            //base_cmd.linear.y = 0.25;
            base_cmd.angular.z = 0.8;
            base_cmd.angular.x = -0.2;
        }

        cmd_vel_pub_.publish(base_cmd);

    }

};


int main(int argc, char** argv)
{
    ros::init(argc, argv, "robot_driver");
    ros::NodeHandle nh;
    printf("0");
    RobotDriver driver(nh);
    ros::spin();
}
