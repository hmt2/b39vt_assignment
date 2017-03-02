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

    float findAverage(const std::vector<float>& range, int minRange, int maxRange)
    {
        float temp = 0;
        int inf = 0;


        for(int i = minRange;i<maxRange;i++)
        {

            if(range[i] == INFINITY || isnan(range[i]) )
            {
                inf++;
                temp = temp + 0;
            }
            else
            {
                temp = temp + range[i];

            }
        }
        temp = temp/((maxRange-minRange)-inf);

        return temp;
    }


    void processLaserScan(const sensor_msgs::LaserScanConstPtr& scan)
    {
        geometry_msgs::Twist base_cmd;
        int inf = 0;
        float a, b, c, c1, d, e;
        //float front, left, right;

        base_cmd.linear.x = 0.1;
        cmd_vel_pub_.publish(base_cmd);

        a = findAverage(scan->ranges,0,90); //size 90
        b = findAverage(scan->ranges,90,180); //size 90
        c = findAverage(scan->ranges,180,256); //size 76
        c1 = findAverage(scan->ranges,256,332); //size 76
        d = findAverage(scan->ranges,332,422); //size 90
        e = findAverage(scan->ranges,422,512); //size 90
        printf("a = %f\n", a);
        printf("b = %f\n", b);
        printf("c = %f\n", c);
        printf("d = %f\n", d);
        printf("e = %f\n", e);

        //set cases for switch statement
        int val;

        if(a<0.2)
        {
            val = 1;
        }

        if(b < 0.3)
        {
            val = 2;
        }

        if(c < 0.2)
        {
            val = 4;
        }

        if (c1<0.2)
        {
            val = 44;
        }

        if(d < 0.3)
        {
            val =6;
        }

        if(e < 0.2)
        {
            val = 8;
        }


        switch(val)
        {

        case 1 :
        {
            base_cmd.linear.x = 0;
            base_cmd.angular.z = 1;

            cmd_vel_pub_.publish(base_cmd);
        }
            break;

        case 2 :
        {
            base_cmd.linear.x = 0;
            base_cmd.angular.z = 2;
            cmd_vel_pub_.publish(base_cmd);
        }
            break;

     /*   case 3 :
        {
            base_cmd.linear.x = 0;
            base_cmd.angular.z = 1;
            //  base_cmd.angular.x = -1;

            cmd_vel_pub_.publish(base_cmd);
        }
            break;
*/
        case 4 :
        {
            base_cmd.linear.x = 0;
            base_cmd.angular.z = 1.5;

            cmd_vel_pub_.publish(base_cmd);
            sleep(1);
        }
            break;

        case 44 :
        {
            base_cmd.linear.x = 0;
            base_cmd.angular.z = -1;

            cmd_vel_pub_.publish(base_cmd);
            sleep(1);
        }
            break;
  /*      case 5 :
        {
            base_cmd.linear.x = 0;
            base_cmd.angular.z = 1.5;

            cmd_vel_pub_.publish(base_cmd);
            sleep(1);
        }
            break;
*/
        case 6 :
        {
            base_cmd.linear.x = 0;
            base_cmd.angular.z = -2;

            cmd_vel_pub_.publish(base_cmd);
        }
            break;

 /*       case 7 :
        {
            base_cmd.linear.x = 0;
            base_cmd.angular.z = 1;
            //  base_cmd.angular.x = -1;

            cmd_vel_pub_.publish(base_cmd);
        }
            break;
*/
        case 8 :
        {
            base_cmd.linear.x = 0;
            base_cmd.angular.z = -1;

            cmd_vel_pub_.publish(base_cmd);
        }
            break;

        default :
            base_cmd.linear.x = 0.1;
        }


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
