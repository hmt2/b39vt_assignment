#include <iostream>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <ros/timer.h>
#include <boost/thread/mutex.hpp>
#include <nav_msgs/Path.h>


class RobotDriver
{
private:
    ros::NodeHandle nh_;
    ros::Publisher cmd_vel_pub_;
    ros::Subscriber scanSub_;
    // ros::Timer timer_;
    // bool turn;

public:
    //ROS node initialization
    RobotDriver(ros::NodeHandle &nh)
    {
        nh_ = nh;
        //set up the publisher for the cmd_vel topic
        cmd_vel_pub_= nh_.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 1);
        //set up the subscriber to laserscan
        scanSub_ = nh_.subscribe<sensor_msgs::LaserScan>("/scan",1, &RobotDriver::processLaserScan, this);
        //timer_ = nh_.createTimer(ros::Duration(36), &RobotDriver::timerCallback, this);//36
        //turn = false;
        //sleep(20);
    }

    /*  void timerCallback(const ros::TimerEvent& event)
    {
        turn = true;
    }
*/

    //find average method
    float findAverage(const std::vector<float>& range, int minRange, int maxRange)
    {
        float temp = 0;

        for(int i = minRange;i<maxRange;i++)
        {
            if(range[i] == INFINITY || isnan(range[i]) )
            {
                //inf++;
                temp = temp + 0;//0.5;
            }
            else
            {
                temp = temp + range[i];
            }
        }
        temp = temp/((maxRange-minRange));
        return temp;
    }
    //callback method
    void processLaserScan(const sensor_msgs::LaserScanConstPtr& scan)
    {
        geometry_msgs::Twist base_cmd;
        int inf = 0;
        float a, b, c, d, e, f;
        //default movement of robot
        base_cmd.linear.x = 0.1;
        base_cmd.angular.z = 0;
        cmd_vel_pub_.publish(base_cmd);
        //call findAverage method for each range segment
        a = findAverage(scan->ranges,0,90); //size 90
        b = findAverage(scan->ranges,90,180); //size 90
        c = findAverage(scan->ranges,180,256); //size 76
        d = findAverage(scan->ranges,256,332); //size 76
        e = findAverage(scan->ranges,332,422); //size 90
        f = findAverage(scan->ranges,422,512); //size 90
        /* printf("a = %f\n", a);
        printf("b = %f\n", b);
        printf("c = %f\n", c);
        printf("d = %f\n", d);
        printf("e = %f\n", e);
        printf("e = %f\n", f);
        */
        /*      while(turn)
        {
            //turn left 90deg
            base_cmd.linear.x = 0;
            base_cmd.angular.z = 0.5;

            for(int i = 0; i<12;i++)//12
            {
                cmd_vel_pub_.publish(base_cmd);
                ros::Duration(0.5).sleep();
            }

            //turn right 180deg
            base_cmd.linear.x = 0;
            base_cmd.angular.z = -0.5;

            for(int i = 0; i<20;i++)
            {

                cmd_vel_pub_.publish(base_cmd);
                ros::Duration(0.5).sleep();
            }

            //turn left 90
            base_cmd.linear.x = 0;
            base_cmd.angular.z = 0.5;

            for(int i = 0; i<12;i++)
            {
                cmd_vel_pub_.publish(base_cmd);
                ros::Duration(0.5).sleep();
            }

            turn = false;
        }
*/
        //set case conditions for switch statement for each segment
        int val;
        if(a<0.25)
        {
            val = 1;
        }
        if(b<0.35)
        {
            val = 2;
        }
        if(c<0.25)
        {
            val = 3;
        }
        if(d<0.25)
        {
            val = 4;
        }
        if(e<0.35)
        {
            val = 5;
        }
        if(f<0.25)
        {
            val = 6;
        }

        switch(val)
        {
        case 1 : //a<0.2
        {
            base_cmd.linear.x = 0;
            base_cmd.angular.z = 1;
            cmd_vel_pub_.publish(base_cmd);
        }
            break;
        case 2 : //b<0.3
        {
            base_cmd.linear.x = 0;
            base_cmd.angular.z = 1.5;
            cmd_vel_pub_.publish(base_cmd);
        }
            break;
        case 3 : //c<0.2
        {
            base_cmd.linear.x = 0;
            base_cmd.angular.z = 1;
            cmd_vel_pub_.publish(base_cmd);
            //sleep(1);
        }
            break;
        case 4 : //d<0.2
        {
            base_cmd.linear.x = 0;
            base_cmd.angular.z = -0.5;
            cmd_vel_pub_.publish(base_cmd);
            //sleep(1);
        }
            break;
        case 5 : //e<0.3
        {
            base_cmd.linear.x = 0;
            base_cmd.angular.z = -1.5;
            cmd_vel_pub_.publish(base_cmd);
        }
            break;
        case 6 : //f<0.2
        {
            base_cmd.linear.x = 0;
            base_cmd.angular.z = -1;
            cmd_vel_pub_.publish(base_cmd);
        }
            break;
        default : //if no case conditions met, default to moving forward
            base_cmd.linear.x = 0.1;
            cmd_vel_pub_.publish(base_cmd);

        }
    }
};
int main(int argc, char** argv)
{
    ros::init(argc, argv, "robot_driver");
    ros::NodeHandle nh;
    RobotDriver driver(nh);
    ros::spin();

    //printf("0");
}

