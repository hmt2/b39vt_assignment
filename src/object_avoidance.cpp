#include <iostream>

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

class RobotDriver
{
    private:
      //! The node handle we'll be using
      ros::NodeHandle nh_;
      //! We will be publishing to the "/base_controller/command" topic to issue commands
      ros::Publisher cmd_vel_pub_;

    public:
      //! ROS node initialization
      RobotDriver(ros::NodeHandle &nh)
      {
        nh_ = nh;
        //set up the publisher for the cmd_vel topic
        cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 1);
      }

    void move_robot()
      {
        geometry_msgs::Twist base_cmd;
        char cmd[50];

				printf("1");
				
        while(nh_.ok())
        {
          //std::cin.getline(cmd, 50);
          base_cmd.linear.x = 0.25;
        	cmd_vel_pub_.publish(base_cmd);
        }

      }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "robot_driver");
    ros::NodeHandle nh;
		
		printf("0");
		
    RobotDriver driver(nh);
    driver.move_robot();

}
