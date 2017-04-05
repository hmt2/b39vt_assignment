#include <b39vt_assignment/image_processing.hpp>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/String.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/PointStamped.h>


#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <tf/transform_listener.h>
#include <tf/tf.h>

class ImageSubscriber
{
    // Start the node if not already started
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    image_transport::Publisher image_pub_;
    ros::Subscriber scanSub_;

    //publishers for point stamp
    ros::Publisher biohazardPub_;
    ros::Publisher dangerPub_;
    ros::Publisher firePub_;
    ros::Publisher alivePub_;
    ros::Publisher deadPub_;
    ros::Publisher radioactivePub_;
    ros::Publisher smokingPub_;
    ros::Publisher toxicPub_;

    /*
    //publishers for pose stamp
    ros::Publisher biohazardPub2_;
    ros::Publisher dangerPub2_;
    ros::Publisher firePub2_;
    ros::Publisher alivePub2_;
    ros::Publisher deadPub2_;
    ros::Publisher radioactivePub2_;
    ros::Publisher smokingPub2_;
    ros::Publisher toxicPub2_;*/

    float distance;


public:
    // Public variables
    cv_bridge::CvImagePtr cv_ptr;

    bool data_valid;

public:
    // Public methods
    ImageSubscriber() : it_(nh_), data_valid(false)
    {
        // Subscribe to input video feed
        image_sub_ = it_.subscribe("/camera/rgb/image_raw", 1,
                                   &ImageSubscriber::imageCb, this);
        scanSub_ = nh_.subscribe<sensor_msgs::LaserScan>("/scan",1, &ImageSubscriber::processLaserScan, this);

        //initialize publishers for point stamp
        biohazardPub_ = nh_.advertise<geometry_msgs::PointStamped>("/biohazard_match", 1);
        dangerPub_ = nh_.advertise<geometry_msgs::PointStamped>("/danger_match", 1);
        firePub_ = nh_.advertise<geometry_msgs::PointStamped>("/fire_match", 1);
        alivePub_ = nh_.advertise<geometry_msgs::PointStamped>("/alive_match", 1);
        deadPub_ = nh_.advertise<geometry_msgs::PointStamped>("/dead_match", 1);
        radioactivePub_ = nh_.advertise<geometry_msgs::PointStamped>("/radioactive_match", 1);
        smokingPub_ = nh_.advertise<geometry_msgs::PointStamped>("/smoking_match", 1);
        toxicPub_ = nh_.advertise<geometry_msgs::PointStamped>("/toxic_match", 1);

        /*
        //initialize publishers for pose stamp
        biohazardPub2_ = nh_.advertise<geometry_msgs::PoseStamped>("/biohazard_match", 1);
        dangerPub2_ = nh_.advertise<geometry_msgs::PoseStamped>("/biohazard_match", 1);
        firePub2_ = nh_.advertise<geometry_msgs::PoseStamped>("/biohazard_match", 1);
        alivePub2_ = nh_.advertise<geometry_msgs::PoseStamped>("/biohazard_match", 1);
        deadPub2_ = nh_.advertise<geometry_msgs::PoseStamped>("/biohazard_match", 1);
        radioactivePub2_ = nh_.advertise<geometry_msgs::PoseStamped>("/biohazard_match", 1);
        smokingPub2_ = nh_.advertise<geometry_msgs::PoseStamped>("/biohazard_match", 1);
        toxicPub2_ = nh_.advertise<geometry_msgs::PoseStamped>("/biohazard_match", 1);*/
    }


    // Access cv_ptr in main method
    cv::Mat getImage()
    {
        return cv_ptr->image;
    }
    // Laser callback function
    void processLaserScan(const sensor_msgs::LaserScanConstPtr& scan)
    {
        for(int i = 256;i>0;i--)
        {

            if(scan->ranges[i] == INFINITY || isnan(scan->ranges[i]) )
            {
                distance = 0;
            }
            else
            {
                distance = scan->ranges[i];
                //coord = distance + tfcoord();
                break;
            }
        }


    }

    void getDistance(int i)
    {
        geometry_msgs::PointStamped p;

        //float coord;

        //sets x distance to be dist calculated above
        p.header.frame_id = "/base_link";
        p.point.x = distance;
        p.point.y = 0;
        p.point.z = 0;
        /*
        tf::TransformListener listener;


            geometry_msgs::PoseStamped pBase, pMap;
            pBase.header.frame_id = "/base_link";
            pBase.pose.position.x = 0.0;
            pBase.pose.position.y = 0.0;
            pBase.pose.orientation = tf::createQuaternionMsgFromYaw(0.0);
            ros::Time current_transform = ros::Time::now();
            listener.getLatestCommonTime(pBase.header.frame_id, "/map", current_transform, NULL);
            pBase.header.stamp = current_transform;
            listener.transformPose("/map", pBase, pMap);
            // pMap now contains the pose of the robot transformed into map
            // coordinates according to the TF data available at time "current_transform"*/


        //publishes to topic depending on match
        //commented code is for pose stamp
        switch(i)
        {
        case 0:
            //ROS_INFO("Biohazard match: %f\n", coord);
            biohazardPub_.publish(p);
            // biohazardPub2_.publish(listener);
            break;
        case 1:
            //ROS_INFO("Danger match: %f\n", coord);
            dangerPub_.publish(p);
            //dangerPub2_.publish(listener);
            break;
        case 2:
            //ROS_INFO("Fire match: %f\n", coord);
            firePub_.publish(p);
            // firePub2_.publish(listener);
            break;
        case 3:
            //ROS_INFO("Alive match: %f\n", coord);
            alivePub_.publish(p);
            // alivePub2_.publish(listener);
            break;
        case 4:
            //ROS_INFO("Dead match: %f\n", coord);
            deadPub_.publish(p);
            //deadPub2_.publish(listener);
            break;
        case 5:
            //ROS_INFO("Radioactive match: %f\n", coord);
            radioactivePub_.publish(p);
            // radioactivePub2_.publish(listener);
            break;
        case 6:
            //ROS_INFO("Smoking match: %f\n", coord);
            smokingPub_.publish(p);
            // smokingPub2_.publish(listener);
            break;
        case 7:
            //ROS_INFO("Toxic match: %f\n", coord);
            toxicPub_.publish(p);
            //s toxicPub2_.publish(listener);
            break;
        }

    }


    // Convert a sensor_msgs::image message to an OpenCV-compatible image
    void imageCb(const sensor_msgs::ImageConstPtr& msg)
    {
        try
        {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
            data_valid = true;
        }

        catch (cv_bridge::Exception& e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }
    }


    // Set threshold for each template
    double getThreshold(int i)
    {
        double threshold;

        switch(i)
        {
        case 0:
            threshold = 1.1;//1.3;  Biohazard
            break;
        case 1:
            threshold = 1.5;//1.7; // Danger
            break;
        case 2:
            threshold = 1.1; // Fire
            break;
        case 3:
            threshold = 0.75;//1.3; // Alive
            break;
        case 4:
            threshold = 0.9;//1.6; // Dead
            break;
        case 5:
            threshold = 1.2;//1.4; // Radioactive
            break;
        case 6:
            threshold = 1.5;//1.95; // Smoking
            break;
        case 7:
            threshold = 0.85;//1.6; // Toxic
            break;
        }
        return threshold;
    }
};


int main(int argc, char** argv)
{
    ros::init(argc, argv, "image_subscriber");
    ImageSubscriber ic;
    bool matchFound;



    // Create vector of pairs
    std::vector< std::pair<cv::Mat, char*> > templ;

    // Fill the vector with pair of the template and string name of template
    templ.push_back(std::pair<cv::Mat, char*>(cv::imread("/home/calum/Pictures/Test_Images/biohazard.png"),"Biohazard"));
    templ.push_back(std::pair<cv::Mat, char*>(cv::imread("/home/calum/Pictures/Test_Images/danger.png"),"Danger"));
    templ.push_back(std::pair<cv::Mat, char*>(cv::imread("/home/calum/Pictures/Test_Images/fire.png"),"Fire"));
    templ.push_back(std::pair<cv::Mat, char*>(cv::imread("/home/calum/Pictures/Test_Images/alive.png"),"Alive"));
    templ.push_back(std::pair<cv::Mat, char*>(cv::imread("/home/calum/Pictures/Test_Images/dead.png"),"Dead"));
    templ.push_back(std::pair<cv::Mat, char*>(cv::imread("/home/calum/Pictures/Test_Images/radioactive.png"),"Radioactive"));
    templ.push_back(std::pair<cv::Mat, char*>(cv::imread("/home/calum/Pictures/Test_Images/smoking2.png"),"Smoking"));
    templ.push_back(std::pair<cv::Mat, char*>(cv::imread("/home/calum/Pictures/Test_Images/toxic.png"),"Toxic"));



    while (ros::ok())
    {
        if (ic.data_valid)
        {
            // Iterate through vector and call templateMatching()
            for(int i = 0;i < 8; i++)
            {

                cv::Mat tem = templ[i].first;
                char* sign = templ[i].second;
                //Template size;
                cv::Size size(210,210);
                //Resize image
                cv::resize(tem,tem,size);

                matchFound = templateMatching(ic.getImage(), tem, sign, ic.getThreshold(i));
                if( matchFound )// if match found call function to find distance
                {
                    ic.getDistance(i);
                }
            }
        }

        ros::spinOnce();
    }

    return 0;
}
