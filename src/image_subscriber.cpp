#include "b39vt_assignment/image_processing.hpp"

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include "geometry_msgs/Point.h"
#include "std_msgs/String.h"

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

class ImageSubscriber
    {
      // Start the node if not already started
      ros::NodeHandle nh_;
      image_transport::ImageTransport it_;
      image_transport::Subscriber image_sub_;
      image_transport::Publisher image_pub_;


      public:
      // Public variables
      cv_bridge::CvImagePtr cv_ptr;

      bool data_valid;

      public:
      // Public methods
      ImageSubscriber() : it_(nh_), data_valid(false)
          {
              // Subscribe to input video feed
              image_sub_ = it_.subscribe("/usb_cam/image_raw", 1, 
              &ImageSubscriber::imageCb, this);
          }


      // Access cv_ptr in main method
      cv::Mat getImage()
          { 
              return cv_ptr->image;
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
                    threshold = 1.3; // Biohazard
                    break;
                case 1:
                    threshold = 1.7; // Danger
                    break;
                case 2:
                    threshold = 1.4; // Fire
                    break;
                case 3:
                    threshold = 1.3; // Alive
                    break;
                case 4:
                    threshold = 1.6; // Dead
                    break;
                case 5:
                    threshold = 1.4; // Radioactive
                    break;
                case 6:
                    threshold = 1.95; // Smoking
                    break;
                case 7:
                    threshold = 1.6; // Toxic
                    break;
              }   
          return threshold;                               
        }
    };


int main(int argc, char** argv)
    {
      ros::init(argc, argv, "image_subscriber");
      ImageSubscriber ic;
        
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
						            
						            //Template size
						            cv::Size size(235,235);  
						            //Resize image
						            cv::resize(tem,tem,size);
						                  
						            templateMatching(ic.getImage(), tem, sign, ic.getThreshold(i));
						            cv::waitKey(3);
				              }
                  }
                  
             ros::spinOnce();
          }
            
        return 0;
    }
/*


#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "geometry_msgs/Point.h"
#include "std_msgs/String.h"

#include "b39vt_assignment/image_processing.hpp"

class ImageSubscriber
{
    // By default, variables and methods are private
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;
    //const char * image_window = "/usb_cam/image_raw";
//  const char * template_window = "Template window";

  const std::string OPENCV_WINDOW;
  
public:
    // Public variables - it is a good practice to separate the declaration of
    // variables from the declaration/definition of methods
      
  cv_bridge::CvImagePtr cv_ptr;
  
    bool data_valid;
  
public:
    // Public methods
  ImageSubscriber() : it_(nh_), data_valid(false), OPENCV_WINDOW("Image window")
  {
    // Subscribe to input video feed
    image_sub_ = it_.subscribe("/usb_cam/image_raw", 1, 
      &ImageSubscriber::imageCb, this);
  //  image_pub_ = it_.advertise("/image_converter/output_video", 1);

   
    cv::namedWindow(OPENCV_WINDOW);
  }

  ~ImageSubscriber()
  {
    cv::destroyWindow(OPENCV_WINDOW);
  }


  cv::Mat getImage(){
  
  return cv_ptr->image;
  
  }
  
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
    
    // Update GUI Window - uncomment if you want to visualize the subscribed
    // image...
    //cv::imshow(OPENCV_WINDOW, cv_ptr->image);
    //cv::waitKey(3);
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_subscriber");
  ImageSubscriber ic;
  
  //cv::Mat a;
  //cv::Mat b;
  cv::Mat templ; 
        //templ = cv::imread("/home/calum/Pictures/Test_Images/biohazard.png");
        //templ = cv::imread("/home/calum/Pictures/Test_Images/danger.png");
        //templ = cv::imread("/home/calum/Pictures/Test_Images/fire.png");
        //templ = cv::imread("/home/calum/Pictures/Test_Images/alive.png");
        //templ = cv::imread("/home/calum/Pictures/Test_Images/dead.png");
        //templ = cv::imread("/home/calum/Pictures/Test_Images/radioactive.png");
        //templ = cv::imread("/home/calum/Pictures/Test_Images/smoking2.png");
        //templ = cv::imread("/home/calum/Pictures/Test_Images/toxic.png");
  
  while (ros::ok())
  {
        if (ic.data_valid)
        {
            //templateMatching(cv_ptr->image, templ);
            
                
                    cv::Size size(225,225);//template size,e.g.100x100
                    cv::resize(templ,templ,size);//resize image
            
                    //cv::imshow(template_window, templ[i]);

                
                    templateMatching(ic.getImage(), templ);
                    cv::waitKey(100);
                
    
 
        }
    ros::spinOnce();
  }
  return 0;
}

*/


