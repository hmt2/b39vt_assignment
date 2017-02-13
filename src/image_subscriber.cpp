#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "geometry_msgs/Point.h"
#include "std_msgs/String.h"


#include <string>

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


  cv::Mat getImage(){ //to access cv_ptr in main method
  
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
  
        
    std::vector< std::pair<cv::Mat, std::string> > templ;
            
          templ.push_back(std::pair<cv::Mat, std::string>(cv::imread("/home/calum/Pictures/Test_Images/biohazard.png"),"biohazard"));
          templ.push_back(std::pair<cv::Mat, std::string>(cv::imread("/home/calum/Pictures/Test_Images/danger.png"),"danger"));
          templ.push_back(std::pair<cv::Mat, std::string>(cv::imread("/home/calum/Pictures/Test_Images/fire.png"),"fire"));
          templ.push_back(std::pair<cv::Mat, std::string>(cv::imread("/home/calum/Pictures/Test_Images/greeen_helmet.png"),"green_helmet"));
          templ.push_back(std::pair<cv::Mat, std::string>(cv::imread("/home/calum/Pictures/Test_Images/red_helmet.png"),"red_helmet"));
          templ.push_back(std::pair<cv::Mat, std::string>(cv::imread("/home/calum/Pictures/Test_Images/radioactive.png"),"radioactive"));
          templ.push_back(std::pair<cv::Mat, std::string>(cv::imread("/home/calum/Pictures/Test_Images/smoking.png"),"smoking"));
          templ.push_back(std::pair<cv::Mat, std::string>(cv::imread("/home/calum/Pictures/Test_Images/toxic.png"),"toxic"));
          
          
    
    /*tempMap[cv::imread("/home/calum/Pictures/Test_Images/biohazard.png")] = "biohazard";
    tempMap[cv::imread("/home/calum/Pictures/Test_Images/danger.png")] = "danger";
    tempMap[cv::imread("/home/calum/Pictures/Test_Images/fire.png")] = "fire";
    tempMap[cv::imread("/home/calum/Pictures/Test_Images/green_helmet.png")] = "green_helmet";
    tempMap[cv::imread("/home/calum/Pictures/Test_Images/red_helmet.png")] = "red_helmet";
    tempMap[cv::imread("/home/calum/Pictures/Test_Images/radioactive.png")] = "radioactive";
    tempMap[cv::imread("/home/calum/Pictures/Test_Images/smoking.png")] = "smoking";
    tempMap[cv::imread("/home/calum/Pictures/Test_Images/toxic.png")] = "toxic";
    
    
  //array of templates
        cv::Mat templ[8]; 
        
        templ[0] = cv::imread("/home/calum/Pictures/Test_Images/biohazard.png");
        templ[1] = cv::imread("/home/calum/Pictures/Test_Images/danger.png");
        templ[2] = cv::imread("/home/calum/Pictures/Test_Images/fire.png");
        templ[3] = cv::imread("/home/calum/Pictures/Test_Images/green_helmet.png");
        templ[4] = cv::imread("/home/calum/Pictures/Test_Images/red_helmet.png");
        templ[5] = cv::imread("/home/calum/Pictures/Test_Images/radioactive.png");
        templ[6] = cv::imread("/home/calum/Pictures/Test_Images/smoking.png");
        templ[7] = cv::imread("/home/calum/Pictures/Test_Images/toxic.png");
  */
  
  while (ros::ok())
  {
        if (ic.data_valid)
        {
            //templateMatching(cv_ptr->image, templ);
            for(int i = 0;i < 7; i++) //change condition to .size()
                {
                    cv::Mat tem = templ[i].first;
                            std::string sign = templ[i].second;
                    
                    printf("%d,%d",tem.cols, tem.rows);
                    
                    cv::Size size(235,235);//template size,e.g.100x100
                    cv::resize(tem,tem,size);//resize image
                            
                            
                    //cv::imshow(template_window, templ[i]);

                
                    templateMatching(ic.getImage(), tem, sign);
                    cv::waitKey(3);
                }
    
 
        }
    ros::spinOnce();
  }
  return 0;
}

