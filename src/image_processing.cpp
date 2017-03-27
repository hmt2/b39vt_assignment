#include "b39vt_assignment/image_processing.hpp"

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include "geometry_msgs/Point.h"
#include "std_msgs/String.h"

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

bool templateMatching(const cv::Mat& img, const cv::Mat& tem, const char* sign, const float threshold)
{
    const char * image_window = "/camera/rgb/image_raw";
    const char * template_window = "Template window";
    bool matchFound = false;

    // Source image to display
    cv::Mat img_display, result;
    int match_method = CV_TM_SQDIFF;
    img.copyTo( img_display );

    // Create the result matrix
    int result_cols =  img.cols - tem.cols + 1;
    int result_rows = img.rows - tem.rows + 1;

    result.create( result_rows, result_cols, CV_32FC1 );

    // Do the matching
    cv::matchTemplate( img, tem, result, match_method );

    // Localizing the best match with minMaxLoc
    double minVal; double maxVal; cv::Point minLoc; cv::Point maxLoc;
    cv::Point matchLoc;

    cv::minMaxLoc( result, &minVal, &maxVal, &minLoc, &maxLoc, cv::Mat() );

    // Create newVal to compare threshold against
    double newVal = minVal/1000000000;
    printf("newVal %.2f \n     ", newVal);

    // Only enter loop if confident in a match
    if(newVal < threshold)
        {

        if( match_method  == CV_TM_SQDIFF || match_method == CV_TM_SQDIFF_NORMED )
              { 
               matchLoc = minLoc;
              }
           
          else
              { 
               matchLoc = maxLoc; 
              }

          // Create rectangle
          cv::rectangle( img_display, matchLoc, cv::Point( matchLoc.x + tem.cols , matchLoc.y + tem.rows) , cv::Scalar::all(0), 2, 8, 0 );

          // Show the match
          cv::imshow( image_window, img_display );
          
          // Output data  
          printf("Template Match: %s\n", sign);
          printf("    Found at: x= %d, y= %d\n", matchLoc.x, matchLoc.y);
         // printf("threshold %d\n", threshold);
          matchFound = true;
        }

    else
        {
         	cv::imshow( image_window, img );
        }
    

    // Show which template is being matched
    cv::imshow(template_window, tem);

    cv::waitKey(3);
    return matchFound;
}


/*
#include "b39vt_assignment/image_processing.hpp"

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "geometry_msgs/Point.h"
#include "std_msgs/String.h"


  void templateMatching(const cv::Mat& img, const cv::Mat& templ)
{
        const char * image_window = "/usb_cam/image_raw";
        const char * template_window = "Template window";
    //Create confidence threshold
        double threshold = 1.5;
        /// Source image to display
        cv::Mat img_display, result;
        int match_method = CV_TM_SQDIFF;
        img.copyTo( img_display );

        /// Create the result matrix
        int result_cols =  img.cols - templ.cols + 1;
        int result_rows = img.rows - templ.rows + 1;
        //ROS_INFO("Matrix creation...");
        result.create( result_rows, result_cols, CV_32FC1 );

        //ROS_INFO("Matrix created...");
        /// Do the Matching and Normalize
        cv::matchTemplate( img, templ, result, match_method );

        //cv::normalize( result, result, 0, 1, cv::NORM_MINMAX, -1, cv::Mat() );
        

        /// Localizing the best match with minMaxLoc
        double minVal; double maxVal; cv::Point minLoc; cv::Point maxLoc;
        cv::Point matchLoc;

        cv::minMaxLoc( result, &minVal, &maxVal, &minLoc, &maxLoc, cv::Mat() );
        /// For SQDIFF and SQDIFF_NORMED, the best matches are lower values. For all the other methods, the higher the better
        //ROS_INFO(("%d"), minVal) ;

     double newVal = minVal/1000000000;

        printf("newVal %.2f\n",newVal);
        
        if(newVal < threshold)
        {
            if( match_method  == CV_TM_SQDIFF || match_method == CV_TM_SQDIFF_NORMED )
             { 
                matchLoc = minLoc;
             }
             
            else
              { 
                 matchLoc = maxLoc; 
              }
    


            cv::rectangle( img_display, matchLoc, cv::Point( matchLoc.x + templ.cols , matchLoc.y + templ.rows ), cv::Scalar::all(0), 2, 8, 0 );

            cv::rectangle( result, matchLoc, cv::Point( matchLoc.x + templ.cols , matchLoc.y + templ.rows ), cv::Scalar::all(0), 2, 8, 0 );


            cv::imshow( image_window, img );
            
            printf("-----------------------------------------------------%f,%f", matchLoc.x, matchLoc.y);
            
            //imshow( result_window, result );
        }
     
            else{cv::imshow( image_window, img );}
            //ROS_INFO("No match");}
    //  return matchLoc;
        
            // Update GUI Window
    cv::imshow(template_window, templ);
        //cv::imshow(image_window, img);
    cv::waitKey(3);
}
*/
