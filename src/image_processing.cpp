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
    bool m = false;

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
    // printf("newVal %.2f \n     ", newVal);

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

        // Output data
        printf("Template Match: %s\n", sign);
        m = true;
    }

    return m;
}
