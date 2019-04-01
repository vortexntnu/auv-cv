'''
Finding the buoys contours for use in the robosub competition in San Diego
'''
// Dependencies
#include <opencv2/opencv.hpp>
#include <iostream>
#include <list>
#include <vector>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "std_msgs/Bool.h"
#include "CameraObjectInfo.h"
#include <sstream>

// Quality of life
using namespace cv;
using namespace std;
using namespace cv_bridge;
using namespace image_transport;
static const std::string OPENCV_WINDOW = "Image window";
static const std::string WINDOW2 = "Image window 2";


// enum
// {
// 	CAMERA_FRONT = 0,
// 	CAMERA_UNDER = 1,
//   SIMULATOR = 2
// };

/***** INPUT SELECTOR *****/
//int src = CAMERA_FRONT;

class ImageConverter
{
  ros::NodeHandle nh_;
  ros::NodeHandle n_;
  ImageTransport it_;
  Subscriber image_sub_;
  ros::Publisher detect_pub_; 

  public:
    ImageConverter(int argc, char** argv)
      : it_(nh_)
    {

      if((argv[1] == NULL))
      {
        std::cout << "Please provide a ROS topic to subscribe. The following are recomended:" << std::endl;
        std::cout << "- /camera/front" << std::endl;
        std::cout << "- /camera/under" << std::endl;
        std::cout << "Subscribing to default topic of simulator: /manta/manta/cameraunder/camera_image" << std::endl;
      }


      // Subscrive to input video feed and publish output video feed
      image_sub_ = it_.subscribe("image", 1, &ImageConverter::imageCb, this);
      detect_pub_ = n_.advertise<pole_detect::CameraObjectInfo>("pole_midpoint",1000);

      cv::namedWindow(OPENCV_WINDOW);
    }
    ~ImageConverter()
    {
      cv::destroyWindow(OPENCV_WINDOW);
    }

'''
    bool compareContourAreas ( std::vector<cv::Point> contour1, std::vector<cv::Point> contour2 ) {
        double i = fabs( contourArea(cv::Mat(contour1)) );
        double j = fabs( contourArea(cv::Mat(contour2)) );
        return ( i < j );
    }

    bool check_rectangle(std::vector<cv::Point> contour) { 
        vector<Point> approx;
        double peri = arcLength(contour,true);
        approxPolyDP(contour, approx, peri * 0.1, true);
        if (approx.length() == 4) {
            return true
        }
        return false

    }
 '''   
    void imageCb(const sensor_msgs::ImageConstPtr& msg)
    {
      cv_bridge::CvImagePtr cv_ptr;  
      try
      {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
      }
      catch(cv_bridge::Exception& e)
      {
        ROS_ERROR("cv_bridge exceptions: %s", e.what());
        return;
      }
      // Variables  
      int counter = 0
      //vector<vector<Point>> buoys
      // Reading video stream, finding contours and sorting from min to max area
      //cvtColor(cv_ptr->image, frame, CV_BGR2HSV);
      //findContours(frame, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
     // sort(contours.begin(), contours.end(), compareContourAreas);
        '''
      // From max area finding the greatest contours that has likeness to a rectangle
      for (int i = contours.size()-1, i>=0, i = i-1) {
            check = check_rectangle(contours(i));
            if (check == true) {
                counter = counter + 1;
                buoys.push_back(contours(i));
                if (counter == 2) {
                    break;
                }
            }  
      }
       ''' 
    }  

	}

	
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "pole_detect");
  ImageConverter ic(argc, argv);
  ros::spin();
  return 0;
}





