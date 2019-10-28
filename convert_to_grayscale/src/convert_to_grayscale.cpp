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
#include <sstream>

// Quality of life
using namespace cv;
using namespace std;
using namespace cv_bridge;
using namespace image_transport;
static const std::string OPENCV_WINDOW = "Image window";

class ImageConverter
{
  ros::NodeHandle nh_;
  //ros::NodeHandle n_;
  ImageTransport it_;
  Subscriber image_sub_;
  Publisher detect_pub_; 

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
      image_sub_ = it_.subscribe("/camera/raw", 1, &ImageConverter::imageCb, this);
      detect_pub_ = it_.advertise("/camera/mono",1);

      cv::namedWindow(OPENCV_WINDOW);
    }
    ~ImageConverter()
    {
      cv::destroyWindow(OPENCV_WINDOW);
    }
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
      // A simple OPENCV conversion between BGR and Grayscale
      Mat cameraFrameGrey;
      cvtColor(cv_ptr->image, cameraFrameGrey, CV_BGR2GRAY);
      sensor_msgs::ImagePtr image_to_publish = cv_bridge::CvImage(std_msgs::Header(), "mono8", cameraFrameGrey).toImageMsg();
      detect_pub_.publish(image_to_publish);
	}

	
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "convert_to_grayscale");
  ImageConverter ic(argc, argv);
  ros::spin();
  return 0;
}



