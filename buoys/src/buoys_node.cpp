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
     
    bool compareContourAreas ( std::vector<cv::Point> &contour1, std::vector<cv::Point> &contour2 ) {
        double i = fabs( contourArea(cv::Mat(contour1)) );
        double j = fabs( contourArea(cv::Mat(contour2)) );
        return ( i < j );
    }

    bool check_rectangle(std::vector<cv::Point> contours) { 
        vector<Point> approx;
        Rect2d bbox;
        double ratio;
        double peri = arcLength(contours,true);
        approxPolyDP(contours, approx, peri * 0.1, true);
        if (approx.size() == 4) {
            bbox = boundingRect(contours);
            ratio = (bbox.height / bbox.width);
            if (ratio < 2.2 && ratio > 1.8) {
                return true;
            }
        }
        return false;
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
      // Variables
      int area = 0;
      std::vector<vector<Point>> contours;
      Rect2d bbox, bbox1, bbox2;
      Mat frame, blury, detected_edges;
      int check1, check2;
      double center, center1, center2;
      int write_to_image = 1;
      // Reading video stream, finding contours 
      cvtColor(cv_ptr->image, frame, CV_BGR2HSV);
      GaussianBlur(frame, blury, Size(9,9),0,0);
      Canny(blury, detected_edges, 10, 50, 3);
      findContours(detected_edges, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
      
      // Findng the two biggest rectangluar contours with the desired ratio of 2:1 (height:width)
      for (int i=0; i < contours.size(); i++) {
          if (check_rectangle(contours[i])) {
            bbox = boundingRect(contours[i]);
            if (bbox.area() > area) {
                bbox1 = bbox;
                area = bbox.area();
                check1 = 1;
                cout<<"Are we here1?"<<endl;
            }
          }
      }
      if (check1 == 1) {
        // Send center of boundingbox to topic  
        center1 = (bbox1.tl().x + bbox1.br().x) / 2.0;
        if (write_to_image == 1) {
            rectangle(cv_ptr->image, bbox1.tl(), bbox1.br(), Scalar(255,0,0),5);
        }
        area = 0;
        for (int i=0; i < contours.size(); i++) {
            if (check_rectangle(contours[i])) {
                bbox = boundingRect(contours[i]);
                center = (bbox.tl().x + bbox.br().x) / 2.0;
                if ((center > center1 + 25 || center < center1 - 25) && !bbox1.contains(bbox.tl()) && !bbox1.contains(bbox.br())) {
                    if (bbox.area() > area) {
                        bbox2 = bbox;
                        area = bbox.area();
                        check2 = 1;
                        center2 = center;
                        cout<<"Are we here?"<<endl;
                    }
                }
            }
        }
        if (check2 == 1) {
            // Send center of boundingbox number two to 
            if (write_to_image == 1) {
                rectangle(cv_ptr->image, bbox2.tl(), bbox2.br(), Scalar(255,0,0),5);
            }
        }
      }
      
      cv::imshow(OPENCV_WINDOW, cv_ptr->image);
      cv::waitKey(3);
      
      
    }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "buoys");
  ImageConverter ic(argc, argv);
  ros::spin();
  return 0;
}



