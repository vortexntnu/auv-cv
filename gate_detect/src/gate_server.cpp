#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <actionlib/server/simple_action_server.h>
#include <gate_detect/GateCVAction.h>

#include <iostream>
#include <list>
#include <vector>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "std_msgs/Bool.h"
#include "CameraObjectInfo.h"
#include <sstream>

//Quality of life
using namespace cv;
using namespace std;
using namespace cv_bridge;
using namespace image_transport;
static const std::string OPENCV_WINDOW = "Image window";
static const std::string WINDOW2 = "Image window 2";

class GateCVAction
{
public:

  GateCVAction(std::string name) :
    as_(nh_, name, false),
    action_name_(name),
    it_(nh_)
  {
    //register the goal and feeback callbacks
    as_.registerGoalCallback(boost::bind(&GateCVAction::goalCB, this));
    as_.registerPreemptCallback(boost::bind(&GateCVAction::preemptCB, this));

    //subscribe to the data topic of interest
    sub_ = nh_.subscribe("/manta/manta/camerafront/camera_image", 1, &GateCVAction::imageCB, this);
    detect_pub_ = nh_.advertise<pole_detect::CameraObjectInfo>("/camera_object_info",1000);

    //cv::namedWindow(OPENCV_WINDOW);

    as_.start();
  }

  ~GateCVAction(void)
  {
    //cv::destroyWindow(OPENCV_WINDOW);
  }

  void goalCB()
  {
    // reset helper variables

    // accept the new goal
    goal_ = as_.acceptNewGoal()->samples;
  }

  void preemptCB()
  {
    ROS_INFO("%s: Preempted", action_name_.c_str());
    // set the action state to preempted
    as_.setPreempted();
  }

  void imageCB(const sensor_msgs::ImageConstPtr& msg)
  {
      if (!as_.isActive())
          {
              return;
          }

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

        // Declaring variables
        Mat cameraFrame, cameraFrameGrey, detected_edges, blury, red1, red2, red3;
        double x1, x2, y1, y2,x11,x22,y11,y22;
        Rect2d bbox;
        Rect2d bbox_big;
        std::vector<Rect2d> act_bbox;
        pole_detect::CameraObjectInfo detected_2;
        // Setting publishing variables to default values
        detected_2.frame_height = cv_ptr->image.rows;//bbox.height;
        detected_2.frame_width = cv_ptr->image.cols;//bbox.width;
        detected_2.confidence = 0;
        detected_2.pos_x = -1;
        detected_2.pos_y = -1;


        // Reading video stream and putting on a red mask
        cvtColor(cv_ptr->image, cameraFrameGrey, CV_BGR2HSV);
        inRange(cameraFrameGrey, Scalar(0,120,120), Scalar(20,255,255), red1);
        inRange(cameraFrameGrey, Scalar(160,120,120), Scalar(180,255,255), red2);
        addWeighted(red1, 1.0, red2, 1.0, 0.0, red3);

        // Finding contours
        GaussianBlur(red3, blury, Size(9,9),0,0);
        Canny(blury, detected_edges, 10, 50, 3);
        std::vector<std::vector<Point> > contours;
        std::vector<Rect2d> heights;
        std::vector<Rect2d> heights2;
        findContours(detected_edges, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
        int height = 0;

        // Filtering countours based on height into two vectors heights and heights2
        for (int i = 0; i < contours.size(); i++) {
          bbox = boundingRect(contours[i]);
          if ( bbox.height > height ) {
            height = bbox.height;
            heights.push_back(bbox);
          }
        }

        if (heights.size() > 0) {
          bbox_big = heights.end()[-1];
          int center = (bbox_big.tl().x + bbox_big.br().x)/2;
          height = 0;

          for (int i = 0; i < contours.size(); i++) {
            bbox = boundingRect(contours[i]);
            int center2 = (bbox.tl().x + bbox.br().x)/2;
            if ((center2 > center + 25 || center2 < center - 25) && !bbox_big.contains(bbox.tl()) && !bbox_big.contains(bbox.br())) {
              if (bbox.height > height) {
                heights2.push_back(bbox);
              }
            }
          }
        }

        // If only one pole are detected, it returns the coordinates of the pole
        if (heights.size() > 0 && heights2.size() == 0) {
          detected_2.confidence = 0.5;
          bbox = heights.end()[-1];
          detected_2.pos_x = (bbox.tl().x + bbox.br().x) / 2;
          detected_2.pos_y = (bbox.tl().y + bbox.br().y) / 2;
          rectangle(cv_ptr->image, bbox.tl(), bbox.br(), Scalar(0,255,0),5);
        }

        // If two different poles are detected, assumes it is a gate and returns the middle position between them
        if (heights.size() > 0 && heights2.size() > 0) {
          bbox = heights.end()[-1];
          rectangle(cv_ptr->image, bbox.tl(), bbox.br(), Scalar(255,0,0),5);
          x1 = bbox.tl().x;
          y1 = bbox.tl().y;
          x2 = bbox.br().x;
          y2 = bbox.br().y;
          bbox = heights2.end()[-1];
          x11 = bbox.tl().x;
          y11 = bbox.tl().y;
          x22 = bbox.br().x;
          y22 = bbox.br().y;
          rectangle(cv_ptr->image, bbox.tl(), bbox.br(), Scalar(255,0,0),5);
          detected_2.confidence = 1;
          detected_2.pos_x = (x11+x22)/2 + (((x1+x2)/2 - (x11+x22)/2)/2);
          detected_2.pos_y = ((y11+y22)/2 + (((y1+y2)/2 - (y11-y22)/2)/2))+50;
          detected_2.confidence = 1;
        }

        // Displaying imshow for testing
        // cv::imshow(OPENCV_WINDOW, red3);
        // cv::imshow(WINDOW2, cv_ptr->image);
        // cv::waitKey(3);

        // Publish message to /Camera_Object_Info
        detect_pub_.publish(detected_2);
  }

protected:

  ros::NodeHandle nh_;
  actionlib::SimpleActionServer<gate_detect::GateCVAction> as_;
  std::string action_name_;
  int  goal_;
  gate_detect::GateCVFeedback feedback_;
  gate_detect::GateCVResult result_;
  ros::Subscriber sub_;
  ros::Publisher detect_pub_;
  ImageTransport it_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "gate_detect");

  GateCVAction gateDetection(ros::this_node::getName());
  ros::spin();

  return 0;
}