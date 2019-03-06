//
//  main.cpp
//  CV
//
//  Created by Thomas Hellum on 22/10/2018.
//  Copyright © 2018 Thomas Hellum. All rights reserved.
//

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <opencv2/opencv.hpp>
#include <opencv2/tracking.hpp>
#include <opencv2/core/ocl.hpp>


#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include <algorithm> 


using namespace cv;
using namespace std;

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sstream> // for converting the command line parameter to integer



int main(int argc, char** argv)
{
    ros::init(argc, argv, "image_publisher");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    image_transport::Publisher pub = it.advertise("/camera/front", 1);

    // Convert the passed as command line parameter index for the video device to an integer
    int video_source = -1;
    
    // Check if it is indeed a number
    if(!(argv[1] == NULL)){
        std::istringstream video_sourceCmd(argv[1]);
        video_sourceCmd >> video_source;
    }

    cv::VideoCapture cap;
    string name = std::getenv("USER");
    switch(video_source){
        case 0:
        case 1:
        case 2:
            cap.open(video_source);
            break;
        case 3:
            cap.open("/home/"+name+"/Videos/GOPR1142.mp4");
            break;
        case 4:
            cap.open("/home/"+name+"/Videos/guide_posts.mp4");
            break;
        default:
            if(argv[1] == NULL){
                std::cout << "Please provide a video source" << std::endl;
                std::cout << "0: video0" << std::endl;
                std::cout << "1: video1" << std::endl;
                std::cout << "2: video2" << std::endl;
                std::cout << "3: GOPR1142.mp4 @ ~/Videos" << std::endl;
                std::cout << "4: guide_posts.mp4 @ ~/Videos" << std::endl;
                std::cout << "default: specify path to source \n" << std::endl;

                std::cout << "Opening default camera at computer" << std::endl;
                cap.open(-1);
              return 1;
            }
            cap.open(argv[1]);
            break;
    }
    // Check if video device can be opened with the given index
    if(!cap.isOpened()){
        std::cout << "Could not open cap" << std::endl;
        return 1;
    }
    cv::Mat frame;
    sensor_msgs::ImagePtr msg;

    ros::Rate loop_rate(30);
    while (nh.ok()) {
        cap >> frame;
        // Check if grabbed frame is actually full with some content
        if(!frame.empty()) {
            msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
            pub.publish(msg);
            cv::waitKey(1);
        }

        ros::spinOnce();
        loop_rate.sleep();
    }
}