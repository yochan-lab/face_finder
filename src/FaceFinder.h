//
// Created by daniel on 6/11/15.
//

#ifndef FACE_RECOGNITION_FACEFINDER_H
#define FACE_RECOGNITION_FACEFINDER_H

#include <iostream>

#include <ros/ros.h>
#include <geometry_msgs/Vector3.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/objdetect/objdetect.hpp>

#include <opencv2/gpu/gpu.hpp>
#include <opencv2/gpu/gpu.hpp>
#include <opencv2/gpu/gpumat.hpp>

using namespace cv;
using namespace std;

class FaceFinder {
    ros::NodeHandle nh;
    image_transport::ImageTransport it;
    image_transport::Subscriber image_source;
    image_transport::Publisher image_output;
    ros::Publisher closest_face;

    cv_bridge::CvImagePtr cv_ptr;

    std::string face_cascade_name;
    std::string eyes_cascade_name;
    //cv::CascadeClassifier face_cascade;
    //cv::CascadeClassifier eyes_cascade;

    cv::gpu::CascadeClassifier_GPU face_cascade;
public:
    FaceFinder();
    ~FaceFinder();
    void findFaces(const sensor_msgs::ImageConstPtr&msg);
};

#endif //FACE_RECOGNITION_FACEFINDER_H
