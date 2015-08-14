//
// Created by daniel on 6/11/15.
//

#include "FaceFinder.h"

FaceFinder::FaceFinder():it(nh) {
    if(!ros::param::get("face_finder/face_cascade", face_cascade_name))
        ROS_ERROR("No haarcascade for face specified");

    if (!ros::param::get("face_finder/image_source", image_source_name))
	ROS_ERROR("No image_source specified");
    image_source = it.subscribe(image_source_name, 1, &FaceFinder::findFaces, this);
    image_output = it.advertise("/face_finder/image_output", 1);
    closest_face = nh.advertise<geometry_msgs::Vector3>("face_finder/closest_face", 1, false);


    if (!face_cascade.load(face_cascade_name))
	ROS_ERROR("Could not load cascade classifier");
    namedWindow("display", WINDOW_AUTOSIZE);
}

FaceFinder::~FaceFinder() { }

void FaceFinder::findFaces(const sensor_msgs::ImageConstPtr &msg) {

    //create a pointer to an opencv image.
    try {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_8UC1);
    }
    catch (cv_bridge::Exception &e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    //cv::Rect *closestFaceRect;
    //cv::Point closestFaceCenter;

    //track faces
    cv::gpu::GpuMat gpuInput(cv_ptr->image), gpuOutput; //these need to be released, not destroyed.
    cv::Mat faceOutput;
    face_cascade.visualizeInPlace = true;
    face_cascade.findLargestObject = true;
    int numberOfFaces = face_cascade.detectMultiScale(gpuInput,
                                                      gpuOutput,
                                                      1.2,
                                                      6,
                                                      cv::Size(cv_ptr->image.cols / 4, cv_ptr->image.rows / 4));
    if (numberOfFaces == 0)
        return;

    //copy information from gpu to cpu
    gpuOutput.colRange(0, numberOfFaces).download(faceOutput);
    /*
    for (int i = 0; i < numberOfFaces; i++) //use if you need multiple faces
        faces.push_back(*(faceOutput.ptr<cv::Rect>()[i]));
    */

    //output image
    cv::Rect *face = (faceOutput.ptr<Rect>()); //use if you need one face
    cv_ptr->image = cv_ptr->image(*face).clone();
    cv_ptr->encoding = sensor_msgs::image_encodings::MONO8;
    //sensor_msgs::Image* outmsg = cv_ptr->toImageMsg();
    //outmsg->encoding = sensor_msgs::image_encodings::MONO8;
    //image_output.publish(outmsg);
    image_output.publish(cv_ptr->toImageMsg());

    //cout << "width:" << face->width << " height:" << face->height << endl;
    cout << cv_ptr->image.cols << endl;
    gpuInput.release(); // free the mats
    gpuOutput.release();


    /*
    //opencv CPU pipeline
    cv::Mat frame_gray;
    cv::cvtColor(cv_ptr->image, frame_gray, CV_BGR2GRAY);
    cv::equalizeHist(frame_gray,frame_gray);
    face_cascade.detectMultiScale( frame_gray, faces, 1.1, 2, 0|CV_HAAR_SCALE_IMAGE, cv::Size(30, 30) );
    for( int i = 0; i < faces.size(); i++ ) {
        cv::Point center(faces[i].x + faces[i].width * 0.5, faces[i].y + faces[i].height * 0.5);
        if(closestFaceRect == NULL || closestFaceRect->width < faces[i].width) {
            closestFaceRect = &faces[i];
            closestFaceCenter.x = center.x;
            closestFaceCenter.y = center.y;
        }
        ellipse(cv_ptr->image, center, Size( faces[i].width*0.5, faces[i].height*0.5), 0, 0, 360, Scalar( 255, 0, 255 ), 2, 8, 0 );
    }
     */

    //calculate vector
    int width = cv_ptr->image.cols; int height = cv_ptr->image.rows;
    geometry_msgs::Vector3 vector;
    double y = face->x + (face->width)/2 - 160;
    double z = -face->y - (face->height)/2 + 120;
    double denom = sqrt(pow(y, 2) + pow(z, 2)); 
    y = -y/denom; //unit vector
    z = -z/denom;
    vector.x = 0;
    vector.y = (abs(y) > 0.8) ? .5*(y - .6 * (y>0?1:-1)) : 0;
    vector.z = (abs(z) > 0.8) ? .5*(z - .6 * (z>0?1:-1)) : 0;
    closest_face.publish(vector);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "face_finder");
    FaceFinder ff;
    ros::spin();
    return 0;
}
