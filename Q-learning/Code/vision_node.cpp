/*
    Author:         LiFan Wu, Siyuan Liu, Chao Dong
    Description:    This node serves as an interface node, it publishes input from the keyboard.
*/

#include "ros/ros.h"
#include "std_msgs/String.h"

#include <stdio.h>
#include <sstream>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/String.h>

#include <aruco/aruco.h>
#include <aruco/cvdrawingutils.h>



#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>
#include <cv.h>
#include <highgui.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>



using namespace cv;
using namespace aruco;

using namespace sensor_msgs;
namespace enc = sensor_msgs::image_encodings;
RNG rng(12345);


class QR
{
  ros::NodeHandle nh_;
  ros::Publisher obstacle_x;
  // using image_transport to publish and subscribe to images
  image_transport::ImageTransport it_;  // ImageTransport API
  image_transport::Subscriber image_sub;
  CameraParameters CameraFactors;// aruco function  in class will automatically take the namespace


public:
  QR()
    : it_(nh_)
  {
    // subscribe to the camera
    image_sub = it_.subscribe("nao_robot/camera/top/camera/image_raw", 1, &QR::vision_CB, this); // in class need "this"
    obstacle_x = nh_.advertise< std_msgs::Float64MultiArray >("/obstacle_x",1000); // publish float array




    // camera factors
    cv::Mat cameraMatrix(3,3,CV_32FC1);   // CV_32FC3
//    cameraMatrix.at<float>(0,0) = 551.589721679688;
//    cameraMatrix.at<float>(0,1) = 0.0;
//    cameraMatrix.at<float>(0,2) = 308.271132841983; //

//    cameraMatrix.at<float>(1,0) = 0.0;
//    cameraMatrix.at<float>(1,1) = 0.0;
//    cameraMatrix.at<float>(1,2) = 550.291320800781;

//    cameraMatrix.at<float>(2,0) = 229.20143668168;
//    cameraMatrix.at<float>(2,1) = 0.0;
//    cameraMatrix.at<float>(2,2) = 0.0;

    cameraMatrix.at<float>(0,0) = 551.543059;
    cameraMatrix.at<float>(0,1) = 0.0;
    cameraMatrix.at<float>(0,2) = 327.382898; //

    cameraMatrix.at<float>(1,0) = 0.0;
    cameraMatrix.at<float>(1,1) = 553.736023;
    cameraMatrix.at<float>(1,2) = 225.026380;

    cameraMatrix.at<float>(2,0) = 0.000000;
    cameraMatrix.at<float>(2,1) = 0.0;
    cameraMatrix.at<float>(2,2) = 1.000000;

    // https://docs.opencv.org/2.4/doc/tutorials/core/mat_the_basic_image_container/mat_the_basic_image_container.html


    cv::Mat distorsionCoeff(1,5,CV_32FC1);
//    distorsionCoeff.at<float>(0,0) = -0.0545211535376379;
//    distorsionCoeff.at<float>(0,1) = 0.0691973423510287;
//    distorsionCoeff.at<float>(0,2) = -0.00241094929163055;
//    distorsionCoeff.at<float>(0,3) = -0.00112245009306511;
//    distorsionCoeff.at<float>(0,4) = 0.0;


    distorsionCoeff.at<float>(0,0) = -0.066494;
    distorsionCoeff.at<float>(0,1) = 0.095481;
    distorsionCoeff.at<float>(0,2) = -0.000279;
    distorsionCoeff.at<float>(0,3) = 0.002292;
    distorsionCoeff.at<float>(0,4) = 0.0;

    CameraFactors.setParams(cameraMatrix,distorsionCoeff,Size(640,480));
    CameraFactors.resize(Size(640,480));  //
    //CameraFactors.readFromXMLFile(argv[2]);
  }
  ~QR()
  {
  }

  // define can not be put in the main, Marker belong to the aruco namespace
  vector<Marker> deterQR(Mat img){
    MarkerDetector qr_detector; // for output data type
    vector<Marker> qr_output;// detect the qr code in the input image
    qr_detector.detect(img, qr_output, CameraFactors);
    return qr_output;
  }



  void vision_CB(const sensor_msgs::ImageConstPtr& msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      // transform ROS image into OpenCV image
      cv_ptr = cv_bridge::toCvCopy(msg, enc::BGR8);
    }
    catch (cv_bridge::Exception& e)		// throw an error msg. if conversion fails
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
    Mat Input_image = cv_ptr->image;
    std_msgs::Float64MultiArray pub_msg;

    vector<Marker> qr_output = deterQR(Input_image);

    for( int i = 0; i < qr_output.size(); i++ )
    {
      Marker qr_get = qr_output[i];
      qr_get.draw(Input_image,Scalar(0,0,255),2);

      qr_get.calculateExtrinsics(0.1,CameraFactors,true);

      if (qr_output.size() >= 4){ // wait for detecting three qr code  means whole goal
        cout << i <<" The detected QR code pos:" << qr_get.id << " x: " << qr_get.Tvec.at<float>(0)<<" y: "<<qr_get.Tvec.at<float>(1)<<" z: "<<qr_get.Tvec.at<float>(2)<<endl;

        pub_msg.data.resize(4);

        // get the goal keeper position, show by x coordinate
        // Make the msg, i.e. determine the value for each element inside the array
        if(qr_get.id == 18){    // left
          pub_msg.data[0]=qr_get.Tvec.at<float>(0);
        }
        if(qr_get.id == 11){    // Right
          pub_msg.data[1]=qr_get.Tvec.at<float>(0);
        }
        if(qr_get.id == 3){     // Up
          pub_msg.data[2]=qr_get.Tvec.at<float>(0);
        }
        if(qr_get.id == 24){    // Goal
          pub_msg.data[3]=qr_get.Tvec.at<float>(0);
        }

        if(i==qr_output.size()-1){  //  i = the last one, 0 1 2 3 make sure save all factor ==3 need to vision first
          obstacle_x.publish(pub_msg);
        }

      }

    }

    imshow("qrcode",Input_image);
    waitKey(10);
  }

};






int main(int argc, char **argv)
{
  cout << "Ros run the vision node now !!" << endl; // cout finishing

  ros::init(argc, argv, "vision_node");
  QR qr;



  ros::spin();
  return 0;
}



