//   Tutorial 4
//   Group C
//   Author: Siyuan Liu, Lifan Wu, Philips Jakovleski

#include <ros/ros.h>

// ROS and OpenCV image processing
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>
#include <cv.h>
#include <highgui.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Int32.h>
#include "std_msgs/String.h"
#include <vector>
#include <fstream>
#include <sstream>
#include <string>
#include <iostream>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <stdio.h>
#include <sstream>
#include <iostream>
#include <vector>
#include <cstdlib>
#include <cassert>
#include <cmath>
#include <iostream>
#include <stdio.h>
#include <sstream>
#include <fstream>

// own files
#include "robot_config.h"
#include "nnetwork.cpp"
#include "cmac.cpp"

Net network;
CMAC cmac;

int IND = 0;
double blobCenter[2];
float blobCenter1[2];
std::vector<float> blobCenter2;
using namespace std;
using namespace cv;
using namespace sensor_msgs;
using namespace message_filters;

namespace enc = sensor_msgs::image_encodings;

std::vector<float> Prediction;


// subscribers to tactile and touch sensors
ros::Subscriber tactile_sub;
ros::Subscriber bumper_sub;

// publisher to robot joints for sending the target joint angles
ros::Publisher target_joint_state_pub;

// joint stiffnesses
ros::Publisher stiffness_pub;

// received motor state of the HEAD
double motor_head_in[HEAD_DOF];

// received motor state of the LEFT ARM
double motor_l_arm_in[L_ARM_DOF];

// received motor state of the RIGHT ARM
double motor_r_arm_in[R_ARM_DOF];

double TEST[2];
// class for Data storage
class NN_pair
{
  public:
  std::vector<double> NN_input;
  std::vector<double> NN_output;
};

std::vector<NN_pair> nn_pairs;
std::vector<NN_pair> nn_pairs_norm;

// NN received training data input
std::vector<std::vector<double> > NN_Input;

// NN received training data output
std::vector<std::vector<double> > NN_Output;


// label of the GUI window showing the raw image of NAO's camera
static const char cam_window[] = "NAO Camera (raw image)";
static const char blob_window[] = "NAO Blob (max)";



// set the stiffness
void setStiffness(float value)
{
//    cout << "setting stiffnesses (head) to " << value << endl;

    float v1[2]={0.9,0.9};
    robot_specific_msgs::JointState target_joint_stiffness;

    // set stiffnesses of HEAD joints
    target_joint_stiffness.name.clear();
    target_joint_stiffness.name.push_back("Head");
    target_joint_stiffness.effort.clear();
    for (int i=0; i<HEAD_DOF; i++)
        target_joint_stiffness.effort.push_back(v1[i]);

    stiffness_pub.publish(target_joint_stiffness);

}

void setStiffness_LArm(float value)
{
//    cout << "setting stiffnesses (LArm) to " << value << endl;
    float v2[6]={0.9,0.9,0.9,0.9,0.9,0.9};
    robot_specific_msgs::JointState target_joint_stiffness;

    // set stiffnesses of  joints
    target_joint_stiffness.name.clear();
    target_joint_stiffness.name.push_back("LArm");
    target_joint_stiffness.effort.clear();
    for (int i=0; i<L_ARM_DOF; i++)

        target_joint_stiffness.effort.push_back(v2[i]);

    stiffness_pub.publish(target_joint_stiffness);

}

void setStiffness_RArm(float value)
{
//    cout << "setting stiffnesses (RArm) to " << value << endl;
    float v3[6]={0.9,0.9,0.9,0.9,0.9,0.9};
    robot_specific_msgs::JointState target_joint_stiffness;

    // set stiffnesses of joints
    target_joint_stiffness.name.clear();
    target_joint_stiffness.name.push_back("RArm");
    target_joint_stiffness.effort.clear();
    for (int i=0; i<R_ARM_DOF; i++)
        target_joint_stiffness.effort.push_back(v3[i]);

    stiffness_pub.publish(target_joint_stiffness);

}






// callback function for bumpers
void bumperCB(const robot_specific_msgs::Bumper::ConstPtr& __bumper)
{
    // check each bumper

    cout << "bumper " << (int)__bumper->bumper << endl;

    static bool left_bumper_flag = false;
    static bool right_bumper_flag = false;

    // check left bumper
    if (((int)__bumper->bumper == 1) && ((int)__bumper->state == 1))
    {
        left_bumper_flag = !left_bumper_flag;   // toggle flag

        // do something, e.g.:
        // set / reset stiffness
        if (left_bumper_flag)
            setStiffness(0.005);
        /*
        else
            setStiffness(0.9);  */

    }

    // check right bumper
    if (((int)__bumper->bumper == 0) && ((int)__bumper->state == 1))
    {
        right_bumper_flag = !right_bumper_flag;     // toggle flag



    }

}


// callback function for the head joints
void jointStateCB(const robot_specific_msgs::JointState::ConstPtr& joint_state)
{
    // buffer for incoming message
    std_msgs::Float32MultiArray buffer;

    // index
    int idx;


    // extract the proprioceptive state of the HEAD
    buffer.data.clear();
    for (int i=0; i<ROBOT_DOF; i++)
    {
        if (joint_state->name[i] == "HeadYaw")
        {
            buffer.data.push_back(joint_state->position[i]);
            // cout << joint_state->name[i] << ": " << joint_state->position[i] << endl;
        }
        if (joint_state->name[i] == "HeadPitch")
        {
            buffer.data.push_back(joint_state->position[i]);
            // cout << joint_state->name[i] << ": " << joint_state->position[i] << endl;
        }
    }

    // write data into array
    idx = 0;
    for(vector<float>::const_iterator iter = buffer.data.begin(); iter != buffer.data.end(); ++iter)
    {
        // store into temporary target motor state buffer
        motor_head_in[idx] = *iter;
        idx++;
    }

    // display data on terminal
//    cout << "Head joints:  ";
//    for (int i=0; i<HEAD_DOF; i++)
//        cout << motor_head_in[i] << " ";
//    cout << endl;


    // extract the proprioceptive state of the LEFT ARM
    buffer.data.clear();
    for (int i=0; i<ROBOT_DOF; i++)
    {
        if (joint_state->name[i] == "LShoulderPitch")
        {
            buffer.data.push_back(joint_state->position[i]);
            // cout << joint_state->name[i] << endl;
        }
        if (joint_state->name[i] == "LShoulderRoll")
        {
            buffer.data.push_back(joint_state->position[i]);
            // cout << joint_state->name[i] << endl;
        }
        if (joint_state->name[i] == "LElbowYaw")
        {
            buffer.data.push_back(joint_state->position[i]);
            // cout << joint_state->name[i] << endl;
        }
        if (joint_state->name[i] == "LElbowRoll")
        {
            buffer.data.push_back(joint_state->position[i]);
            // cout << joint_state->name[i] << endl;
        }
        if (joint_state->name[i] == "LWristYaw")
        {
            buffer.data.push_back(joint_state->position[i]);
            // cout << joint_state->name[i] << endl;
        }
        if (joint_state->name[i] == "LHand")
        {
            buffer.data.push_back(joint_state->position[i]);
            // cout << joint_state->name[i] << endl;
        }

    }

    // write data into array
    idx = 0;
    for(vector<float>::const_iterator iter = buffer.data.begin(); iter != buffer.data.end(); ++iter)
    {
        // store into temporary target motor state buffer
        motor_l_arm_in[idx] = *iter;
        idx++;
    }

    // display data on terminal
//    cout << "Left arm joints:  ";
//    for (int i=0; i<L_ARM_DOF; i++)
//        cout << motor_l_arm_in[i] << " ";
//    cout << endl;


    // extract the proprioceptive state of the RIGHT ARM
    buffer.data.clear();
    for (int i=0; i<ROBOT_DOF; i++)
    {
        if (joint_state->name[i] == "RShoulderPitch")
        {
            buffer.data.push_back(joint_state->position[i]);
            // cout << joint_state->name[i] << endl;
        }
        if (joint_state->name[i] == "RShoulderRoll")
        {
            buffer.data.push_back(joint_state->position[i]);
            // cout << joint_state->name[i] << endl;
        }
        if (joint_state->name[i] == "RElbowYaw")
        {
           buffer.data.push_back(joint_state->position[i]);
            // cout << joint_state->name[i] << endl;
        }
        if (joint_state->name[i] == "RElbowRoll")
        {
            buffer.data.push_back(joint_state->position[i]);
            // cout << joint_state->name[i] << endl;
        }
        if (joint_state->name[i] == "RWristYaw")
        {
            buffer.data.push_back(joint_state->position[i]);
            // cout << joint_state->name[i] << endl;
        }
        if (joint_state->name[i] == "RHand")
        {
            buffer.data.push_back(joint_state->position[i]);
            // cout << joint_state->name[i] << endl;
        }

    }

    // write data into array
    idx = 0;
    for(vector<float>::const_iterator iter = buffer.data.begin(); iter != buffer.data.end(); ++iter)
    {
        // store into temporary target motor state buffer
        motor_r_arm_in[idx] = *iter;
        idx++;
    }

    // display data on terminal
//    cout << "Right arm joints:  ";
//    for (int i=0; i<R_ARM_DOF; i++)
//        cout << motor_r_arm_in[i] << " ";
//    cout << endl;

}


// send commanded joint positions of the HEAD
void sendTargetJointStateHead(/* maybe a result as function argument */)
{
  double dummy[HEAD_DOF];  // dummy representing the comanded joint state
    robot_specific_msgs::JointAnglesWithSpeed target_joint_state;

    // specify the limb
    target_joint_state.joint_names.clear();
    target_joint_state.joint_names.push_back("Head");

    // specifiy the angle
    target_joint_state.joint_angles.clear();
    for (int i=0; i<HEAD_DOF; i++)
        target_joint_state.joint_angles.push_back(dummy[i] /* array containing result */);

    // set speed
    target_joint_state.speed = 0.2;

    // set the mode of joint change
    target_joint_state.relative = 0;

    // send to robot
    target_joint_state_pub.publish(target_joint_state);

}


// send commanded joint positions of the LEFT ARM
void sendTargetJointStateLArm(/* maybe a result as function argument */)
{

//    double dummy[L_ARM_DOF]={Prediction[0], Prediction[1], -0.04146, -1.20722, -1.30854, 0.85 };  // dummy representing the comanded joint state
    double dummy[L_ARM_DOF]={Prediction[0], Prediction[1], -0.398882, -0.711734, -1.12446 ,0.848 };

    robot_specific_msgs::JointAnglesWithSpeed target_joint_state;

    // specify the limb
    target_joint_state.joint_names.clear();
    target_joint_state.joint_names.push_back("LArm");

    // specifiy the angle
    target_joint_state.joint_angles.clear();
    for (int i=0; i<L_ARM_DOF; i++)
        target_joint_state.joint_angles.push_back(dummy[i] /* array containing result */);

    // set speed
    target_joint_state.speed = 0.2;

    // set the mode of joint change
    target_joint_state.relative = 0;

    // send to robot
    target_joint_state_pub.publish(target_joint_state);

}
void sendTargetJointStateLArmRepeat(/* maybe a result as function argument */)
{
    double dummy[L_ARM_DOF]={ 0.354312, 0.251534, -1.70432, -1.37902 ,-0.687274, 0.8444  };  // dummy representing the comanded joint state
    robot_specific_msgs::JointAnglesWithSpeed target_joint_state;


    // specify the limb
    target_joint_state.joint_names.clear();
    target_joint_state.joint_names.push_back("LArm");

    // specifiy the angle
    target_joint_state.joint_angles.clear();
    for (int i=0; i<L_ARM_DOF; i++)
        target_joint_state.joint_angles.push_back(dummy[i] /* array containing result */);

    // set speed
    target_joint_state.speed = 0.2;

    // set the mode of joint change
    target_joint_state.relative = 0;

    // send to robot
    target_joint_state_pub.publish(target_joint_state);

}


// send commanded joint positions of the RIGHT ARM
void sendTargetJointStateRArm(/* maybe a result as function argument */)
{
    double dummy[R_ARM_DOF]={0.202446, 0.044528, 0.994074, 1.38516 ,0.705682, 0.8444};  // dummy representing the comanded joint state
    robot_specific_msgs::JointAnglesWithSpeed target_joint_state;


    // specify the limb
    target_joint_state.joint_names.clear();
    target_joint_state.joint_names.push_back("RArm");

    // specifiy the angle
    target_joint_state.joint_angles.clear();
    for (int i=0; i<R_ARM_DOF; i++)
        target_joint_state.joint_angles.push_back(dummy[i] /* array containing result */);

    // set speed
    target_joint_state.speed = 0.2;

    // set the mode of joint change
    target_joint_state.relative = 0;

    // send to robot
    target_joint_state_pub.publish(target_joint_state);

}

// give another position (different angles for each joint)
// we can also use an array as input
void sendTargetJointStateRArmRepeat(/* maybe a result as function argument */)
{
    double dummy[R_ARM_DOF]={0.354312, -0.251534, 1.70432, 1.37902 ,0.687274, 0.8444 };  // dummy representing the comanded joint state
    robot_specific_msgs::JointAnglesWithSpeed target_joint_state;

    // specify the limb
    target_joint_state.joint_names.clear();
    target_joint_state.joint_names.push_back("RArm");

    // specifiy the angle
    target_joint_state.joint_angles.clear();
    for (int i=0; i<R_ARM_DOF; i++)
        target_joint_state.joint_angles.push_back(dummy[i] /* array containing result */);

    // set speed
    target_joint_state.speed = 0.2;

    // set the mode of joint change
    target_joint_state.relative = 0;

    // send to robot
    target_joint_state_pub.publish(target_joint_state);

}

void sendTargetJointStateRArmBoth(/* maybe a result as function argument */)
{
    double dummy[R_ARM_DOF]={0.354312, -0.251534, 1.70432, 1.37902 ,0.687274, 0.8444 };  // dummy representing the comanded joint state
    robot_specific_msgs::JointAnglesWithSpeed target_joint_state;

    // specify the limb
    target_joint_state.joint_names.clear();
    target_joint_state.joint_names.push_back("RArm");

    // specifiy the angle
    target_joint_state.joint_angles.clear();
    for (int i=0; i<R_ARM_DOF; i++)
        target_joint_state.joint_angles.push_back(dummy[i] /* array containing result */);

    // set speed
    target_joint_state.speed = 0.2;

    // set the mode of joint change
    target_joint_state.relative = 0;

    // send to robot
    target_joint_state_pub.publish(target_joint_state);

}

void sendTargetJointStateRArmI(/* maybe a result as function argument */)
{
    double dummy[R_ARM_DOF]={0.934248,-0.0752079,0.54146,1.39291,0.486236,0.8448  };  // dummy representing the comanded joint state
    robot_specific_msgs::JointAnglesWithSpeed target_joint_state;

    // specify the limb
    target_joint_state.joint_names.clear();
    target_joint_state.joint_names.push_back("RArm");

    // specifiy the angle
    target_joint_state.joint_angles.clear();
    for (int i=0; i<R_ARM_DOF; i++)
        target_joint_state.joint_angles.push_back(dummy[i] /* array containing result */);

    // set speed
    target_joint_state.speed = 0.2;

    // set the mode of joint change
    target_joint_state.relative = 0;

    // send to robot
    target_joint_state_pub.publish(target_joint_state);

}

// callback function for vision
void visionCB(const sensor_msgs::ImageConstPtr& msg)
{
    // pointer on OpenCV image
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


    //   show the raw camera image
    imshow(cam_window, cv_ptr->image);
    waitKey(3);
    //  transform to HSV image
    Mat HSVImage;
    Mat ThreshImage;
    Mat ThreshImage_lower;
    Mat ThreshImage_upper;
    cvtColor(cv_ptr->image, HSVImage,CV_BGR2HSV);
    //  give the range of red color
    //    inRange(HSVImage,Scalar(0,100,100),Scalar(10,255,255),ThreshImage);
    // can also do it like this but not necessary, to give an upper and lower bound to the color
    inRange(HSVImage,Scalar(0,100,100),Scalar(10,255,255),ThreshImage_lower);
    inRange(HSVImage,Scalar(160,100,100),Scalar(179,255,255),ThreshImage_upper);
    cv::addWeighted(ThreshImage_lower,1.0,ThreshImage_upper,1.0,0,ThreshImage);
    cv::GaussianBlur(ThreshImage,ThreshImage,Size(9,9),2,2);

    //    cv::GaussianBlur(ThreshImage,ThreshImage,Size(7,7),1.5,1.5);
    // find the contours
    vector<vector<Point> > contours;
    vector<Vec4i> hierarchy;
    threshold(ThreshImage,ThreshImage,0.0,255.0,CV_THRESH_BINARY | CV_THRESH_OTSU);
    findContours(ThreshImage,contours,hierarchy,CV_RETR_TREE,CV_CHAIN_APPROX_SIMPLE,Point(0,0));

    //draw contours
    vector<vector<Point> > contours_poly( contours.size() );
    vector<Rect> boundRect( contours.size() );
    vector<Point2f>center( contours.size() );
    vector<float>radius( contours.size() );

    for(int i = 0; i < contours.size(); i++ )
    { approxPolyDP( Mat(contours[i]), contours_poly[i], 3, true );
      // get rectangle bound
      boundRect[i] = boundingRect( Mat(contours_poly[i]) );
      // get the circle inside the Rect.
      minEnclosingCircle( (Mat)contours_poly[i], center[i], radius[i] );
    }


    Mat drawing = Mat::zeros( ThreshImage.size(),CV_8UC3);

    double area;
    double area_max=0;

    int max_ind;
    // find the maximal blob
    if(contours.size()!=0){

      for(int i=0;i<contours.size();i++)
      {


        // calculate the area of each blob
        area = boundRect[i].area();
        //std::cout<<area<<endl;

        // choose the largest one
        if(area>area_max){
          area_max = area;
          max_ind = i;
        }

      }

      // the color of rect.
      Scalar color = Scalar(255,255,255);
      // only draw the largest one in window
      rectangle( drawing, boundRect[max_ind].tl(), boundRect[max_ind].br(), color, 2, 8, 0 );

      // show the largest blob
      imshow(blob_window,drawing);
      waitKey(3);

//      std::cout<<"Left "<<boundRect[max_ind].tl()<<std::endl;;
//      std::cout<<"Right "<<boundRect[max_ind].br()<<std::endl;

      double xx = boundRect[max_ind].tl().x + ( boundRect[max_ind].br().x - boundRect[max_ind].tl().x )/2;
      double yy = boundRect[max_ind].tl().y + ( boundRect[max_ind].br().y - boundRect[max_ind].tl().y )/2;

      blobCenter[0] = xx;
      blobCenter[1] = yy;

      std::cout<<"Center:  [ "<<xx<<" , "<<yy<<" ]"<<std::endl;


      //  make it to run constant
      blobCenter1[0] = blobCenter[0]/640;
      blobCenter1[1] = blobCenter[1]/480;
      blobCenter2.resize(2);


      blobCenter2[0] =  blobCenter1[0] ;
      blobCenter2[1] =  blobCenter1[1] ;

      Prediction.resize(2);

      cout<<"Center : "<<blobCenter2[0]<<"  , "<<blobCenter2[1]<<endl;

      Prediction = cmac.prediction(blobCenter2);

      cout<<"Prediction : "<<Prediction[0]<<" ,  "<<Prediction[1]<<endl;

      Prediction[0] = Prediction[0]*2*2.0857 - 2.0857;
      Prediction[1] = Prediction[1]*1.6407 - 0.3142;

      cout<<"Prediction norm: "<<Prediction[0]<<" ,  "<<Prediction[1]<<endl;

      sendTargetJointStateLArm();

    }

}


// callback function for tactile buttons (TBs) on the head
void tactileCB(const robot_specific_msgs::TactileTouch::ConstPtr& __tactile_touch)
{
    // check TB 3 (rear)
    if (((int)__tactile_touch->button == 3))// && ((int)__tactile_touch->state == 1))
    {
        cout << "TB " << (int)__tactile_touch->button << " touched" << endl;
        IND = 3;

    }

    // check TB 2 (middle)
    if (((int)__tactile_touch->button == 2))// && ((int)__tactile_touch->state == 1))
    {
        cout << "TB " << (int)__tactile_touch->button << " touched" << endl;
        IND = 2;

    }

    // check TB 1 (front)
    if (((int)__tactile_touch->button == 1))// && ((int)__tactile_touch->state == 1))
    {
        cout << "TB " << (int)__tactile_touch->button << " touched" << endl;
        IND = 1;

    }


}

// callback function for key events
void keyCB(const std_msgs::String::ConstPtr& msg)
{
    ROS_INFO("key pushed: %s", msg->data.c_str());

    // start the robot behaviour
    if (*(msg->data.c_str()) == '0')
	{
    cout << "keyCB()" << endl;
		
    	
	}

}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "central_node");
    ros::NodeHandle central_node_nh;

    // messaging with the NAO nodes

    // advertise joint stiffnesses
    stiffness_pub = central_node_nh.advertise<robot_specific_msgs::JointState>("joint_stiffness", 1);


    // subscribe to the joint states
    // the topic is the same as the one of the wrapper node of the NAO robot
    ros::Subscriber joint_state_sub;
    joint_state_sub = central_node_nh.subscribe("joint_states", 1, &jointStateCB);

    // advertise the target joint states
    target_joint_state_pub = central_node_nh.advertise<robot_specific_msgs::JointAnglesWithSpeed>("joint_angles", 1);    // to NAO robot

    // using image_transport to publish and subscribe to images
    image_transport::ImageTransport image_tran(central_node_nh);

    // subscribe to the raw camera image
    image_transport::Subscriber image_sub;
    image_sub = image_tran.subscribe("/nao_robot/camera/top/camera/image_raw", 1, &visionCB);

    // subscribe to tactile and touch sensors
    tactile_sub = central_node_nh.subscribe("tactile_touch", 1, tactileCB);
    bumper_sub = central_node_nh.subscribe("bumper", 1, bumperCB);




//============================================  CMAC ===============================================


        int trainPair_num = 150; // Number of training pairs
        int resolution = 50;     // Initialize the resolution
        int field_size = 5;      //  Field size 3
        int channel_num = 2;     // Two input channels y1 and y2
        float alpha =0.5;

        // 150 trainging pairs
        double *p = new double[150];
        ifstream dataE("/home/bilhr_b/catkin_ws/src/bilhr/src/TrainingData_150_v2.txt");

        for(int ii=0;ii<150*4;ii++)  {
            dataE>>p[ii];
        }


        double centerX[150];
        double centerY[150];
        double out[150];
        double out2[150];



        for(int ii=0;ii<150;ii++){

            centerX[ii]=p[4*ii]/640;
            centerY[ii]=p[4*ii + 1]/480;
            float temp = p[4*ii + 2];

            out[ii]=( temp + 2.0857 )/(2*2.0857);
            float temp1 = p[4*ii + 3];
            out2[ii]=( temp1 + 0.3142 )/1.6407;

            cout<<temp<<"  "<<temp1<<endl;


            cout<<"INDEX : "<<ii<<endl;
        }


        cout<<"TEST"<<endl;

        cmac.initial(resolution,field_size,channel_num,trainPair_num,alpha,centerX,centerY,out,out2);

        float MSE = 100;
        int n = 0;
        while(MSE>0.00002){

            for(int m=0;m<trainPair_num;m++){

                cmac.activeNeuronPosition(m);

                cmac.update(m);

            }

            MSE = cmac.getError();


            cout<<"MSE in the "<< n << "  th epoch : "<<MSE<<endl;

            n = n+1;

        }



        cout<<"Finish to train the NET !!! "<<endl;


// TEST THE NET


//        std::vector<float> test;
//        test.resize(2);

//        test[0]=183.0/640.0;
//        test[1]=130.0/480.0;
//        std::vector<float> result;

//        result = cmac.prediction(test);
//        cout<<"TEST Input  : "<<test[0]<<"  "<<test[1]<<endl;
//        cout<<"Prediction  : "<<result[0]<<"  "<<result[1]<<endl;
//        cout<<"Prediction  : "<<( result[0]*2*2.0857 - 2.0857  )<<"  "<<( result[1]*1.6407 - 0.3142 )<<endl;
//        cout<<"Target      : -0.0583339 0.516916"<<endl;

//        cout<<"!!!!!!!!!!!!!!!!!!!!!1!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!1"<<endl;





//==================================================== TACTILE  PART !!!=================================================

        // definition of the movements
        int u=1;

        // Store the NN training data pair, only 15 pairs is okay
        NN_Input.resize(2);
        NN_Output.resize(2);

        int numr =0;


        while (ros::ok()){

          // T3
          setStiffness(0.9);
          setStiffness_LArm(0.9);// inside this function, we set the stiffness for shoulder = 0.0
          setStiffness_RArm(0.9);

          if(IND==1){

            nn_pairs.resize(112);//34
            NN_pair& nn_pair(nn_pairs[numr]);

            nn_pair.NN_input.resize(2);
            nn_pair.NN_output.resize(2);

            nn_pair.NN_input[0] = blobCenter[0];
            nn_pair.NN_input[1] = blobCenter[1];
            nn_pair.NN_output[0] = motor_l_arm_in[0];
            nn_pair.NN_output[1] = motor_l_arm_in[1];

            std::cout<<"==================  The NN_INPUT is : "<<nn_pair.NN_input[0]<<", "<<nn_pair.NN_input[1]<<std::endl;
            std::cout<<"==================  The NN_OUTPUT is : "<<nn_pair.NN_output[0]<<", "<<nn_pair.NN_output[1]<<std::endl;
            std::cout<<"==================  The Center is : "<<blobCenter[0]<<", "<<blobCenter[1]<<std::endl;
            std::cout<<"==================  The shoulder position is : "<< motor_l_arm_in[0]<<", "<< motor_l_arm_in[1]<<std::endl;
            numr=numr+1;

            TEST[0]=blobCenter[0];
            TEST[1]=blobCenter[1];

            IND = 3;
            sleep(1);
          }


          if(IND==2){

            nn_pairs_norm.resize(56);//16

            int i=0;
            ofstream inputV("/home/bilhr_b/Desktop/inputV.txt",ios::app);
            ofstream targetV("/home/bilhr_b/Desktop/outputV.txt",ios::app);
            ofstream inputV2("/home/bilhr_b/Desktop/inputV2.txt",ios::app);
            ofstream targetV2("/home/bilhr_b/Desktop/outputV2.txt",ios::app);

            for(int j=0;j<56;j++){//16
              NN_pair& nn_pair_norm(nn_pairs_norm[j]);
              NN_pair& nn_pair(nn_pairs[i]);


                nn_pair_norm.NN_input.resize(2);
                nn_pair_norm.NN_output.resize(2);

                nn_pair_norm.NN_input[0] = nn_pair.NN_input[0]/640;
                nn_pair_norm.NN_input[1] = nn_pair.NN_input[1]/480;

                nn_pair_norm.NN_output[0] = (nn_pair.NN_output[0]+2.0857)/2*2.0857;
                nn_pair_norm.NN_output[1] = (nn_pair.NN_output[1]+0.3142)/1.6407;


                std::cout<<"NN input : "<<nn_pair.NN_input[0]<<"  "<<nn_pair.NN_input[1]<<std::endl;
                std::cout<<"NN output : "<<nn_pair.NN_output[0]<<"  "<<nn_pair.NN_output[1]<<std::endl;
                std::cout<<"NN input NORM : "<<nn_pair_norm.NN_input[0]<<"  "<<nn_pair_norm.NN_input[1]<<std::endl;
                std::cout<<"NN output NORM: "<<nn_pair_norm.NN_output[0]<<"  "<<nn_pair_norm.NN_output[1]<<std::endl;


                inputV.open("/home/bilhr_b/Desktop/inputV.txt",ios::app);
                inputV<<nn_pair_norm.NN_input[0]<<endl;
                inputV.close();

                targetV.open("/home/bilhr_b/Desktop/outputV.txt",ios::app);
                targetV<<nn_pair_norm.NN_output[0]<<endl;
                targetV.close();


                inputV2.open("/home/bilhr_b/Desktop/inputV2.txt",ios::app);
                inputV2<<nn_pair_norm.NN_input[1]<<endl;
                inputV2.close();

                targetV2.open("/home/bilhr_b/Desktop/outputV2.txt",ios::app);
                targetV2<<nn_pair_norm.NN_output[1]<<endl;
                targetV2.close();

              i=i+2;
            }


          }

          if(IND==3){



          }

          ros::spinOnce();

        }





// ================================================================================================================

    // set up the subscriber for the keyboard
    ros::Subscriber key_sub;
    key_sub = central_node_nh.subscribe("key", 5, keyCB);

    // create a GUI window for the raw camera image
    namedWindow(cam_window, 0);
    namedWindow(blob_window, 0);


    ros::spin();

    return 0;
}
