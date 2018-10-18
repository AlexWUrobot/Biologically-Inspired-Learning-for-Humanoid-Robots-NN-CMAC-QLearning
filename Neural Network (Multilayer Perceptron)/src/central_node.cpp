/*
 *   Author:      Erhard Wieser
 *   Students:  Wu Li-Fan, Jakovleski Philipp, Fehr Axel
     Description: A node for sending and receiving sensorimotor data from the NAO robot.
 */

//
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

// own files
#include "robot_config.h"
#include <fstream>
#include "neuro_network.h"


using namespace std;
using namespace cv;
using namespace sensor_msgs;
using namespace message_filters;

namespace enc = sensor_msgs::image_encodings;
RNG rng(12345);

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

// label of the GUI window showing the raw image of NAO's camera
static const char cam_window[] = "NAO Camera (raw image)";
static const char hsv_window[] = "(HSV image)";
static const char color_window[] = "(color image)";

int btn_flag;

int pixel_x;
int pixel_y;



// Set stiffness for head and elbow
void setStiffness_shoulder(float value)
{
    cout << "setting stiffnesses of shoulder's pitch and row to " << value << endl;

    robot_specific_msgs::JointState target_joint_stiffness;

    // set stiffnesses of L_ARM joints
    target_joint_stiffness.name.clear();
    target_joint_stiffness.name.push_back("LArm");

    // set the pitch and row of shoulder's stiffness zero
    float stiffnesses[] = {0,0,0.9,0.9,0.9,0.9};
    for (int i=0; i<L_ARM_DOF; i++)
      target_joint_stiffness.effort.push_back(stiffnesses[i]);

    stiffness_pub.publish(target_joint_stiffness);
}

// set the stiffness
void setStiffness(float value)
{

    cout << "setting stiffnesses (L_Arm) to " << value << endl;

    robot_specific_msgs::JointState target_joint_stiffness;

    // set stiffnesses of HEAD joints
    target_joint_stiffness.name.clear();
    target_joint_stiffness.name.push_back("LArm");

    for (int i=0; i<L_ARM_DOF; i++)
        target_joint_stiffness.effort.push_back(value);

    stiffness_pub.publish(target_joint_stiffness);

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

    // show the raw camera image
    //imshow(cam_window, cv_ptr->image);
    Mat HSV_image;
    cvtColor(cv_ptr->image,HSV_image,CV_BGR2HSV);
    Mat low_red_image;
    Mat up_red_image;
    Mat red_image;
    //

    // defining range thresholds in the HSV color space
    cv::inRange(HSV_image,Scalar(0,100,100),Scalar(10,255,255),low_red_image);
    cv::inRange(HSV_image,Scalar(160,100,100),Scalar(179,255,255),up_red_image);

    // combines the images
    cv::addWeighted(low_red_image, 1.0, up_red_image, 1.0, 0.0, red_image);
    imshow(hsv_window,red_image);
    cv::GaussianBlur(red_image, red_image, cv::Size(9,9), 2, 2); // smoothing contours

    //Find Contours
    vector<vector<Point> > contours;
    vector<Vec4i> hierarchy;

    findContours(red_image, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0,0) );
    //cout << "Col x  " << red_image.cols <<  "Row Y " << red_image.rows  <<endl;  // 640 x 480

    //DrawContours
    vector<vector<Point> > contours_poly( contours.size() );
    vector<Rect> boundRect( contours.size() );
    vector<Point2f>center( contours.size() );

    int maxarea = 0;
    int max_area_index;
    int cent_contr_x;
    int cent_contr_y;

    // computing bounding boxes around color blobs
    for (int i = 0; i < contours.size(); i++){
      approxPolyDP( Mat(contours[i]),contours_poly[i],3, true);
      boundRect[i] = boundingRect( Mat(contours_poly[i]));
    }
    // Preventing error when there is no red in the image
    if(contours.size() != 0){
      Mat drawing = Mat::zeros( red_image.size(), CV_8UC3);

      for( int i = 0; i < contours.size(); i++ ){
        // drawing bounding box just around biggest color blob
        if( i == 0 ){
          maxarea = boundRect[i].area();
        } else if(boundRect[i].area() > maxarea)
        {
          maxarea = boundRect[i].area();
          max_area_index = i;
        }
      }

      // sets the color of the bounding box and draws it
      Scalar color = Scalar ( 255, 255, 255);
      rectangle( drawing, boundRect[max_area_index].tl(), boundRect[max_area_index].br(), color, 2, 8, 0);

      // computing center of color blob in pixel coordinates
      cent_contr_x = boundRect[max_area_index].x + boundRect[max_area_index].width/2 ;
      cent_contr_y = boundRect[max_area_index].y + boundRect[max_area_index].height/2 ;

      // Output the position of biggest blob
      cv::circle(drawing, cv::Point(cent_contr_x, cent_contr_y), 5, CV_RGB(255,0,0));
      cout << " X: " << cent_contr_x << " Y: " << cent_contr_y << " width: " << boundRect[max_area_index].width << "  height: "<< boundRect[max_area_index].height <<endl;
      pixel_x = cent_contr_x;
      pixel_y = cent_contr_y;
      imshow(color_window,drawing);
    }

    imshow(cam_window,cv_ptr->image);
    waitKey(3);
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
    cout << "Head joints:  ";
    for (int i=0; i<HEAD_DOF; i++)
        cout << motor_head_in[i] << " ";
    cout << endl;


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
    cout << "Left arm joints:  ";
    for (int i=0; i<L_ARM_DOF; i++)
        cout << motor_l_arm_in[i] << " ";
    cout << endl;


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
    cout << "Right arm joints:  ";
    for (int i=0; i<R_ARM_DOF; i++)
        cout << motor_r_arm_in[i] << " ";
    cout << endl;

}


// send commanded joint positions of the HEAD
void sendTargetJointStateHead(double dummy[HEAD_DOF])
{
    //double dummy[HEAD_DOF];  // dummy representing the comanded joint state
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


// send command joint positions of the LEFT ARM
// input: joint position
void sendTargetJointStateLArm(double dummy[L_ARM_DOF])
{
    robot_specific_msgs::JointAnglesWithSpeed target_joint_state;

    // specify the limb
    target_joint_state.joint_names.clear();
    target_joint_state.joint_names.push_back("LArm");

    // specifiy the angle
    target_joint_state.joint_angles.clear();

    for (int i=0; i<L_ARM_DOF; i++)
        target_joint_state.joint_angles.push_back(dummy[i]);

    // set speed
    target_joint_state.speed = 0.2;  // 0.2

    // set the mode of joint change
    target_joint_state.relative = 0;

    // send to robot
    target_joint_state_pub.publish(target_joint_state);

}

// send commanded joint positions of the RIGHT ARM
void sendTargetJointStateRArm(double dummy[R_ARM_DOF])
{
    //double dummy[R_ARM_DOF];  // dummy representing the comanded joint state
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




// callback function for tactile buttons (TBs) on the head
void tactileCB(const robot_specific_msgs::TactileTouch::ConstPtr& __tactile_touch)
{

    // check TB 3 (rear)
    if (((int)__tactile_touch->button == 3) && ((int)__tactile_touch->state == 1))
    {
        cout << "TB " << (int)__tactile_touch->button << " touched" << endl;
        btn_flag = 3;
    }

    // check TB 2 (middle)
    if (((int)__tactile_touch->button == 2) && ((int)__tactile_touch->state == 1))
    {
        cout << "TB " << (int)__tactile_touch->button << " touched" << endl;
        btn_flag = 2;    // Start Tracking
    }

    // check TB 1 (front)
    if (((int)__tactile_touch->button == 1) && ((int)__tactile_touch->state == 1))
    {
        cout << "TB " << (int)__tactile_touch->button << " touched" << endl;
        btn_flag = 1;  // Catching Point
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


//int main(int argc, char** argv)
int main(int argc, char** argv)
{
    ros::init(argc, argv, "central_node");
    ros::NodeHandle central_node_nh;

    // specifying home position and wave position of both arms
    double L_WavePosition[6] = {0.224006, 0.20398, -1.79329, -1.43578, 0.849794, 0.6468};
    double L_HomePosition[6] = {0.224006, 0.37272, -0.069072, -1.37596, -1.4374, 0.6468};
    double R_WavePosition[6] = {0.224006, -0.20398, 1.79329, 1.43578, -0.849794, 0.6468};
    double R_HomePosition[6] = {0.224006, -0.37272, 0.069072, 1.37596, 1.4374, 0.6468};

    double R_interval[6], L_interval[6], R_goal[6], L_goal[6];
    int interval_range = 20; // movement is split into submovements  do not be smaller than 5
    int half_motion = interval_range/2;

    // arrays to store the joint positions after every submovement
    double joint_state_r[6][interval_range];
    double joint_state_l[6][interval_range];

    int interval_count = 0;
    int last_flag = 0;

    // compute change of joint states between each submovement
    for(int i = 0; i < 6; i++){
      R_interval[i] =  R_WavePosition[i] - R_HomePosition[i] ;
      L_interval[i] =  L_WavePosition[i] - L_HomePosition[i];
      R_interval[i] = R_interval[i] / interval_range;
      L_interval[i] = L_interval[i] / interval_range;  
    }

    //====================================== Interval
    for(int i = 0; i < 6; i++ ){
      for(int k = 0; k < interval_range; k++){
        joint_state_r[i][k] = R_HomePosition[i] + k * R_interval[i];
        joint_state_l[i][k] = L_HomePosition[i] + k * L_interval[i];
      }
    }

    int direction = 1;
    btn_flag = 0;
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
    image_sub = image_tran.subscribe("nao_robot/camera/top/camera/image_raw", 1, &visionCB);

    // subscribe to tactile and touch sensors
    tactile_sub = central_node_nh.subscribe("tactile_touch", 1, tactileCB);
    bumper_sub = central_node_nh.subscribe("bumper", 1, bumperCB);

    // set up the subscriber for the keyboard
    ros::Subscriber key_sub;
    key_sub = central_node_nh.subscribe("key", 5, keyCB);

    // create a GUI window for the raw camera image
    namedWindow(cam_window, 0);


    double Head_detect_pos[2] = {0.374254, -0.262356};
    double L_detect_pos[6] = {-0.069072, 0.351244, -1.12446, -0.389594, -0.61671, 0.6512};
    double L_now[6] = {-0.069072, 0.351244, -1.12446, -0.389594, -0.61671, 0.6512};
    int save_data_flag = 0;

    ros::Rate loop_rate(10);     // publish with 10 Hz

    // Open the File containing the training Data
    ofstream myfile;
    myfile.open ("TrainingData", ios::out);


    NN nn;
    nn.load_data();             // Training and save the weight and bias
    nn.Load_weight_bias();      // Load the weight and bias

//    Testing XOR file
//    double ans[1];
//    nn.Calculate_pos(1,0,ans);
//    cout << "1 and 0 ans: " << ans[0] << endl;
//    nn.Calculate_pos(0,1,ans);
//    cout << "0 and 1 ans: " << ans[0] << endl;
//    nn.Calculate_pos(1,1,ans);
//    cout << "1 and 1 ans: " << ans[0] << endl;
//    nn.Calculate_pos(0,0,ans);
//    cout << "0 and 0 ans: " << ans[0] << endl;
//    sleep(10);

    while (ros::ok()){

      switch(btn_flag){

      // if no button is pressed, nothing happens
      case 0:
        cout<<"======= 0 ";
        break;

      // if the front button is pressed, the robot start to save the training data
      case 1:

        if ( save_data_flag < 10 ){
          setStiffness_shoulder(0);
          cout << "Ready to save the train data, now just keep pressing front button" << endl;
          save_data_flag++;
          //sleep(5);
        }
        else{
          cout<<"======= 1 ";
          if (myfile.is_open()) {
            myfile << pixel_x <<" "<< pixel_y <<" "<< motor_l_arm_in[0] <<" "<< motor_l_arm_in[1] << endl;
          }else{
            cout<<"====================ERROR==========================";
          }
          btn_flag = 0; // change the state from 1 to 0 after saving data
          sleep(5);
        }
        break;

      // if the button in the middle is pressed, the robot start to record the value
      case 2:
        cout<<"======= 2 ";
        sendTargetJointStateHead(Head_detect_pos);
        sendTargetJointStateLArm(L_detect_pos);
        break;

      // if the button in the back is pressed, the robot start to move and follow the blob
      case 3:
        cout<<"======= 3 ";
        nn.Calculate_pos( pixel_x, pixel_y, L_now);
        for (int i = 0; i < 6; i++){
          cout << L_now [i] << "  ";
        }
        cout << endl;
        sendTargetJointStateLArm(L_now);
        break;
      }
      ros::spinOnce();
      loop_rate.sleep();
    }
    myfile.close();
    return 0;
}
