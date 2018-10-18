/*
 *   Author:      Erhard Wieser
 *   Students:  Jakovleski Philipp, Lifan Wu, Siyuan Liu, Hajer Chebil, Chao Dong
 *
 *   JOB distribution:
 *   1. Leg Motion : Jakovleski Philipp and Hajer Chebil
 *   2. Vision :     LiFan Wu, Siyuan Liu, Chao Dong
 *   3. QL:          All group members do it together and modify the code
 *
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


//#include <opencv2/aruco.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Float64MultiArray.h>


#include <std_msgs/Int32.h>
#include "std_msgs/String.h"
// own files
#include "robot_config.h"
#include <fstream>
//#include "neuro_network.h"
//#include "cmac_network.h"
//#include "cmac.cpp"
#include "Rein_learning.h"
//#include "keyboard_node.cpp"


using namespace std;
using namespace cv;
using namespace sensor_msgs;
using namespace message_filters;
namespace enc = sensor_msgs::image_encodings;
RNG rng(12345);

// subscribers to tactile and touch sensors
ros::Subscriber tactile_sub;
ros::Subscriber bumper_sub;
ros::Subscriber goalkeeper_sub;

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

//---------------------------- LEG Motion: Hajer Chebil, Jakovleski Philipp  -------------------------------------------------------

// receive motor state of the LEFT and RIGHT Leg
double motor_l_leg_in[L_LEG_DOF];  //left leg, NEW FOR TUT 5
double motor_r_leg_in[R_LEG_DOF];  //right leg, NEW FOR TUT 5

int flag=0;
double VAR_JOINT0,VAR_JOINT1;
double init_head[HEAD_DOF] = {-0.362066, 0.276078};  //  //  -0.18719, 0.053648

double move_interval = 0.08;

//double dummy_safe_left[L_ARM_DOF]= { 0.527654,0.271476,-0.635118,-1.27318,-1.00481,0.392};
//double dummy_move_left[L_ARM_DOF] = { 0.438682,0.268408,-1.31008,-1.34067,-1.00481,0.3916 };
//double dummy_move_2_left[L_ARM_DOF] = {0.417206,0.269942,-1.98197,-1.34528,-1.00481,0.3916};
//double dummy_inital_right[R_ARM_DOF] ={0.83914,-0.251618,0.34204,1.29167,1.04154,0.3932};
//double dummy_safe_right[R_ARM_DOF]= {0.527654,-0.271476,0.635118,1.27318,1.00481,0.3922};
//double dummy_move_right[R_ARM_DOF] = {0.438682,-0.268408,1.31008,1.34067,1.00481,0.3916};
//double dummy_move_2_right[R_ARM_DOF] = {0.417206,-0.269942,1.98197,1.34528,1.00481,0.3916};
//double dummy_tracking[6]= {0,0,0.18719,1.29312, 0.998678,0.3956}; //{0,0,0.722472,-0.0349066,-1.52024,0.822}; //{0,0,-0.18719,-1.29312, -0.998678,0.3956};  // {0,0,0.722472,-0.0349066,-1.52024,0.822};//motor_l_arm_in[2],motor_l_arm_in[3],motor_l_arm_in[4],motor_l_arm_in[5]};

// The pose of standing
double dummy_r_leg_safe[R_LEG_DOF] =  {-0.111, -0.105804, -0.0923279, 0.125746, -0.01,0.07};  // -0.046062};//{-0.139552, -0.105804, -0.0061779, -0.0923279, 0.116626, 0}; //{-0.174834, 0.128898, 0, -0.0923279, 0.197928, 0};   //{0.0583339, 0.338972, 0.0521979, -0.133416, -0.0705221, 0}; //{-0.251534,0.15,0.888228,-0.391128,0.0583339,0.5};//{-0.18864,-0.00310993,0.698012,-0.66418,0.0890141,0};
double dummy_l_leg_safe[L_LEG_DOF] = {-0.111, 0.105804, -0.0923279, 0.125746, -0.01, -0.07};  //0.046062};//{0.265424, -0.208582, 0, -0.239346, -0.0475121, 0}; //{0.211734, 0.257754, -0.01845, 0.00609398, -0.184038,0};  //{-0.220854, 0.25, 0.214718, -0.0383921,0.023052,-0.5}; //{-0.0950661,-0.230058,0.745482,-0.45564,0.016916,0};

// The pose before kicking
double kickpos_r_leg[R_LEG_DOF] = {-0.174834,-0.2, -0.25622, 0.5, 0.1, 0.0153821};
double kickpos_l_leg[L_LEG_DOF] = {-0.174834,0.0, 0.0106959, 0.25, -0.142704, 0.3};  // hip -0.0106959

// The pose of kicking
double roll_r_leg = kickpos_r_leg[1];
double kick_r_leg[R_LEG_DOF] = {-0.174834,-0.2, -1, 0.5, 0.5, 0.0153821};

//-------------------------------------------------------------------------------------------------------------------------------------


// label of the GUI window showing the raw image of NAO's camera
static const char cam_window[] = "NAO Camera (raw image)";
static const char hsv_window[] = "(HSV image)";
static const char color_window[] = "(color image)";

int btn_flag;
int pixel_x;
int pixel_y;
int goal_state;



// set the stiffness
void setStiffness_head(float value)
{
  cout << "setting stiffnesses (head) to " << value << endl;
  robot_specific_msgs::JointState target_joint_stiffness;
  // set stiffnesses of HEAD joints

  target_joint_stiffness.name.push_back("Head");
  target_joint_stiffness.effort.clear();
  for (int i=0; i<HEAD_DOF; i++)
    target_joint_stiffness.effort.push_back(value);
  stiffness_pub.publish(target_joint_stiffness);

}



// Set stiffness for head and elbow
void setStiffness_shoulder(float value)
{
  cout << "setting stiffnesses of shoulder's pitch and row to " << value << endl;

  robot_specific_msgs::JointState target_joint_stiffness;

  // set stiffnesses of L_ARM joints
  target_joint_stiffness.name.clear();
  target_joint_stiffness.name.push_back("LArm");

  // set the pitch and row of shoulder's stiffness zero
  float stiffnesses[] = {0.9,0.9,0.9,0.9,0.9,0.9};
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
//------------------------------------- VISION: Wu Lifan , Siyuan Liu, Chao Dong -----------------------------------------------------
void visionCB_goal(const std_msgs::Float64MultiArray::ConstPtr& msg)   // ::ConstPtr&msg
{

  // x position
  float L_bound = msg->data[0]; // The left bound of goal
  float R_bound = msg->data[1]; // The Right bound of goal
  float U_bound = msg->data[2]; // The Up bound of goal
  float Goalkeeper = msg->data[3]; // The position of goalkeeper
  float range = R_bound-L_bound;
  float interval = range/5;

  // To decide the state of goal keeper
  if( L_bound <= Goalkeeper &&  Goalkeeper <= (L_bound + interval)){
    goal_state = 0;
  }else if ( (L_bound + interval) <= Goalkeeper &&  Goalkeeper <= (L_bound + 2*interval) ){
    goal_state = 1;
  }else if ( (L_bound + 2*interval) <= Goalkeeper &&  Goalkeeper <= (L_bound + 3*interval)){
    goal_state = 2;
  }else if ( (L_bound + 3*interval) <= Goalkeeper &&  Goalkeeper <= (L_bound + 4*interval)){
    goal_state = 3;
  }else if ( (L_bound + 4*interval) <= Goalkeeper &&  Goalkeeper <= (L_bound + 5*interval)){
    goal_state = 4;
  }
  cout<<" X interval : "<< L_bound<< "  "<<  L_bound + interval<< "  "<< L_bound + 2*interval<< "  "<< L_bound + 3*interval<< "  "<<L_bound + 4*interval << " " << L_bound + 5*interval<<endl;
  cout << "Goal state: " << goal_state  <<  " L:  "  <<  L_bound<< " G:  " << Goalkeeper  << " R:  " << R_bound << endl;
  sleep(2);
}

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
  cv::inRange(HSV_image,Scalar(0,100,100),Scalar(10,255,255),low_red_image);   // red
  cv::inRange(HSV_image,Scalar(160,100,100),Scalar(179,255,255),up_red_image);

  //    cv::inRange(HSV_image,Scalar(110,50,50),Scalar(110,50,50),low_red_image);   //  0-180
  //    cv::inRange(HSV_image,Scalar(110,50,50),Scalar(110,50,50),up_red_image);

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
  int area_temp = 0;
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
      area_temp = boundRect[i].area();

      if(area_temp >= maxarea){
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

    // Output the position of biggest blob --- Ball
    cv::circle(drawing, cv::Point(cent_contr_x, cent_contr_y), 5, CV_RGB(255,0,0));
    cout << " X: " << cent_contr_x << " Y: " << cent_contr_y << " width: " << boundRect[max_area_index].width << "  height: "<< boundRect[max_area_index].height <<endl;
    pixel_x = cent_contr_x;
    pixel_y = cent_contr_y;
    imshow(color_window,drawing);

  }
  //cv::cam_window("NAO Camera (raw image)",CV_WINDOW_AUTOSIZE);
  imshow(cam_window,cv_ptr->image);
  waitKey(3);
}

//---------------------------------------------------------- End for  vision ------------------------------------------------------------------------------------------------

// callback function for bumpers
void bumperCB(const robot_specific_msgs::Bumper::ConstPtr& __bumper)
{
  // check each bumper
  static bool left_bumper_flag = false;
  static bool right_bumper_flag = false;
  // check left bumper
  if (((int)__bumper->bumper == 1) && ((int)__bumper->state == 1))
  {
    left_bumper_flag = !left_bumper_flag;   // toggle flag
    // do something, e.g.:
    // set / reset stiffness
    /*if (left_bumper_flag)
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
  //  cout << "Left arm joints:  ";
  //  for (int i=0; i<L_ARM_DOF; i++)
  //    cout << motor_l_arm_in[i] << " ";
  //  cout << endl;


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
  //  cout << "Right arm joints:  ";
  //  for (int i=0; i<R_ARM_DOF; i++)
  //    cout << motor_r_arm_in[i] << " ";
  //  cout << endl;

  //------------------------------------------------  LEG Motion : Hajer Chebil, Jakovleski Philipp -------------------------------------------------------------------------

  // JOINT STATE LEGS : LEFT AND RIGHT
  buffer.data.clear();
  for (int i=0; i<ROBOT_DOF; i++)  //right
  {
    if (joint_state->name[i] == "RHipYawPitch")
    {
      buffer.data.push_back(joint_state->position[i]);
      // cout << joint_state->name[i] << endl;
    }
    if (joint_state->name[i] == "RHipRoll")
    {
      buffer.data.push_back(joint_state->position[i]);
      // cout << joint_state->name[i] << endl;
    }
    if (joint_state->name[i] == "RHipPitch")
    {
      buffer.data.push_back(joint_state->position[i]);
      // cout << joint_state->name[i] << endl;
    }
    if (joint_state->name[i] == "RKneePitch")
    {
      buffer.data.push_back(joint_state->position[i]);
      // cout << joint_state->name[i] << endl;
    }
    if (joint_state->name[i] == "RAnklePitch")
    {
      buffer.data.push_back(joint_state->position[i]);
      // cout << joint_state->name[i] << endl;
    }
    if (joint_state->name[i] == "RAnkleRoll")
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
    motor_r_leg_in[idx] = *iter;
    idx++;
  }
  // display data on terminal:right leg
  cout << "right leg joints:  ";
  for (int i=0; i<6; i++)
    cout << motor_r_leg_in[i] << " ";
  cout << endl;
  //left leg

  buffer.data.clear();
  for (int i=0; i<ROBOT_DOF; i++)
  {
    if (joint_state->name[i] == "LHipYawPitch")
    {
      buffer.data.push_back(joint_state->position[i]);
      // cout << joint_state->name[i] << endl;
    }
    if (joint_state->name[i] == "LHipRoll")
    {
      buffer.data.push_back(joint_state->position[i]);
      // cout << joint_state->name[i] << endl;
    }
    if (joint_state->name[i] == "LHipPitch")
    {
      buffer.data.push_back(joint_state->position[i]);
      // cout << joint_state->name[i] << endl;
    }
    if (joint_state->name[i] == "LKneePitch")
    {
      buffer.data.push_back(joint_state->position[i]);
      // cout << joint_state->name[i] << endl;
    }
    if (joint_state->name[i] == "LAnklePitch")
    {
      buffer.data.push_back(joint_state->position[i]);
      // cout << joint_state->name[i] << endl;
    }
    if (joint_state->name[i] == "LAnkleRoll")
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
    motor_l_leg_in[idx] = *iter;  //left leg
    idx++;
  }
  // display data on terminal:left leg
  cout << "left leg joints:  ";
  for (int i=0; i<6; i++)
    cout << motor_l_leg_in[i] << " ";
  cout << endl;
  // END JOINT STATE LEGS

  //-----------------------------------------------------------------------------------------------------------------------------------------
}


void sendTargetJointStateLLeg(double dummy_left[L_LEG_DOF]/* dummy representing the comanded joint state */)
{
  robot_specific_msgs::JointAnglesWithSpeed target_joint_state;

  // specify the limb
  target_joint_state.joint_names.clear();
  target_joint_state.joint_names.push_back("LLeg");

  // specifiy the angle
  target_joint_state.joint_angles.clear();
  for (int i=0; i<L_LEG_DOF; i++)
    target_joint_state.joint_angles.push_back(dummy_left[i] /* array containing result */);

  // set speed
  target_joint_state.speed = 0.2;

  // set the mode of joint change
  target_joint_state.relative = 0;

  // send to robot
  target_joint_state_pub.publish(target_joint_state);

}

// send commanded joint positions of the RIGHT LEG
void sendTargetJointStateRLeg(double dummy_right[R_LEG_DOF],double speedinput/* dummy representing the comanded joint state */)
{
  robot_specific_msgs::JointAnglesWithSpeed target_joint_state;

  // specify the limb
  target_joint_state.joint_names.clear();
  target_joint_state.joint_names.push_back("RLeg");

  // specifiy the angle
  target_joint_state.joint_angles.clear();
  for (int i=0; i<R_LEG_DOF; i++)
    target_joint_state.joint_angles.push_back(dummy_right[i] /* array containing result */);

  // set speed
  target_joint_state.speed = speedinput; // kick 0.5

  // set the mode of joint change
  target_joint_state.relative = 0;

  // send to robot
  target_joint_state_pub.publish(target_joint_state);
}

// 0 1 2 3 4
// 0 -0.2 -0.4 interval -0.1
// -0.04  -0.36  interval 0.08
// Default range -0.4~0.0
// To move the position gradually according to the action
void leg_move_in(){
  roll_r_leg +=0.1;
  if (roll_r_leg<= -0.2+2*move_interval ){  // max to reach is 0
    kickpos_r_leg[1] =roll_r_leg;
    kick_r_leg[1] = roll_r_leg;
    sendTargetJointStateRLeg(kickpos_r_leg,0.2);
  }else{
    roll_r_leg -= move_interval;  // 0.1
  }
}
void leg_move_out(){
  roll_r_leg -=0.1;
  if (roll_r_leg>= -0.2-2*move_interval){  // min to reach is -0.4
    kickpos_r_leg[1] =roll_r_leg;
    kick_r_leg[1] = roll_r_leg;
    sendTargetJointStateRLeg(kickpos_r_leg,0.2);
  }else{
    roll_r_leg += move_interval;  // 0.1
  }
}
//------------------------------------------------------- LEG Motion : Hajer Chebil, Jakovleski Philipp ----------------------------------------------------------------------------
//---------------------------------------------------------------------------------------------------------------------------------------------

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

  // check TB 2 (middle)key pushed: a

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

//---------------------------------  ALL GROUP MEMBER write the QL part together and modify this part  -----------------------------------
int main(int argc, char** argv)
{
  ros::init(argc, argv, "central_node");
  ros::NodeHandle central_node_nh;
  // -0.980967  -0.712769  -0.44457  -0.176371  0.0918273 0.360026 :  five goal keeper position

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

  // subscribe to tactile and touch sensors
  tactile_sub = central_node_nh.subscribe("tactile_touch", 1, tactileCB);
  bumper_sub = central_node_nh.subscribe("bumper", 1, bumperCB);
  goalkeeper_sub = central_node_nh.subscribe("/obstacle_x", 1, visionCB_goal);

  // set up the subscriber for the keyboard
  ros::Subscriber key_sub;
  key_sub = central_node_nh.subscribe("key", 5, keyCB);
  namedWindow(cam_window, 0);

  // Declare the class
  RL R_L;
  //RL.RL();

  // Setting initial state
  R_L.LegState = 2;     // Now State
  R_L.LegStateNew = 2;  // Next State
  int RLLegstateOld = 0;// Old State


  // Initialize R_L.action
  int GKP = 0;   // Train for different positions of goal keeper by using "goal_state"

  bool first_time = true;
  int foot_state = 2;
  int last_foot_state =2;
  double last_kick_angle;
  bool tb_out = false;
  bool tb_in = false;

  // Initial P Q matrix
  R_L.initQN(GKP);
  R_L.initP();

  /* Main Loop
   Button 1: Stand Up
   Button 2: Lift leg for kicking
   Button 3: Mode1: Q table training  Mode2: test the performance
   */
  while(ros::ok()){

    // Stand pose
    if (btn_flag == 1){
      sendTargetJointStateHead(init_head);
      sendTargetJointStateRLeg(dummy_r_leg_safe,0.2);
      sendTargetJointStateLLeg(dummy_l_leg_safe);
      roll_r_leg = dummy_r_leg_safe[1];
    }
    if(btn_flag == 3 ){
      // ----------------- Kick Test ----------------------------------------

////      setStiffness_head(0);

//      // To the position
//      //  prepare to kick left
//      kickpos_r_leg[1] = -0.2+2*move_interval;  // 0.0
//      roll_r_leg = kickpos_r_leg[1];
//      sendTargetJointStateRLeg(kickpos_r_leg,0.2);
//      sleep(2);


//      // kick
//      kick_r_leg[1]= roll_r_leg; // same position of the leg for kicking
//      sendTargetJointStateRLeg(kick_r_leg,0.5); // kick dummy array
//      sleep(1);

//      // recover
//      cout << "go mid" << endl;
//      kickpos_r_leg[1]= roll_r_leg;
//      sendTargetJointStateRLeg(kickpos_r_leg,0.3);
//      sleep(1);

//      // return initial
//      kickpos_r_leg[1] = -0.2;
//      roll_r_leg = kickpos_r_leg[1];
//      sendTargetJointStateRLeg(kickpos_r_leg,0.2);
//      sleep(2);


//      // return initial ==
//      kickpos_r_leg[1] = -0.2-2*move_interval;  // -0.4
//      roll_r_leg = kickpos_r_leg[1];
//      sendTargetJointStateRLeg(kickpos_r_leg,0.2);
//      sleep(2);


//      // kick
//      kick_r_leg[1]= roll_r_leg; // same position of the leg for kicking
//      sendTargetJointStateRLeg(kick_r_leg,0.5); // kick dummy array
//      sleep(1);

//      // recover
//      cout << "go mid" << endl;
//      kickpos_r_leg[1]= roll_r_leg;
//      sendTargetJointStateRLeg(kickpos_r_leg,0.3);
//      sleep(1);

//      // return initial
//      kickpos_r_leg[1] = -0.2;
//      roll_r_leg = kickpos_r_leg[1];
//      sendTargetJointStateRLeg(kickpos_r_leg,0.2);
//      sleep(2);

      // -----------------    Mode2: test the performance  ----------------------------------------

//      cout << "Test Q table!!!! " << endl;
//      int s_goal_state = goal_state;
//      cout << "Now Goal Keeper state is: " <<  s_goal_state << endl;
//      sleep(2);

//      //Read the Q table
//      vector < vector <vector<double> > >  Q_Final =vector< vector <vector<double> > >(5, vector<vector <double> >(3, vector<double>(5)));
//      ifstream myfileQ;
//      myfileQ.open ("/home/bilhr_c/catkin_ws/Q.txt");
//      for(int k = 0;k<5;k++){
//        for(int i = 0;i<5;i++){
//          for(int j=0; j<3;j++){
//            myfileQ >> Q_Final[i][j][k];
//            cout<<Q_Final[i][j][k]<<endl;
//          }
//        }
//      }
//      myfileQ.close();


//      // Initial leg state
//      int Action_Final = 0;
//      int Q_compare_value = -100;
//      double kickpos_r_leg_testQ[R_LEG_DOF] = {-0.174834,-0.2, -0.25622, 0.5, 0.1, 0.0153821};  // inital
//      double kick_r_leg_testQ[R_LEG_DOF] = {-0.174834,-0.2, -1, 0.5, 0.5, 0.0153821};  // kick

//      if(first_time){
//        foot_state = 2; // 0-4    5.0*rand()/(RAND_MAX); // 0 - 4.99
//        first_time = false;
//      }else{
//        foot_state = last_foot_state;
//        kickpos_r_leg_testQ[1] = last_kick_angle;
//      }


//      //if(foot_state==2){
//      //  kickpos_r_leg[1] = (-0.1)*foot_state;
//      //}else if(rand_pos>2){
//      //  kickpos_r_leg[1] = kickpos_r_leg[1] - (foot_state-2)*move_interval; // 3 4
//      //}else{
//      //  kickpos_r_leg[1] = kickpos_r_leg[1] + (2-foot_state)*move_interval; // 0 1
//      // }

////      kickpos_r_leg_testQ[1] = -0.1*foot_state;
////      sendTargetJointStateRLeg(kickpos_r_leg_testQ,0.2); // Do motion
////      sleep(5);


//      // Do the motions according to Q table until the robot kick the ball
//      while(Action_Final != 2){
//        // Search the best action for each state
//        for(int j=0;j<3;j++){
//          if(Q_Final[foot_state][j][s_goal_state] > Q_compare_value){
//            Q_compare_value = Q_Final[foot_state][j][s_goal_state];
//            Action_Final = j;
//          }
//        }

//        if(tb_out && Action_Final==1){
//          Action_Final = 0;
//          tb_out = false;
//        }else if(tb_in && Action_Final==0){
//          Action_Final = 1;
//          tb_out = false;
//        }

//        // Do the action
//        // If in the right/left bound state, stay in this state
//        // Otherwise, do the move in / move out / Kicking motion
//        if( foot_state == 4 && Action_Final == 1) {  //-1
//          foot_state = 4;
//          cout<<"Bound R !!! No Action !!!"<<endl;
//          sendTargetJointStateRLeg(kickpos_r_leg_testQ,0.2); // Do motion
//          sendTargetJointStateRLeg(kickpos_r_leg_testQ,0.2);
//          sendTargetJointStateRLeg(kickpos_r_leg_testQ,0.2);
//          tb_out = true;
//         sleep(1);

//        }else if(foot_state == 0 && Action_Final == 0){
//          foot_state = 0;  //+1
//          cout<<"Bound L !!! No Action !!!"<<endl;
//          sendTargetJointStateRLeg(kickpos_r_leg_testQ,0.2); // Do motion
//          sendTargetJointStateRLeg(kickpos_r_leg_testQ,0.2);
//          sendTargetJointStateRLeg(kickpos_r_leg_testQ,0.2);
//          tb_in = false;
//          sleep(1);


//        }else if( Action_Final == 1){

//          cout<<"Now the action is : "<<Action_Final<<endl;

//          foot_state = foot_state +1;
//          kickpos_r_leg_testQ[1]+=move_interval; // 0.1
//          sendTargetJointStateRLeg(kickpos_r_leg_testQ,0.2); // Do motion
//          sendTargetJointStateRLeg(kickpos_r_leg_testQ,0.2);
//          sendTargetJointStateRLeg(kickpos_r_leg_testQ,0.2);
//          sleep(1);

//        }else if( Action_Final == 0){

//          cout<<"Now the action is : "<<Action_Final<<endl;

//          foot_state = foot_state -1;
//          kickpos_r_leg_testQ[1]-=move_interval;
//          sendTargetJointStateRLeg(kickpos_r_leg_testQ,0.2); // Do motion
//          sendTargetJointStateRLeg(kickpos_r_leg_testQ,0.2);
//          sendTargetJointStateRLeg(kickpos_r_leg_testQ,0.2);
//          sleep(1);


//        }else{ // Kicking action

//          cout<<"Now the action is : "<<Action_Final<<endl;
//          kick_r_leg_testQ[1]=  kickpos_r_leg_testQ[1]; // same position of the leg for kicking
//          sendTargetJointStateRLeg(kick_r_leg_testQ,0.5);   // kick dummy array
//          sendTargetJointStateRLeg(kick_r_leg_testQ,0.5);
//          sendTargetJointStateRLeg(kick_r_leg_testQ,0.5);
//          sleep(5);
//          sendTargetJointStateRLeg(kickpos_r_leg_testQ,0.2); // Recover
//          sendTargetJointStateRLeg(kickpos_r_leg_testQ,0.2);
//          sendTargetJointStateRLeg(kickpos_r_leg_testQ,0.2);
//          sleep(3);
//          //kickpos_r_leg_testQ[1] = -0.2;  // Initial pose   // set to initial position after every kick
//          //foot_state = 2;
//          //sendTargetJointStateRLeg(kickpos_r_leg_testQ,0.2);
//          cout << "Finish kicking"<< endl;
//        }
//        cout <<" Now State: " << foot_state << " angle: "<< kick_r_leg_testQ[1] << endl;
//        sleep(1);
//      }
//      last_foot_state = foot_state;
//      last_kick_angle = kick_r_leg_testQ[1];


 //      --------------------------------    Mode1: Q table training
            vector < vector <vector<double> > >  Q_initial =vector< vector <vector<double> > >(5, vector<vector <double> >(3, vector<double>(5)));
            vector < vector <vector<double> > >  Q_update =vector< vector <vector<double> > >(5, vector<vector <double> >(3, vector<double>(5)));


            // Initialize the Q matrix
            for(int i = 0;i<5;i++){
              for(int j = 0; j<3;j++){
                Q_initial[i][j][GKP] = 0;
                Q_update[i][j][GKP]=0;
              }
            }

            // Trail : From initial state until robot do the kicking motion
            // if the kicking action is done, kk trun to 0, otherwise kk =1
            // When kk trun from 1 to 0, one trial is done
            int kk=1;

            while(kk==1){

              // For every trail, we initialize the N table again.
              // N : number of state action pair <state - action - next state> in one trail
              R_L.initN();

              // Get the Goal keepers position GKP
              int VisionState = GKP;
              int LastAction = 0;


              // If the action is not kicking, i.e. this trail is not finished
              while(LastAction !=2){

                srand(time(0));
                // REWARDS: Move in/out -1, shoot the Goal: 20, Missing the goal: -10;
                // Update the legstate
                R_L.LegState = R_L.LegStateNew;

                cout << "Goal keeper state: " << VisionState <<" Legs States: "<< R_L.LegState << endl;
                // Get the action for current state
                R_L.performAction(GKP,R_L.LegState);
                cout << "Current State: " << R_L.LegState << " action: " <<  R_L.Action << " Next State: " << R_L.LegStateNew << endl;

                int act =R_L.getAction();
                // According to the actions
                if (act == 0){  //leg in
                  leg_move_in();
                }else if (act ==1){//leg out
                  leg_move_out();
                }else if (act ==2){  //kick
                  kick_r_leg[1]= roll_r_leg; // same position of the leg for kicking
                  sendTargetJointStateRLeg(kick_r_leg,0.5); // kick dummy array
                  sleep(1);

                  // recover
                  cout << "go mid";
                  kickpos_r_leg[1]= roll_r_leg;
                  sendTargetJointStateRLeg(kickpos_r_leg,0.2);
                  sleep(1);

                  cout << " wait motion done"<< endl;
                  // return initial
                  kickpos_r_leg[1] = -0.2;
                  roll_r_leg = kickpos_r_leg[1];
                  sendTargetJointStateRLeg(kickpos_r_leg,0.2);
                  sleep(1);

                  // Random
                  cout << "Random pos" << endl;
                  int rand_pos =5.0*rand()/(RAND_MAX); // 0 - 4.99
                  //kickpos_r_leg[1] = (-0.1)*rand_pos;  // -0.2

                  if(rand_pos==2){
                    kickpos_r_leg[1] = (-0.1)*rand_pos;
                  }else if(rand_pos>2){
                    kickpos_r_leg[1] = kickpos_r_leg[1] - (rand_pos-2)*move_interval; // 3 4
                  }else{
                    kickpos_r_leg[1] = kickpos_r_leg[1] + (2-rand_pos)*move_interval; // 0 1
                  }

                  cout << "rand_pos: " << rand_pos << " r_leg[1] " << kickpos_r_leg[1] << endl;
                  sendTargetJointStateRLeg(kickpos_r_leg,0.2);
                  R_L.LegStateNew = rand_pos;
                  sleep(1);



                }
                // R_L.updateN(R_L.LegState,R_L.Action,R_L.LegStateNew);
                // R_L.ProbCalculation(R_L.LegState,act,R_L.LegStateNew);
                R_L.N_Calculation(R_L.LegState,act);
                //R_L.updateN(R_L.LegState,act,R_L.LegStateNew);
                R_L.bestAction(R_L.LegStateNew);
                Q_update = R_L.updateQ(R_L.LegState, act);
                RLLegstateOld = R_L.LegState;

                R_L.saveQ();//(Q);
                R_L.action_step++;
                R_L.saveN();  //(N);
                LastAction = act;

              }
              R_L.P_Calculation();
              //  R_L.performAction_exploitation(VisionState,R_L.LegState);
              cout << "The ball has been kicked" << endl;

              //cout<<"TEST Interation index: 4 "<<endl;
              // determine , if the Q matrix is converge
              // if converge kk change from 1 to 0 , stop the training while loop
              // if not converge , kk remains 1, keep training
              int num=0;
              for(int i = 0;i<5;i++){
                for(int j = 0; j<3;j++){

                  if(Q_initial[i][j][GKP] == Q_update[i][j][GKP]){
                    num++;
                  }
                }
              }

              cout<<"The "<<num<<" of Q have already converged"<<endl;
              if(num == 15){
                kk=0;
                cout<<"Converge!!!!"<<endl;
              }
              //cout<<"TEST Interation index: 2 "<<endl;
              // we store thee updated Q matrix inside the Q_initial
              // it can be used to check the convergency for the next iteration
              for(int i = 0;i<5;i++){
                for(int j = 0; j<3;j++){
                  Q_initial[i][j][GKP] == Q_update[i][j][GKP];

                }
              }
             //cout<<"TEST Interation index:3  "<<endl;
            }// end for while , if converge , stop

      //=============================================================


    }
    if(btn_flag == 2){

      kickpos_r_leg[1] = -0.2;
      roll_r_leg = kickpos_r_leg[1];
      sendTargetJointStateLLeg(kickpos_l_leg);
      sendTargetJointStateRLeg(kickpos_r_leg,0.2);



    }
    ros::spinOnce();
  }

  //doing_movement();

  return 0;


}
