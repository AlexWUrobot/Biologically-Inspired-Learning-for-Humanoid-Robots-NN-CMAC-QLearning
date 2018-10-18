//
//  cmac.cpp
//  CMAC
//  Created by apple on 2018/5/17.
//  Copyright © 2018年 apple. All rights reserved.
//
//   Author: Siyuan Liu, Lifan Wu, Philips Jakovleski

#include <iostream>
#include <cmath>
#include <vector>
#include "cmac.h"
#include <math.h>
#include <stdio.h>
#include <vector>
#include <eigen3/Eigen/Dense>

#include <sstream>
#include <fstream>

using namespace std;
using namespace Eigen;

void CMAC::initial(int resolution,int field_size,int channel_num,int trainPair_num,float alpha,double centerX[150], double centerY[150], double out[150],double out2[150]){
    
    
    // Initial value
    m_resolution = resolution;
    m_field_size = field_size;
    m_channel_num = channel_num;

    m_trainPair_num = trainPair_num;
    m_alpha = alpha;
    
    m_w1.resize(m_resolution,m_resolution);
    m_w2.resize(m_resolution,m_resolution);
    
    

    Positions.resize(m_field_size);
    RFposes.resize(m_field_size);
    
    // get input and output value

    trainPairs.resize(m_trainPair_num);
    
    for(int m=0;m<m_trainPair_num;m++){
        
        TrainPair& trainPair(trainPairs[m]);

        trainPair.y.resize(2);
        trainPair.x.resize(2);
        trainPair.x_target.resize(2);
        trainPair.error.resize(2);
        
        trainPair.y[0]=centerX[m];
        trainPair.y[1]=centerY[m];
        trainPair.x_target[0]=out[m];
        trainPair.x_target[1]=out2[m];


    }


   
    
    //generate a filed, give the local coordinate to each neuron

// field size = 3

//    for(int r=0;r<m_field_size;r++){
//        RF& pos(RFposes[r]);
//        pos.RFpos.resize(2);
        
//        pos.RFpos[0] = r;
//        pos.RFpos[1] = r;
//    }


// field size = 5
        for(int r=0;r<m_field_size;r++){
            RF& pos(RFposes[r]);
            pos.RFpos.resize(2);

            if(r==0){
              pos.RFpos[0] =0;
              pos.RFpos[1] =3;
            }
            if(r==1){
              pos.RFpos[0] = 1;
              pos.RFpos[1] = 0;
            }
            if(r==2){
              pos.RFpos[0] = 2;
              pos.RFpos[1] = 2;
            }
            if(r==3){
              pos.RFpos[0] = 3;
              pos.RFpos[1] = 4;
            }
            if(r==4){
              pos.RFpos[0] = 4;
              pos.RFpos[1] = 1;
            }


        }
    
cout<<"test field size !!!!"<<endl;

    // initial random weights
    srand(time(0));
    for(int j=0;j<m_resolution;j++){
        for(int k=0;k<m_resolution;k++){
            m_w1(j,k) = -1.0 + 2.0 * rand() / (RAND_MAX);
            m_w2(j,k) = -1.0 + 2.0 * rand() / (RAND_MAX);
        }
    }



}

// compute the active neurons of L2 depending on the input
void CMAC::activeNeuronPosition(int m){

    
    TrainPair& trainPair(trainPairs[m]);
   
    m_a = MatrixXf::Zero(m_resolution,m_resolution);
    
    // get global coordinate of each neuron
    for(int r=0;r<m_field_size;r++){
        
        RF& pos(RFposes[r]);
        Position& position(Positions[r]);
        
        position.Coord.resize(2);
        pos.RFpos.resize(2);
        
        for(int c=0;c<2;c++){
            

            int input_index = round(trainPair.y[c] * m_resolution);

            // if the index is more or smaller than resolution
            if(input_index> (m_resolution-m_field_size) ){
              input_index = m_resolution - m_field_size;
            }
            // calculate the index
            int shift_amount = m_field_size - input_index % m_field_size;

            int local_coord = ( shift_amount + pos.RFpos[c]) % m_field_size;

            int coord = input_index + local_coord;
            position.Coord[c] = coord;
//            cout<<"RFPOSE [ "<<c<<" ] = "<<pos.RFpos[c]<<endl;
//            cout<<"coord"<<c<<" = "<<coord<<endl;
        
        }

//        cout<<"Global coordinate for "<<r<<" th neuron  : "<<position.Coord[0]<<" , "<<position.Coord[1]<<endl;

        m_a(position.Coord[0],position.Coord[1]) = 1;
        
      
    }


//    std::cout<<"m_a"<<m_a<<std::endl;
}
    
void CMAC::update(int m){
    
    TrainPair& trainPair(trainPairs[m]);
    
    
    // calculate output
    
    for(int c=0;c<2;c++){
        
        trainPair.x[c] = 0;
   
        for(int j =0;j<m_resolution;j++){

            for(int k=0;k<m_resolution;k++){

                if(c==0){
                    trainPair.x[c] = trainPair.x[c] + m_w1(j,k) * m_a(j,k);

//                    cout<<"output X : "<< trainPair.x[c]<<endl;

                }
                else{
                    trainPair.x[c] = trainPair.x[c] + m_w2(j,k) * m_a(j,k);
//                    cout<<"output X : "<< trainPair.x[c]<<endl;
                }


            }
        }
    
    } // end for c
    
    
//    cout<<"m_a"<<m_a<<endl;
//    cout<<"target output x : "<<trainPair.x_target[0]<<"  "<<trainPair.x_target[1]<<endl;
//    cout<<"real   output x : "<<trainPair.x[0]<<"  "<<trainPair.x[1]<<endl;
    
    
    // update weight
    
    for(int c=0;c<2;c++){
        
        
        for(int j =0;j<m_resolution;j++){

            for(int k=0;k<m_resolution;k++){

                if(c==0){

                    m_w1(j,k) = m_w1(j,k) + ( ( m_alpha * m_a(j,k) )/m_field_size ) * (trainPair.x_target[c] - trainPair.x[c]);
                    trainPair.error[c] = trainPair.x_target[c] - trainPair.x[c];

                }
                else{
                    m_w2(j,k) = m_w2(j,k) + ( ( m_alpha * m_a(j,k) )/m_field_size ) * (trainPair.x_target[c] - trainPair.x[c]);
                    trainPair.error[c] = trainPair.x_target[c] - trainPair.x[c];
                }

            }
        }
        
        
        
    }
//     cout<<"ERROR "<<trainPair.error[0]<<"  "<<trainPair.error[1]<<endl;


//    cout<<"weight 1 "<<m_w1(5,5)<<endl;
//    cout<<"weight 2 "<<m_w2(5,5)<<endl;
    
}

// calculate the MSE
float CMAC::getError(){
    
    m_MSE = 0;
    float sum1 = 0;
    for(int m=0;m<m_trainPair_num;m++){
        TrainPair& trainPair(trainPairs[m]);
        
        double sum2=0;
        for(int c=0;c<2;c++){
            sum2 = sum2 + trainPair.error[c] * trainPair.error[c];
            
        }
        
        sum1 = sum1 + sum2;
        
    }
    
    m_MSE = sum1 / ( m_trainPair_num * 2 );
    
    return m_MSE;
}

// Prediction
std::vector<float> CMAC::prediction(std::vector<float> test){  // input 0-1
    
 // cout<<"weight 1 TEST "<<m_w1(5,5)<<endl;
 // cout<<"weight 2 TEST "<<m_w2(5,5)<<endl;

    Eigen::MatrixXf Prediction(m_field_size,m_field_size);
    std::vector<float> result;
    result.resize(2);
    result[0]=0;
    result[1]=0;
    double dist=10000000;
    double dist_new;
    int ind;
    bool included_data = true;

    //Obtain the test data
    //========================================================================================== New data training start
    for(int i=0;i<150;i++){
        TrainPair& trainPair(trainPairs[i]);
        if(trainPair.y[0]!=test[0]){
          if(trainPair.y[1]!=test[1]){

            dist_new = (test[0] - trainPair.y[0])*(test[0] - trainPair.y[0]) + (test[1] - trainPair.y[1])*(test[1] - trainPair.y[1]);

            if(dist>dist_new){
              dist = dist_new;
              ind = i;
              included_data = false;
            }
          }

        }

    }
//    if(included_data==false){
//      TrainPair& trainPair(trainPairs[ind]);
//      test[0] = trainPair.y[0];
//      test[1] = trainPair.y[1];
//    }



    //========================================================================================== New data training end

    TrainPair& trainPair(trainPairs[ind]);
    test[0] = trainPair.y[0];
    test[1] = trainPair.y[1];



    // get the coordinates of active neuros for test data
    for(int r=0;r<m_field_size;r++){
        
        RF& pos(RFposes[r]);
        
        for(int c=0;c<2;c++){
            
            int input_index = round(test[c] * m_resolution);
            
            int shift_amount = m_field_size - input_index%m_field_size;
            
            int local_coord = ( shift_amount + pos.RFpos[c]) % m_field_size;
            
            int coord = input_index + local_coord;
        
            Prediction(r,c) = coord;
            
        }
        
        // cout<<"Global coordinate for "<<r<<" th neuron  : "<<position.Coord[0]<<" , "<<position.Coord[1]<<endl;
        
    }
    
 
        // use the active weights to calculate the final output
        for(int r=0;r<m_field_size;r++){
     
            result[0] = result[0] + m_w1(Prediction(r,0),Prediction(r,1));
            result[1] = result[1] + m_w2(Prediction(r,0),Prediction(r,1));
        }
        
    return result;
    
}











void CMAC::Test_network(){

  string data_file_location = "Hands/";
  string load_name ="TrainingData_150_v3";
  string load_path_name = data_file_location + load_name;

  //check which weight is not

//  cout << "======================= x_L2 table ================= " <<endl;

//  // try to find two table value
//  for(int k = 0; k < resolution; k++){
//      for(int j = 0; j < resolution; j++){
//        if(x_L2[k][j] == x_L2_init[k][j]){
//          cout<< "1 "; // the same
//      }else{
//        cout<< "x ";
//      }
//      }
//      cout << endl;
//    }

//  cout << "======================= y_L2 table ================= " <<endl;

//	for(int j = 0; j < resolution; j++){
//	    for(int h = 0; h < resolution; h++){
//	    	if(y_L2[j][h] == y_L2_init[j][h]){
//          cout<< "0 "; // the same
//			}else{
//				cout<< "x ";
//			}
//	    }
//	    cout << endl;
//  	}

  //=========================================================== Orignial Data
  string line;
  ifstream myfile ( load_path_name.c_str() );
  int line_num = 0;
  int iter = 0;

  string delimite_space =" ";
  std::string in_out_arr[N+Q]; // 2+2
  string s;
  int npos;
  float c_test[2];


  if ( myfile.is_open() ){
    while ( getline(myfile,line) )
    {
      line_num++;
    }
    cout << "=================Start Test Network sample points:" << line_num << endl;
    int train_data_num = line_num;

    float a[line_num][N];
    float c[line_num][Q];

    // Set the index to zero
    myfile.clear();
    myfile.seekg(0, ios::beg);

    while (getline(myfile,line))
    {
      for(int i=0; i<N+Q-1; i++ ){ // 4-1 = 3 space
        if(i==0){
          s = line;
        }else{
          s = in_out_arr[i];
        }
        npos = 0;
        in_out_arr[i]= s.substr (npos, s.find(delimite_space));
        npos+= in_out_arr[i].size() + delimite_space.size();
        in_out_arr[i+1] = s.substr (npos);
      }
      // Train Input  0 1
      for(int i=0; i<N; i++ ){
        a[iter][i] = string_to_double(in_out_arr[i]);
      }
      // Train Out  2 3
      for(int i=0; i<Q; i++ ){
        c[iter][i] = string_to_double(in_out_arr[N+i]);
      }
      iter++;
    }
    // Test data
    for (int i= 0; i< train_data_num; i++){
      cout << i <<" InOut:  ";
      for(int k=0; k<N; k++ ){
        cout << a[i][k] <<" ";
      }
      //Calculate_pos(a[i][0], a[i][1], c_test);

      std::vector<float> bc;
      std::vector<float> Calpos;

      bc.resize(2);
      bc[0] =  a[i][0]/640.0;
      bc[1] =  a[i][1]/480.0;

      Calpos.resize(2);
      Calpos = prediction(bc);
      Calpos[0] = Calpos[0]*2*2.0857 - 2.0857;
      Calpos[1] = Calpos[1]*1.6407 - 0.3142;


      for(int k=0; k<Q; k++ ){
        cout << c[i][k] <<" ";
      }
      cout <<"  ||  Predic:";
      for(int k=0; k<Q; k++ ){
        cout << Calpos[k] <<" ";
      }
      cout <<"Err:";
      for(int k=0; k<Q; k++ ){
        cout <<  c[i][k]-Calpos[k] <<" ";
      }
      cout << endl;
    }
  }
  else{
    cout << "Can not find the test file !" << endl;
  }
  //=========================================================== whole Data


}



double CMAC::string_to_double( const std::string& s ){
  std::istringstream i(s);
  float x;
  //double x;
  if (!(i >> x))
    return 0;
  return x;
}










