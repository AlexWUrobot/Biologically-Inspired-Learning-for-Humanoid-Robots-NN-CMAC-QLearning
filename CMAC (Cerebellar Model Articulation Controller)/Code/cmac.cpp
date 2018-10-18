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
    
    m_w1.resize(m_resolution+4,m_resolution+4);
    m_w2.resize(m_resolution+4,m_resolution+4);
    
    

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
    for(int j=0;j<m_resolution+4;j++){
        for(int k=0;k<m_resolution+4;k++){
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

            // if the index is more or smaller than resolution (We replace this part by using a larger weight matrix in line 35,36: m_resolution+4)
            
//            if(input_index> (m_resolution-m_field_size) ){
//              input_index = m_resolution - m_field_size;
//            }
            
            
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
std::vector<float> CMAC::prediction(std::vector<float> test){
    
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

