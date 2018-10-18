//
//  cmac.h
//  CMAC
//  Created by apple on 2018/5/17.
//  Copyright © 2018年 apple. All rights reserved.
//
//   Author: Siyuan Liu, Lifan Wu, Philips Jakovleski


#ifndef cmac_h
#define cmac_h

#include <eigen3/Eigen/Dense>

using namespace Eigen;


class CMAC
{
public:
    
    void initial(int resolution,int field_size, int channel_num,int trainPair_num,float alpha,double centerX[150], double centerY[150], double out[150],double out2[150]);
    void activeNeuronPosition(int m);
    void update(int m);
    std::vector<float> prediction(std::vector<float> test);
    float getError();
    
private:
    
    
    int m_resolution;
    int m_field_size;
    int m_channel_num;
    int m_trainPair_num;

    float m_alpha;
    float m_MSE;
    
    Eigen::MatrixXf m_a;
    Eigen::MatrixXf m_w1;
    Eigen::MatrixXf m_w2;
    
    class TrainPair
    {
    public:
        std::vector<float> y;
        std::vector<float> x;
        std::vector<float> x_target;
        std::vector<float> error;

    };
    
    std::vector<TrainPair> trainPairs;
    
    class Position
    {
    public:
        std::vector<int> Coord;
    };
    
    std::vector<Position> Positions;
    
    class RF
    {
    public:
        std::vector<int> RFpos;
    };
    
    std::vector<RF> RFposes;

};



#endif /* cmac_h */
