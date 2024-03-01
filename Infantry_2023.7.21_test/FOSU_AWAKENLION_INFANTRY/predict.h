#ifndef PREDICT_H
#define PREDICT_H

#include"kalman/armor_kalman.h"
#include"ArmorDetector/inference_api2.hpp"
#include<iostream>
#include<opencv2/opencv.hpp>
#include<Eigen/Eigen>

using namespace armor_detector;

class predict
{
public:
    predict(Eigen::Vector3d tvec,Eigen::Vector3d rvec,double *cars_radio,int object_cls);

    bool findSameCls();
    bool solveCarRadio();
    bool predictRotated();
    bool predictMove();
    bool stateAdd();
    bool predictArmor();






private:
    Eigen::Vector3d tvec1;
    Eigen::Vector3d rvec1;
    Eigen::Vector3d tvec2;
    Eigen::Vector3d rvec2;
    double car_radio;
    ArmorDetector ad;




};

#endif // PREDICT_H
