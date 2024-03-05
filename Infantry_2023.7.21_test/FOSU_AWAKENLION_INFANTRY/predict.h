#ifndef PREDICT_H
#define PREDICT_H

#include"kalman/armor_kalman.h"
#include <queue>
#include"ArmorDetector/inference_api2.hpp"
#include"ArmorDetector/armor_detector.h"
#include"AngleSolver/PnpSolver.h"
#include<iostream>
#include<opencv2/opencv.hpp>
#include<Eigen/Eigen>

using namespace armor_detector;

class PredictTool
{
public:
    PredictTool(AngleSolver angle_solver,double moto_pitch,double moto_yaw, double *cars_radio,ArmorObject* object_addr,std::unordered_map<int,MultipleValue>cars_map,double runnig_time);

    bool findSameCls();
    bool solveCarRadio();
    bool predictRotated();
    bool predictMove();
    bool stateAdd();
    bool predictArmor();

    Eigen::Vector3d tvec_armor;





private:
    Eigen::Vector3d tvec1;
    Eigen::Vector3d tvec2;
    Eigen::Vector3d moto_tvec1;
    Eigen::Vector3d moto_tvec2;
    Eigen::Vector3d car_tvec;
    double car_angle;
    double *cars_radio;
    double car_radio;
    ArmorObject object_addr;
    std::unordered_map<int,MultipleValue> cars_map;
    AngleSolver angle_solver;

    
    double moto_pitch;
    double moto_yaw;

    Armor_Kalman angle_kalman;
    Armor_Kalman car_kalman;
    double angle_predict;
    Eigen::Vector3d car_predict;
    


    bool switch_y;    
    double running_time;




};

#endif // PREDICT_H
