#include"kalman/armor_kalman.h"
Eigen::Matrix2d Armor_Kalman::P=Eigen::Matrix2d::Identity();
Eigen::Vector2d Armor_Kalman::x_k1=Eigen::Vector2d::Zero();
