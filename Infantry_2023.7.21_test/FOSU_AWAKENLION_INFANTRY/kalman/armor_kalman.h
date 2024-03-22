#ifndef _ARMOR_KALMAN_H_
#define _ARMOR_KALMAN_H_

#include<eigen3/Eigen/Dense>
#include<iostream>

using namespace std;
class Armor_Kalman {

private:
    Eigen::Vector2d x_k1; 
    Eigen::Matrix2d K;    
    Eigen::Matrix2d A;    
    Eigen::Matrix2d H;    
    Eigen::Matrix2d R;    
    Eigen::Vector2d Q;    
    Eigen::Matrix2d P;    

    double t;
public:
    Armor_Kalman() = default;

    Armor_Kalman(Eigen::Matrix2d A, Eigen::Matrix2d H, Eigen::Matrix2d R, Eigen::Matrix2d Q, Eigen::Vector2d init, double t) {
        reset(A, H, R, Q, init, t);
    }

    void reset(Eigen::Matrix2d A, Eigen::Matrix2d H, Eigen::Matrix2d R, Eigen::Matrix2d Q, Eigen::Vector2d init, double t) {
        this->A = A;
        this->H = H;
        this->P <<0.0,0.0,0.0,0.0;
        this->R = R;
        this->Q = Q;
        x_k1 = init;
        this->t = t;
    }
   Eigen::Vector2d update(Eigen::Vector2d z_k, double t) {
       this->t=t;
       A(0,1)=t;
       Eigen::Vector2d p_x_k = Eigen::saturate(A * x_k1);  
       P = Eigen::saturate(A * P * A.transpose() + A * Q * A.transpose()) ; 
       K = Eigen::saturate(P * H.transpose() * (H * P * H.transpose() + R).inverse()) ;  
       x_k1 = p_x_k + K * (z_k - H * p_x_k); 
       P = Eigen::saturate((Eigen::Matrix2d::Identity() - K * H) * P);
       return x_k1;
   }
};


#endif
