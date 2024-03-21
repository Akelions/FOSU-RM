#ifndef _ARMOR_KALMAN_H_
#define _ARMOR_KALMAN_H_

#include<eigen3/Eigen/Dense>
#include<iostream>

using namespace std;
class Armor_Kalman {

private:
    Eigen::Vector2d x_k1; // k-1时刻的滤波值，即是k-1时刻的值
    Eigen::Matrix2d K;    // Kalman增益
    Eigen::Matrix2d A;    // 转移矩阵
//    Eigen::Matrix3d A_d;    // 转移矩阵的微分
    Eigen::Matrix2d H;    // 观测矩阵
    Eigen::Matrix2d R;    // 预测过程噪声偏差的方差
    Eigen::Vector2d Q;    // 测量噪声偏差，(系统搭建好以后，通过测量统计实验获得)
//    Eigen::Vector3d one;
    Eigen::Matrix2d P;    // 估计误差协方差

    double t;
public:
    Armor_Kalman() = default;

    Armor_Kalman(Eigen::Matrix2d A, Eigen::Matrix2d H, Eigen::Matrix2d R, Eigen::Vector2d Q, Eigen::Vector2d init, double t) {
        reset(A, H, R, Q, init, t);
    }

    void reset(Eigen::Matrix2d A, Eigen::Matrix2d H, Eigen::Matrix2d R, Eigen::Vector2d Q, Eigen::Vector2d init, double t) {
        this->A = A;
//        this->A_d=Eigen::Matrix3d::Identity();
        this->H = H;
        this->P <<0.0,0.0,0.0,0.0;
        this->R = R;
        this->Q = Q;
        x_k1 = init;
        this->t = t;
    }

    void reset(Eigen::Vector2d init, double t) {
        x_k1 = init;
        this->t = t;
    }


   Eigen::Vector2d update(Eigen::Vector2d z_k, double t) {
        // 设置转移矩阵中的时间项
        this->t=t;
       A(0,1)=0.0;
        // 预测下一时刻的值
        if(isnan(x_k1(0,0))||(x_k1(0,0)==0&&x_k1(1,0)==0)){
            x_k1=z_k;
            Eigen::Vector2d p_x_k = A * x_k1 /*+ Q*/;   //x的先验估计由上一个时间点的后验估计值和输入信息给出
            //求协方差
            if(isnan(P(0,0))||isnan(P(1,1))){
                this->P<<0.0,0.0,0.0,0.0;;
                std::cout<<"nan!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"<<std::endl;
                std::cout<<"is it recover??????????????????????????????????????????????????????????????????"<<std::endl;
                return z_k;
            }
            else{
                 P = A * P * A.transpose() ;  //计算先验均方差 p(n|n-1)=A^2*p(n-1|n-1)+q
                 if(isnan(P(0,0))||isnan(P(1,1))){
                     this->P<<0.0,0.0,0.0,0.0;;
                     return z_k;
                 }
                 else{
                      //计算kalman增益
                      K = P * H.transpose() * (H * P * H.transpose()).inverse() ;  //Kg(k)= P(k|k-1) H’ / (H P(k|k-1) H’ + R)
                      if(isnan(K(0,0))||isnan(K(1,1))){
                          this->P=Eigen::Matrix2d::Identity();
                          return z_k;
                      }
                      else{
                           //修正结果，即计算滤波值
                           x_k1 = p_x_k + K * (z_k - H * p_x_k);  //利用残余的信息改善对x(t)的估计，给出后验估计，这个值也就是输出  X(k|k)= X(k|k-1)+Kg(k) (Z(k)-H X(k|k-1))
                           if(isnan(x_k1(0,0))||isnan(x_k1(1,0))){
                               return z_k;
                           }
                           else{
                                       P = (Eigen::Matrix2d::Identity() - K * H) * P;//计算后验均方差  P[n|n]=(1-K[n]*H)*P[n|n-1]
                                       return x_k1;
                           }
                      }
                 }
            }
        }
  }

};


#endif
