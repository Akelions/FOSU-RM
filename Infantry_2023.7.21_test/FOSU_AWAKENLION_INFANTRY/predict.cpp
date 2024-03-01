#include "predict.h"

predict::predict(ArmorDetector ad, Eigen::Vector3d tvec, Eigen::Vector3d rvec, double *cars_radio,int object_cls)
{
    this->tvec1=tvec;
    this->rvec1=rvec;
    this->car_radio=*(cars_radio+object_cls);
    this->ad=ad;
}

bool predict::findSameCls()
{
    if(car_radio==0){






    }
}

bool predict::solveCarRadio()
{


}

bool predict::predictRotated()
{

}

bool predict::predictMove()
{

}

bool predict::stateAdd()
{

}

bool predict::predictArmor()
{

}
