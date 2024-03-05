#ifndef ARMORDETECTOR_H
#define ARMORDETECTOR_H

#include "Settings/Settings.h"
#include "Preprocessing/Preprocessing.h"
#include "TargetDetection.h"
#include "AngleSolver/PnpSolver.h"
#include "Serial/PackData.h"
#include "AngleSolver/GravityCompensateResolve.h"
#include "svm/svm.h"
#include "kalman/armor_kalman.h"
#include "inference_api2.hpp"
#define POINT_DIST(p1, p2) std::sqrt((p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y-p2.y))
#define CAMERA_CENTER cv::Point2f(SHOW_WIDTH/2,SHOW_WIDTH/2)
#define GET_DIST(a,b) std::abs(a-b)/(a>b?a:b)

using namespace armor_detector;


    enum ArmorType
    {
        BLUE_SMALL,
        BLUE_BIG,
        RED_SMALL,
        RED_BIG,
        GRAY_SMALL,
        GRAY_BIG,
        PURPLE_SMALL,
        PURPLE_BIG
    };
    
    struct MultiValueMap {
        int cls;
        std::vector<ArmorObject> same_id_objects;
    };

class DetectorTool {
public:
    DetectorTool();
    DetectorTool(std::vector<ArmorObject> objects);

    bool bestArmor(std::vector<ArmorObject> objects, ArmorObject best_object);
    void detectorArmor(vector<ArmorObject> objects);

    std::vector<ArmorObject> objects;
    std::unordered_map<int,MultiValueMap> cars_map;
    bool Blue_or_Red=0;



};









#endif // ARMORDETECTOR_H
