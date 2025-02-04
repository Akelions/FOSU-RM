#ifndef ARMORDETECTOR_H
#define ARMORDETECTOR_H

#include "../Settings/Settings.h"
#include "../AngleSolver/PnpSolver.h"
#include "../Serial/PackData.h"
#include "../kalman/armor_kalman.h"
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

class DetectorTool {
public:
    DetectorTool();
    DetectorTool(std::vector<ArmorObject> objects,bool color_num);

    bool bestArmor(ArmorObject *best_object);
    void setColor(int color_num);
    void detectorArmor(vector<ArmorObject> objects);

    std::vector<ArmorObject> objects;
    std::vector<ArmorObject> cars_map[8];
    bool Blue_or_Red=1;



};









#endif // ARMORDETECTOR_H
