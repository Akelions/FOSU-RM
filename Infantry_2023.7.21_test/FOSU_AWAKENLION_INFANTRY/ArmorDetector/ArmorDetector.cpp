#include "ArmorDetector.h"
#include "inference_api2.hpp"

bool bestArmor(std::unordered_map<int,MultiValueMap> map, ArmorObject best_object)
{
    static ArmorObject* last_object=std::nullptr_t;
    int max_area=-1;
    ArmorObject* max_area_armor=std::nullptr_t;

    if(last_object==std::nullptr_t){
        for(auto object:objects){
            if(object.cls==last_object.cls){
                best_object=object;
                last_object=&object;
                max_area=-1;
                return 1;
            }
            else if(max_area<object.area){
                    max_area=object.area;
                    max_area_armor=&object;
                }
        }

        best_object=&max_area_armor;
        last_object=max_area_armor;
        return 1;
    }
    else{
        double min_dist=-99999.0;
        ArmorObject* min_dist_armor;
        for(auto object:objects){
            cv::Point2f armor_center=cv::Point2f((object.apex[0].x+object.apex[2].x)/2,(object.apex[0].y+object.apex[2].y)/2);
            double camera_point_dist=POINT_DIST(armor_center,CAMERA_CENTER);
            if(min_dist>camera_point_dist){
                min_dist=camera_point_dist;
                min_dist_armor=&object;
            }
        }

        best_object=&min_dist_armor;
        last_object=min_dist_armor;
        return 1;

    }
}

void detectorArmor(cv::Mat src,ArmorDetector ad, vector<ArmorObject> objects)
{
    //ad应在程序一开始运行是就初始化
    ad.detect(src,objects);
}
