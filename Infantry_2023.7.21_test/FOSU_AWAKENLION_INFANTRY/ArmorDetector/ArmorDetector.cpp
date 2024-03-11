#include "ArmorDetector.h"
#include "inference_api2.hpp"

DetectorTool::DetectorTool()
{
}

DetectorTool::DetectorTool(std::vector<ArmorObject> objects)
{
    this->objects=objects;
    std::vector<ArmorObject> same_ids[8];
    for(int i=0;i<8;i++)
    this->cars_map.insert(std::make_pair<int,std::vector<ArmorObject>>(i,));
    std::vector<std::vector<ArmorObject>> armors;
    for(auto obj:objects){
            if (Blue_or_Red == 1)
            {
                if (obj.color == BLUE_SMALL || obj.color == BLUE_BIG || obj.color == PURPLE_SMALL || obj.color == PURPLE_BIG)
                    continue;
            }
            else if (Blue_or_Red == 0)
            {
                if (obj.color == RED_SMALL || obj.color == RED_BIG || obj.color == PURPLE_SMALL || obj.color == PURPLE_BIG)
                    continue;
            }



    }
}



bool DetectorTool::bestArmor(ArmorObject *best_object)
{
    static ArmorObject* last_object=NULL;
    int max_area=-1;
    ArmorObject* max_area_armor=NULL;
    double min_dist=-99999.0;
    ArmorObject* min_dist_armor;

    if(last_object==NULL){

            for(auto car:this->cars_map){
                if(car.second.same_id_objects.size()>0){
                    for(auto object:car.second.same_id_objects){
                        cv::Point2f armor_center=cv::Point2f((object.apex[0].x+object.apex[2].x)/2,(object.apex[0].y+object.apex[2].y)/2);
                        double camera_point_dist=POINT_DIST(armor_center,CAMERA_CENTER);
                        if(min_dist>camera_point_dist){
                            min_dist=camera_point_dist;
                            min_dist_armor=&object;
                        }   
                    }
                }
            }

            best_object=min_dist_armor;
            last_object=min_dist_armor;
            return 1;
    }

    else{
        for(auto car:this->cars_map){
            if(/*car.first==&last_object.cls*/0){
                if(car.second.size()==1){
                    best_object=car.second.same_id_objects[0];
                    last_object=&best_object;
                    
                }
                else if(car.second.same_id_objects[0].area>car.second.same_id_objects[1].area){
                    best_object=car.second.same_id_objects[0];
                    last_object=&best_object;
                    
                }
                else{
                    best_object=car.second.same_id_objects[1];
                    last_object=&best_object;
                }
                return 1;
            }
            else{
                for(auto object:car.second.same_id_objects){
                    // if(max_area<object.area){
                    //     max_area=object.area;
                    //     max_area_armor=&object;
                    // }
                        cv::Point2f armor_center=cv::Point2f((object.apex[0].x+object.apex[2].x)/2,(object.apex[0].y+object.apex[2].y)/2);
                        double camera_point_dist=POINT_DIST(armor_center,CAMERA_CENTER);
                        if(min_dist>camera_point_dist){
                            min_dist=camera_point_dist;
                            min_dist_armor=&object;
                        }       
                }


            }

        }

        // best_object=max_area_armor;
        // last_object=max_area_armor;
        best_object=min_dist_armor;
        last_object=min_dist_armor;
        return 1;
    }
    
}


