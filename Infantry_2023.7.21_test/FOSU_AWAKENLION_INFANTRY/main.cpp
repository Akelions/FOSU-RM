#include <iostream>
#include <thread>
#include <opencv2/opencv.hpp>
#include <QFile>
#include <QDir>
#include <unistd.h>
#include <ctime>

#include "Settings/Settings.h"
#include "AngleSolver/PnpSolver.h"
#include "Camera/MVVideoCapture.h"
#include "Serial/PackData.h"
#include "Serial/Serial.h"
#include "ArmorDetector/ArmorDetector.h"
#include "RuneDetector/RuneDetector.h"
#include "Gui/Gui.h"

cv::Mat src;
ArmorObject armor_object;
AngleSolver angle_solver;
std::unordered_map<int,MultiValueMap> cars_map;
// rune_object;
double cars_radio[6]={0,0,0,0,0,0};


PackData pack_data;
UnpackData unpack_data;
UnpackData *p_unpack_data = &unpack_data;
Serial serial(main_setting.port_param);



MainSettings main_settings(PARAM_OTHER_PATH,PARAM_CAMERA_PATH);

void cameraThread(cv::Mat &src){

        std::cout << "MVVideoCapture Init" << std::endl;
        if(-1 == MVVideoCapture::Init())
        {
            std::cout << "MVVideoCapture ERROR!!!" << std::endl;
            return;
        }
        else{
            MVVideoCapture::Play();
            MVVideoCapture::SetExposureTime(false, (main_setting->debug.expore_time)*0.7);
            MVVideoCapture::SetLargeResolution(true);
            std::cout << "MVVideoCapture Finished!" << std::endl;

            while(1){
                MVVideoCapture::GetFrame(src);
            }
        }
}
void detectThread(cv::Mat src,ArmorObject &object,AngleSolver &angle_solver){
    ArmorDetector ad;
    vector<ArmorObject> objects;
    ArmorObject best_objects;
    ad.initModel(MODEL_PATH);
    while(1){
        ad.detect(src,objects);
        if(bestArmor(objects,best_objects)){
            object=best_objects;
        }
    }
}

void predictThread(ArmorObject object,AngleSolver angle_solver,PackData &pack_data,UnpackData &unpack_data,Serial &serial){
    Eigen::Vector3d tvec;
    Eigen::Vector3d moto_tvec;
    Eigen::Vector3d rvec;
    double moto_move_pitch;
    double moto_move_yaw;
    while(1){
        double moto_pitch_angle=unpack_data->getStm2PcMesg()->stm32_info_data.robot_pitch;
        double moto_yaw_angle=unpack_data->getStm2PcMesg()->stm32_info_data.robot_yaw;
        double bullet_speed=unpack_data.getStm2PcMesg()->stm32_info_data.bullet_level;
        angle_solver.getAngle(object.pts,tvec,rvec);
        //predict//
        angle_solver.coordinary_transformation(moto_pitch_angle,moto_yaw_angle,tvec,moto_tvec);
        angle_solver.Camera2Moto(moto_pitch_angle,moto_yaw_angle,moto_tvec,moto_move_pitch,moto_move_yaw,bullet_speed,9.8);

        pack_data.setPc2StmMesg()->gimbal_control_data.aim_pitch=moto_move_pitch;
        pack_data.setPc2StmMesg()->gimbal_control_data.aim_yaw=moto_move_yaw;
        pack_data.setPc2StmMesg()->gimbal_control_data.mode_Union.info.visual_valid=1;
        pack_data.processing(serial);
    }

}


std::thread ct(&cameraThread(src));
std::thread dt;
std::thread pt;
std::thread st;



int main(){

    if(main_settings.main_mode==armor_mode){
        dt(&detectThread(src,armor_object,angle_solver));
        pt(&predictThread(armor_object,angle_solver,pack_data,unpack_data,serial));
    }
    st(&UnpackData::processing, p_unpack_data, &serial);

#ifdef DEBUG_MODE
    main_settings.setMainParam(WIN_OTHER);
    main_settings.setCameraParam(WIN_CAMERA);
#endif
      ct.join();
      dt.join();
      pt.join();
      st.join();

#ifdef DEBUG_MODE
      while(1){
        cv::Mat src_clone=src.clone();
        if(main_settings.debug.src==1) cv::imshow("src",src);
        if(main_settings.debug.detect_armor==1){
          //添加画图处理
        }
        if(main_settings.debug.armor_chosen){
          //添加画图处理
        }
        if(main_settings.debug.predict_point){
          //
        }
        if(main_settings.debug.inaccuracy_point){

        }
        if(main_settings.debug.dc_pitch){

        }
        if(main_settings.debug.dc_yaw){

        }
      }
#endif
    return 0;
}
