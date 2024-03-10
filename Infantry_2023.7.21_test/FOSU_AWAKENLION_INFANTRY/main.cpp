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
#include "predict/predict.h"

cv::Mat src;
ArmorObject *armor_object;
AngleSolver angle_solver;
std::unordered_map<int,MultiValueMap> cars_map;
double cars_radio[7]={0,0,0,0,0,0,0};

BuffObject buff_object;


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
void detectThread(MainSettings &main_settings,cv::Mat src,ArmorObject *armor_object,BuffObject &buff_object,AngleSolver &angle_solver,std::unordered_map<int,MultiValueMap> &cars_map,UnpackData &unpack_data){

    ArmorDetector ad;
    std::vector<ArmorObject> objects;
    ArmorObject* best_object;
    ad.initModel(MODEL_PATH);
    DetectorTool detector_tool();

    BuffDetector bd;
    bd.initModel(MODEL_PATH_BUFF);



    while(1){
        //获取裁判系统敌方装甲板颜色
       if(unpack_data->getStm2PcMesg()->stm32_info_data.enemy_color == 1)
           detector_tool.Blue_or_Red = 1;
       else if(unpack_data->getStm2PcMesg()->stm32_info_data.enemy_color == 2)
           detector_tool.Blue_or_Red = 0;
       // 接收电控发送的模式切换(比赛前切记打开)
       if(!std::isnan(unpack_data->getStm2PcMesg()->stm32_info_data.task_mode))
           main_settings->main_mode = unpack_data->getStm2PcMesg()->stm32_info_data.task_mode;

       if(main_settings->main_mode == 0){
         ad.detect(src,objects);
         detector_tool(objects);
         cars_map=detector_tool.cars_map;
         if(detector_tool.bestArmor(best_objects)){
             object=best_objects;
         }
       }
       else if(main_settings->main_mode == 1){
        //rune_detector
           bd.detect(src,buff_object);
       }

    }
}

void predictThread(MainSettings main_settings,AngleSolver angle_solver, double *cars_radio,ArmorObject* object_addr, BuffObject buff_object,
                   std::unordered_map<int,MultipleValue>cars_map,double running_time,
                   PackData &pack_data,UnpackData &unpack_data,Serial &serial){
    Eigen::Vector3d moto_tvec;
    double moto_move_pitch;
    double moto_move_yaw;
    static double last_pitch=0;
    static double last_yaw=0;
    static int lost_flag=0;
    

    while(1){
        if(main_settings.main_mode == 0){
            double moto_pitch_angle=unpack_data->getStm2PcMesg()->stm32_info_data.robot_pitch;
            double moto_yaw_angle=unpack_data->getStm2PcMesg()->stm32_info_data.robot_yaw;
            double bullet_speed=unpack_data.getStm2PcMesg()->stm32_info_data.bullet_level;
            ArmorPredictTool armor_predict_tool(angle_solver,moto_pitch,moto_yaw, cars_radio,object_addr,cars_map,bullet_speed,running_time);
            if(armor_predict_tool.predictArmor()){
                lost_flag=0;
                angle_solver.Camera2Moto(moto_pitch_angle,moto_yaw_angle,predict_tool.tvec_armor,moto_move_pitch,moto_move_yaw,bullet_speed,9.8);
                last_pitch=moto_move_pitch;
                last_yaw=moto_move_yaw;
                pack_data.setPc2StmMesg()->gimbal_control_data.aim_pitch=moto_move_pitch;
                pack_data.setPc2StmMesg()->gimbal_control_data.aim_yaw=moto_move_yaw;
                pack_data.setPc2StmMesg()->gimbal_control_data.mode_Union.info.visual_valid=1;
                pack_data.processing(serial);
            }
            else if(lost_flag<=4){
                pack_data.setPc2StmMesg()->gimbal_control_data.aim_pitch=last_pitch;
                pack_data.setPc2StmMesg()->gimbal_control_data.aim_yaw=last_yaw;
                pack_data.setPc2StmMesg()->gimbal_control_data.mode_Union.info.visual_valid=1;
                pack_data.processing(serial);
                lost_flag++;
            }
            else{
                pack_data.setPc2StmMesg()->gimbal_control_data.aim_pitch=0;
                pack_data.setPc2StmMesg()->gimbal_control_data.aim_yaw=0;
                pack_data.setPc2StmMesg()->gimbal_control_data.mode_Union.info.visual_valid=0;
                pack_data.processing(serial);
            }
        }

        else if(main_settings.main_mode==1){
            double moto_pitch_angle=unpack_data->getStm2PcMesg()->stm32_info_data.robot_pitch;
            double moto_yaw_angle=unpack_data->getStm2PcMesg()->stm32_info_data.robot_yaw;
            double bullet_speed=unpack_data.getStm2PcMesg()->stm32_info_data.bullet_level;
            RunePredictTool rune_predict_tool(angle_solver,moto_pitch_angle,moto_yaw_angle,bullet_speed,running_time);
            if(rune_predict_tool.setRuneCoordinary()){
                lost_flag=0;
                angle_solver.Camera2Moto(moto_pitch_angle,moto_yaw_angle,rune_predict_tool.cur_moto_tvec,moto_move_pitch,moto_move_yaw,bullet_speed,9.8);
                last_pitch=moto_move_pitch;
                last_yaw=moto_move_yaw;
                pack_data.setPc2StmMesg()->gimbal_control_data.aim_pitch=moto_move_pitch;
                pack_data.setPc2StmMesg()->gimbal_control_data.aim_yaw=moto_move_yaw;
                pack_data.setPc2StmMesg()->gimbal_control_data.mode_Union.info.visual_valid=1;
                pack_data.processing(serial); 
            }
            else if(lost_flag<=4){
                pack_data.setPc2StmMesg()->gimbal_control_data.aim_pitch=last_pitch;
                pack_data.setPc2StmMesg()->gimbal_control_data.aim_yaw=last_yaw;
                pack_data.setPc2StmMesg()->gimbal_control_data.mode_Union.info.visual_valid=1;
                pack_data.processing(serial);
                lost_flag++;
            }
            else{
                pack_data.setPc2StmMesg()->gimbal_control_data.aim_pitch=0;
                pack_data.setPc2StmMesg()->gimbal_control_data.aim_yaw=0;
                pack_data.setPc2StmMesg()->gimbal_control_data.mode_Union.info.visual_valid=0;
                pack_data.processing(serial);
            }
        }
    }

}


std::thread ct(cameraThread,&src);
std::thread dt;
std::thread pt;
std::thread st;

dt(detectThread,main_settings,src,armor_object,buff_object,angle_solver,cars_map,unpack_data);
pt(PredictThread,main_settings,angle_solver,cars_radio,armor_object,buff_object,
    cars_map,running_time,
    pack_data,unpack_data,serial);
st(&UnpackData::processing, p_unpack_data, &serial);



int main(){


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
