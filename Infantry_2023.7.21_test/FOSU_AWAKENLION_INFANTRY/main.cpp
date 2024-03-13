#include <iostream>
#include <thread>
#include <opencv2/opencv.hpp>
#include <unistd.h>
#include <ctime>

#include "Settings/Settings.h"
#include "AngleSolver/PnpSolver.h"
#include "Camera/MVVideoCapture.h"
#include "Serial/PackData.h"
#include "Serial/Serial.h"
#include "ArmorDetector/ArmorDetector.h"
#include "RuneDetector/inference_api.hpp"
#include "predict/predict.h"

using namespace buff_detector;


void cameraThread(cv::Mat &src){

        std::cout << "MVVideoCapture Init" << std::endl;
        if(-1 == MVVideoCapture::Init())
        {
            std::cout << "MVVideoCapture ERROR!!!" << std::endl;
            return;
        }
        else{
            MVVideoCapture::Play();
            MVVideoCapture::SetExposureTime(false, (/*main_settings.debug.expore_time*/1200)*0.7);
            MVVideoCapture::SetLargeResolution(true);
            std::cout << "MVVideoCapture Finished!" << std::endl;

            while(1){
                MVVideoCapture::GetFrame(src);
                if(src.empty()){
                    std::cout<<"!!!!!!!!!!!!!!!!!!!!"<<std::endl;
                    continue;
                }
                cv::imshow("src",src);
            }
        }
}
void detectThread(MainSettings &main_settings,cv::Mat src,ArmorObject *armor_object,BuffObject &buff_object,AngleSolver &angle_solver,std::vector<ArmorObject> *cars_map,UnpackData *unpack_data,bool &flag){

    ArmorDetector ad;
    std::vector<ArmorObject> objects;
    std::vector<BuffObject> bobjects;
    ArmorObject* best_object;
    ad.initModel(MODEL_PATH);
    DetectorTool detector_tool;

    BuffDetector bd;
    bd.initModel(MODEL_PATH_BUFF);



    while(1){

        if(src.empty()){
            std::cout<<"src is empty"<<endl;
            continue;
        }

        //获取裁判系统敌方装甲板颜色
       if(unpack_data->getStm2PcMesg()->stm32_info_data.robot_color == 1)
           detector_tool.Blue_or_Red = 1;
       else if(unpack_data->getStm2PcMesg()->stm32_info_data.robot_color == 2)
           detector_tool.Blue_or_Red = 0;
       // 接收电控发送的模式切换(比赛前切记打开)
       if(!std::isnan(unpack_data->getStm2PcMesg()->stm32_info_data.task_mode))
           main_settings.main_mode = unpack_data->getStm2PcMesg()->stm32_info_data.task_mode;

       if(main_settings.main_mode == 0){
         ad.detect(src,objects);
         detector_tool=DetectorTool(objects);
         cars_map=detector_tool.cars_map;
         if(detector_tool.bestArmor(best_object)){
             armor_object=best_object;
         }
       }
       else if(main_settings.main_mode == 1){
        //rune_detector
           bd.detect(src,bobjects);
           for(auto fan:bobjects){
               if(fan.cls==0){
                   buff_object=fan;
                   break;
               }
           }
       }
       flag=true;

    }
}

void predictThread(MainSettings main_settings,AngleSolver angle_solver, double *cars_radio,ArmorObject* object_addr, BuffObject buff_object,
                   std::vector<ArmorObject> *cars_map,double running_time,
                   PackData *pack_data,UnpackData *unpack_data,Serial &serial,bool flag){
    Eigen::Vector3d moto_tvec;
    double moto_move_pitch;
    double moto_move_yaw;
    static double last_pitch=0;
    static double last_yaw=0;
    static int lost_flag=0;
    

    while(1){
        if(flag==false)
            continue;

        if(main_settings.main_mode == 0){
            double moto_pitch_angle=unpack_data->getStm2PcMesg()->stm32_info_data.robot_pitch;
            double moto_yaw_angle=unpack_data->getStm2PcMesg()->stm32_info_data.robot_yaw;
            double bullet_speed=unpack_data->getStm2PcMesg()->stm32_info_data.bullet_level;
            ArmorPredictTool armor_predict_tool(angle_solver,moto_pitch_angle,moto_yaw_angle, cars_radio,object_addr,cars_map,bullet_speed,running_time);
            if(armor_predict_tool.predictArmor()){
                lost_flag=0;
                angle_solver.Camera2Moto(moto_pitch_angle,moto_yaw_angle,armor_predict_tool.tvec_armor,moto_move_pitch,moto_move_yaw,bullet_speed,9.8);
                last_pitch=moto_move_pitch;
                last_yaw=moto_move_yaw;
                pack_data->setPc2StmMesg()->gimbal_control_data.aim_pitch=moto_move_pitch;
                pack_data->setPc2StmMesg()->gimbal_control_data.aim_yaw=moto_move_yaw;
                pack_data->setPc2StmMesg()->gimbal_control_data.mode_Union.info.visual_valid=1;
                pack_data->processing(&serial);
            }
            else if(lost_flag<=4){
                pack_data->setPc2StmMesg()->gimbal_control_data.aim_pitch=last_pitch;
                pack_data->setPc2StmMesg()->gimbal_control_data.aim_yaw=last_yaw;
                pack_data->setPc2StmMesg()->gimbal_control_data.mode_Union.info.visual_valid=1;
                pack_data->processing(&serial);
                lost_flag++;
            }
            else{
                pack_data->setPc2StmMesg()->gimbal_control_data.aim_pitch=0;
                pack_data->setPc2StmMesg()->gimbal_control_data.aim_yaw=0;
                pack_data->setPc2StmMesg()->gimbal_control_data.mode_Union.info.visual_valid=0;
                pack_data->processing(&serial);
            }
        }

        else if(main_settings.main_mode==1){
            double moto_pitch_angle=unpack_data->getStm2PcMesg()->stm32_info_data.robot_pitch;
            double moto_yaw_angle=unpack_data->getStm2PcMesg()->stm32_info_data.robot_yaw;
            double bullet_speed=unpack_data->getStm2PcMesg()->stm32_info_data.bullet_level;
            RunePredictTool rune_predict_tool(angle_solver,buff_object,moto_pitch_angle,moto_yaw_angle,bullet_speed,running_time);
            if(rune_predict_tool.setRuneCoordinary()){
                lost_flag=0;
                angle_solver.Camera2Moto(moto_pitch_angle,moto_yaw_angle,rune_predict_tool.cur_moto_tvec,moto_move_pitch,moto_move_yaw,bullet_speed,9.8);
                last_pitch=moto_move_pitch;
                last_yaw=moto_move_yaw;
                pack_data->setPc2StmMesg()->gimbal_control_data.aim_pitch=moto_move_pitch;
                pack_data->setPc2StmMesg()->gimbal_control_data.aim_yaw=moto_move_yaw;
                pack_data->setPc2StmMesg()->gimbal_control_data.mode_Union.info.visual_valid=1;
                pack_data->processing(&serial);
            }
            else if(lost_flag<=4){
                pack_data->setPc2StmMesg()->gimbal_control_data.aim_pitch=last_pitch;
                pack_data->setPc2StmMesg()->gimbal_control_data.aim_yaw=last_yaw;
                pack_data->setPc2StmMesg()->gimbal_control_data.mode_Union.info.visual_valid=1;
                pack_data->processing(&serial);
                lost_flag++;
            }
            else{
                pack_data->setPc2StmMesg()->gimbal_control_data.aim_pitch=0;
                pack_data->setPc2StmMesg()->gimbal_control_data.aim_yaw=0;
                pack_data->setPc2StmMesg()->gimbal_control_data.mode_Union.info.visual_valid=0;
                pack_data->processing(&serial);
            }
        }
    }

}







int main(){

    cv::Mat src;
    ArmorObject *armor_object;
    AngleSolver angle_solver(PARAM_CALIBRATION_752);
    std::vector<ArmorObject> cars_map[8];
    double cars_radio[8]={0,0,0,0,0,0,0};
    double running_time=2.0;

    BuffObject buff_object;

    MainSettings main_settings(PARAM_OTHER_PATH,PARAM_CAMERA_PATH);
    bool flag=false;

    PackData *pack_data;
    UnpackData *unpack_data;
    UnpackData *p_unpack_data = unpack_data;
    Serial serial(main_settings.port_param);

    std::thread ct(cameraThread,ref(src));
    //std::thread dt(detectThread,ref(main_settings),ref(src),ref(armor_object),ref(buff_object),ref(angle_solver),ref(cars_map),ref(unpack_data),ref(flag));
    //std::thread pt(predictThread,ref(main_settings),ref(angle_solver),ref(cars_radio),ref(armor_object),ref(buff_object),
    //               ref(cars_map),ref(running_time),
    //               ref(pack_data),ref(unpack_data),ref(serial),ref(flag));
    //std::thread st(&UnpackData::processing, p_unpack_data, &serial);


#ifdef DEBUG_MODE
    main_settings.setMainParam(WIN_OTHER);
    main_settings.setCameraParam(WIN_CAMERA);
#endif

      ct.join();
      //dt.join();
      //pt.join();
      //st.join();

//#ifdef DEBUG_MODE
//      while(1){
//        cv::Mat src_clone=src.clone();
//        if(main_settings.debug.src==1) cv::imshow("src",src);
//        if(main_settings.debug.detect_armor==1){
//          //添加画图处理
//        }
//        if(main_settings.debug.armor_chosen){
//          //添加画图处理
//        }
//        if(main_settings.debug.predict_point){
//          //
//        }
//        if(main_settings.debug.inaccuracy_point){

//        }
//        if(main_settings.debug.dc_pitch){

//        }
//        if(main_settings.debug.dc_yaw){

//        }
//        cv::waitKey(1);
//      }
//#endif
    return 0;
}
