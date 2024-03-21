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

cv::Mat src;
ArmorObject armor_object;
AngleSolver angle_solver(PARAM_CALIBRATION_752);
std::vector<ArmorObject> cars_map[8];
double cars_radio[8]={0,0,0,0,0,0,53.7,0};
double running_time=2.0;

BuffObject buff_object;

MainSettings main_settings(PARAM_OTHER_PATH,PARAM_CAMERA_PATH);
bool flag_detector=false;
bool flag_predictor=true;

PackData pack_data;
UnpackData unpack_data;
UnpackData *p_unpack_data = &unpack_data;
Serial serial(main_settings.port_param);




void cameraThread(cv::Mat *src){

//        std::cout << "MVVideoCapture Init" << std::endl;
//        if(-1 == MVVideoCapture::Init())
//        {
//            std::cout << "MVVideoCapture ERROR!!!" << std::endl;
//            return;
//        }
//        else{
//            MVVideoCapture::Play();
//            MVVideoCapture::SetExposureTime(true, (/*main_settings.debug.expore_time*/1200)*0.7);
//            MVVideoCapture::SetLargeResolution(true);
//            std::cout << "MVVideoCapture Finished!" << std::endl;

//            while(1){
//                MVVideoCapture::GetFrame(*src);
//                if((*src).empty()){
//                    std::cout<<"!!!!!!!!!!!!!!!!!!!!"<<std::endl;
//                }
//                else{
//                    cv::imshow("src",(*src));
//                    cv::waitKey(1);
//                }
//            }

//      }
}
void detectThread(MainSettings *main_settings,cv::Mat *src,ArmorObject *armor_object,BuffObject *buff_object,double *running_time,
                  AngleSolver *angle_solver,std::vector<ArmorObject> *cars_map,UnpackData *unpack_data,bool *flag1,bool *flag2){

    ArmorDetector ad;
    std::vector<ArmorObject> objects;
    std::vector<BuffObject> bobjects;
    ArmorObject best_object;
    ad.initModel(MODEL_PATH);
    DetectorTool detector_tool;

    BuffDetector bd;
    bd.initModel(MODEL_PATH_BUFF);



    while(1){
        MVVideoCapture::GetFrame(*src);
        if((*src).empty()){
            std::cout<<"!!!!!!!!!!!!!!!!!!!!"<<std::endl;
        }
        else{
            cv::resize(*src,*src,cv::Size(640,480));
            cv::imshow("src",(*src));
            cv::waitKey(1);
        }

        if((*src).empty()){
            std::cout<<"src is empty"<<endl;
            
            sleep(1);
            continue;
        }

        

       double time1=cv::getTickCount();
//        //获取裁判系统敌方装甲板颜色
//       if(unpack_data->getStm2PcMesg()->stm32_info_data.robot_color == 1){
//           (*main_settings).enenmy_color=red;
//       }
//       else if(unpack_data->getStm2PcMesg()->stm32_info_data.robot_color == 2){
//           (*main_settings).enemy_color=blue;
//       }
//       // 接收电控发送的模式切换(比赛前切记打开)
//       if(!std::isnan(unpack_data->getStm2PcMesg()->stm32_info_data.task_mode))
//           (*main_settings).main_mode = unpack_data->getStm2PcMesg()->stm32_info_data.task_mode;


       if((*main_settings).main_mode == 0){

         ad.detect((*src),objects);
         detector_tool=DetectorTool(objects,(bool)(*main_settings).enemy_color^1);
         //best_object=&objects[0];
         if(*flag2==false){
             *flag1=true;
             continue;
         }
         *flag1=false;

         for(int i=0;i<8;i++){
             cars_map[i]=detector_tool.cars_map[i];
         }


         if(detector_tool.bestArmor(&best_object)){

             if(best_object.color==1||best_object.color==3){

                 (*angle_solver).setTargetSize(23.1,5.7);//big
             }
             else{

                 (*angle_solver).setTargetSize(15.3,5.7);//small
             }
             *armor_object=best_object;

             double time2=cv::getTickCount();
             *running_time=(time2-time1)/cv::getTickFrequency();//s
             *flag1=true;
#ifdef DEBUG_MODE
       cv::Mat src_clone=(*src).clone();
       std::vector<cv::Scalar> color_list;
       color_list.push_back(cv::Scalar(255,0,0));
       color_list.push_back(cv::Scalar(0,255,0));
       color_list.push_back(cv::Scalar(0,0,255));
       color_list.push_back(cv::Scalar(255,255,0));
       if(&best_object!=NULL)
       for(int i=0;i<4;i++){
           cv::line(src_clone,best_object.apex[i%4],best_object.apex[(i+1)%4],color_list[i],2);
           std::stringstream s;
           std::stringstream d;

           s << best_object.color;
           d << best_object.cls;
           cv::putText(src_clone, "color_size: " + s.str(), best_object.apex[2], cv::FONT_HERSHEY_COMPLEX, 0.3, cv::Scalar(0, 255, 0));
           cv::putText(src_clone, "class_num: " + d.str(), best_object.apex[3], cv::FONT_HERSHEY_COMPLEX, 0.3, cv::Scalar(0, 255, 0));
        }

       std::stringstream time;
       time<<1.0/(*running_time);
       cv::putText(src_clone, "fps: " + time.str(), cv::Point(50,50), cv::FONT_HERSHEY_COMPLEX, 0.9, cv::Scalar(0, 255, 0));
       cv::imshow("best_armor",src_clone);
       cv::waitKey(1);
#endif
         }



       }
       else if((*main_settings).main_mode == 1){
        //rune_detector
           bd.detect((*src),bobjects);
           double time2=cv::getTickCount();
           *running_time=(time2-time1)/cv::getTickFrequency();//s
           for(auto fan:bobjects){
               if(fan.cls==0){
                   *buff_object=fan;
                   *flag1=true;
                   break;
               }
           }
       }
     }
}

void predictThread(MainSettings *main_settings,AngleSolver *angle_solver, double *cars_radio,ArmorObject *object_addr, BuffObject *buff_object,
                   std::vector<ArmorObject> *cars_map,double *running_time,
                   PackData *pack_data,UnpackData *unpack_data,Serial *serial,bool *flag1,bool *flag2){
    Eigen::Vector3d moto_tvec;
    double moto_move_pitch;
    double moto_move_yaw;
    static double last_pitch=0;
    static double last_yaw=0;
    static int lost_flag=0;
    static double during_time=2.0;
    double predict_time=during_time+*running_time;//s
    static ArmorPredictTool armor_predict_tool;
    armor_predict_tool.kalmanInit();

    while(1){

        if(*flag1==false){
            //sleep(1);
            *flag2=true;
            continue;
        }
        *flag2=false;


        double time1=cv::getTickCount();
        if((*main_settings).main_mode == 0){
            if(cars_radio==NULL&&object_addr==NULL&&cars_map==NULL&&running_time==NULL){
                continue;
            }

            double moto_pitch_angle=unpack_data->getStm2PcMesg()->stm32_info_data.robot_pitch;
            double moto_yaw_angle=unpack_data->getStm2PcMesg()->stm32_info_data.robot_yaw;
//            double moto_pitch_angle=20.0;
//            double moto_yaw_angle=0.0;
            double bullet_speed=unpack_data->getStm2PcMesg()->stm32_info_data.bullet_level;

            armor_predict_tool.inputData(*angle_solver,moto_pitch_angle,moto_yaw_angle, cars_radio,object_addr,cars_map,bullet_speed,predict_time);

            if(armor_predict_tool.predictArmor()){

                lost_flag=0;
                (*angle_solver).Camera2Moto(moto_pitch_angle,moto_yaw_angle,armor_predict_tool.tvec_armor,moto_move_pitch,moto_move_yaw,bullet_speed,9.8);
                last_pitch=moto_move_pitch;
                last_yaw=moto_move_yaw;
                pack_data->setPc2StmMesg()->gimbal_control_data.aim_pitch=moto_move_pitch;
                pack_data->setPc2StmMesg()->gimbal_control_data.aim_yaw=moto_move_yaw;
                pack_data->setPc2StmMesg()->gimbal_control_data.mode_Union.info.visual_valid=1;
                pack_data->process(serial);
                //std::cout<<moto_move_pitch<<std::endl;
            }
            else if(lost_flag<=4){
                pack_data->setPc2StmMesg()->gimbal_control_data.aim_pitch=last_pitch;
                pack_data->setPc2StmMesg()->gimbal_control_data.aim_yaw=last_yaw;
                pack_data->setPc2StmMesg()->gimbal_control_data.mode_Union.info.visual_valid=1;
                pack_data->process(serial);
                lost_flag++;
            }
            else{
                pack_data->setPc2StmMesg()->gimbal_control_data.aim_pitch=0;
                pack_data->setPc2StmMesg()->gimbal_control_data.aim_yaw=0;
                pack_data->setPc2StmMesg()->gimbal_control_data.mode_Union.info.visual_valid=0;
                pack_data->process(serial);
            }
        }

//        else if((*main_settings).main_mode==1){
//            double moto_pitch_angle=unpack_data->getStm2PcMesg()->stm32_info_data.robot_pitch;
//            double moto_yaw_angle=unpack_data->getStm2PcMesg()->stm32_info_data.robot_yaw;
//            double bullet_speed=unpack_data->getStm2PcMesg()->stm32_info_data.bullet_level;
//            RunePredictTool rune_predict_tool(*angle_solver,*buff_object,moto_pitch_angle,moto_yaw_angle,bullet_speed,predict_time);
//            if(rune_predict_tool.setRuneCoordinary()){
//                lost_flag=0;
//                (*angle_solver).Camera2Moto(moto_pitch_angle,moto_yaw_angle,rune_predict_tool.cur_moto_tvec,moto_move_pitch,moto_move_yaw,bullet_speed,9.8);
//                last_pitch=moto_move_pitch;
//                last_yaw=moto_move_yaw;
//                pack_data->setPc2StmMesg()->gimbal_control_data.aim_pitch=moto_move_pitch;
//                pack_data->setPc2StmMesg()->gimbal_control_data.aim_yaw=moto_move_yaw;
//                pack_data->setPc2StmMesg()->gimbal_control_data.mode_Union.info.visual_valid=1;
//                pack_data->process(serial);
//            }
//            else if(lost_flag<=4){
//                pack_data->setPc2StmMesg()->gimbal_control_data.aim_pitch=last_pitch;
//                pack_data->setPc2StmMesg()->gimbal_control_data.aim_yaw=last_yaw;
//                pack_data->setPc2StmMesg()->gimbal_control_data.mode_Union.info.visual_valid=1;
//                pack_data->process(serial);
//                lost_flag++;
//            }
//            else{
//                pack_data->setPc2StmMesg()->gimbal_control_data.aim_pitch=0;
//                pack_data->setPc2StmMesg()->gimbal_control_data.aim_yaw=0;
//                pack_data->setPc2StmMesg()->gimbal_control_data.mode_Union.info.visual_valid=0;
//                pack_data->process(serial);
//            }
//        }
        double time2=cv::getTickCount();
        during_time=(time2-time1)/cv::getTickFrequency();//s
        *flag2=true;
    }

}







int main(){

    serial.openPort("/dev/ttyACM0");
    std::cout << "MVVideoCapture Init" << std::endl;
    if(-1 == MVVideoCapture::Init())
    {
        std::cout << "MVVideoCapture ERROR!!!" << std::endl;
        return 1;
    }
    else{
        MVVideoCapture::Play();
        MVVideoCapture::SetExposureTime(false, (/*main_settings.debug.expore_time*/10000));//bu yao zi dong tiao bao guang!!!
        MVVideoCapture::SetLargeResolution(true);
        std::cout << "MVVideoCapture Finished!" << std::endl;
    }


#ifdef DEBUG_MODE
    main_settings.setMainParam(WIN_OTHER);
    //main_settings.setCameraParam(WIN_CAMERA);
#endif

    std::thread dt(detectThread,&main_settings,&src,&armor_object,&buff_object,&running_time,&angle_solver,cars_map,&unpack_data,&flag_detector,&flag_predictor);
    std::thread st(&UnpackData::processing, p_unpack_data, &serial);
    dt.detach();
    st.detach();
    predictThread(&main_settings,&angle_solver,cars_radio,&armor_object,&buff_object,cars_map,&running_time,&pack_data,&unpack_data,&serial,&flag_detector,&flag_predictor);
#ifdef DEBUG_MODE
//      while(1){
//        cv::Mat src_clone=src.clone();
//        //if(main_settings.debug.src==1) cv::imshow("src",src);
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
//      }
#endif
    //serial.closeDevice();
    return 0;
}
