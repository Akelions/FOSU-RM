#include "RuneDetector/RuneDetector.h"
#include "RuneDetector.h"

RuneHitLogic::RuneHitLogic(RuneSettings *rune_settings, MainSettings *main_settings, bool is_clockwise):
                           RuneDetector(rune_settings, main_settings, is_clockwise)
{
    T = (rune_settings->rune_param.rune_time_delay.TRIG2SHOOT2HIT_DELAY + rune_settings->rune_param.rune_time_delay.PTZ_SHIFT_DELAY) * 0.01;
    frame_count = 0;
}

RuneDetector::RuneDetector(RuneSettings *rune_settings, MainSettings *main_settings, bool is_clockwise):
                           DetectRuneCenter(rune_settings, main_settings),
                           prep_activate_detection(rune_settings->rune_param.tgt_size_param_u, main_settings->debug),
                           prep_activate_adaptive_threshold(rune_settings->preprocess_param)
{
    this->is_clockwise = is_clockwise;
}

DetectRuneCenter::DetectRuneCenter(RuneSettings *rune_settings, MainSettings *main_settings):
                                   center_detection(rune_settings->rune_param.tgt_size_param_c, main_settings->debug),
                                   center_adaptive_threshold(rune_settings->preprocess_param)
{
    this->rune_settings = rune_settings;
    this->main_settings = main_settings;
}

cv::Point2f DetectRuneCenter::getRuneCenter(void)
{
    return this->rune_center;
}

float RuneHitLogic::getPitAbsAngle()
{
    return pit_abs_angle;
}

float RuneHitLogic::getYawAbsAngle()
{
    return yaw_abs_angle;
}

bool RuneHitLogic::getRotation()
{
    return is_clockwise;
}

bool DetectRuneCenter::setRuneCenter(cv::Mat src, cv::RotatedRect rune_armor_corner)
{
    cv::Mat dst;
    std::vector<std::vector<cv::Point>>contours;
    std::vector<TargetSize> target_size;

    // 预处理
    redOrBlueTre(src, dst, center_adaptive_threshold);
    //center_adaptive_threshold.filter(dst);

#ifdef DEBUG_MODE
    cv::Mat src_clone = src.clone();
    center_detection.ret_4 = src.clone();
#endif // DEBUG_MODE

    // 将滑动条设置为tgt_size_param_c，否则与tgt_size_param_u冲突
    center_detection.setRuneTargetSize(rune_settings->rune_param.tgt_size_param_c);
    center_detection.targetDetectionProc(dst, contours, target_size,"center_rect");
    //center_detection.setRuneTargetSize(rune_settings->rune_param.tgt_size_param_u);

#ifdef DEBUG_MODE
    if(main_settings->debug.b_show_center_rune)
    {
        cv::namedWindow("debug_center_rune");
        cv::imshow("debug_center_rune", center_detection.ret_4);
    }
    else
    {
        if(-1 != cv::getWindowProperty("debug_center_rune", 1))
            cv::destroyWindow("debug_center_rune");
    }
#endif // DEBUG_MODE

    // 使用勾股定理
    int center_distance=0.0;
    for(int i = 0; i < center_detection.getTargetRotRect().size(); ++i)
    {
        std::cout<<"RuneCenter's amount:"<<center_detection.getTargetRotRect().size()<<std::endl;
#ifdef DEBUG_MODE
        cv::circle(src_clone, cv::Point(center_detection.getTargetRotRect()[i].center), 1, cv::Scalar(0, 255, 0), 2);
        cv::circle(src_clone, cv::Point(rune_armor_corner.center), 1, cv::Scalar(0, 255, 0), 2);
#endif // DEBUG_MODE
        int delta_x = rune_armor_corner.center.x - center_detection.getTargetRotRect()[i].center.x;
        int delta_y = rune_armor_corner.center.y - center_detection.getTargetRotRect()[i].center.y;
#ifdef DEBUG_MODE
        int add_x = rune_armor_corner.center.x + center_detection.getTargetRotRect()[i].center.x;
        int add_y = rune_armor_corner.center.y + center_detection.getTargetRotRect()[i].center.y;
#endif // DEBUG_MODE
        center_distance = abs(sqrt(delta_x * delta_x + delta_y * delta_y));

        if(center_distance < rune_settings->rune_param.tgt_size_param_c.center_distance_min || center_distance > rune_settings->rune_param.tgt_size_param_c.center_distance_max)
        {
#ifdef DEBUG_MODE
        std::ostringstream s_center_distance;
        s_center_distance << center_distance;
        cv::putText(src_clone, "distance: " + s_center_distance.str(), cv::Point(add_x / 2, add_y / 2) + cv::Point(0, 10), cv::FONT_HERSHEY_COMPLEX, 0.3, cv::Scalar(0, 0, 255));
        cv::line(src_clone, rune_armor_corner.center, center_detection.getTargetRotRect()[i].center, cv::Scalar(0, 0, 255));
#endif // DEBUG_MODE
        continue;
        }
#ifdef DEBUG_MODE
        std::ostringstream s_center_distance;
        s_center_distance << center_distance;
        cv::putText(src_clone, "distance: " + s_center_distance.str(), cv::Point(add_x / 2, add_y / 2) + cv::Point(0, 10), cv::FONT_HERSHEY_COMPLEX, 0.3, cv::Scalar(0, 255, 0));
        cv::line(src_clone, rune_armor_corner.center, center_detection.getTargetRotRect()[i].center, cv::Scalar(0, 255, 0));
#endif // DEBUG_MODE
        rune_center = center_detection.getTargetRotRect()[i].center;
    }
#ifdef DEBUG_MODE
    if(main_settings->debug.b_show_rune_center)
    {
        cv::namedWindow("debug_rune_center_distance");
        cv::imshow("debug_rune_center_distance", src_clone);
    }
    else
    {
        if(-1!=cv::getWindowProperty("debug_rune_center_distance", 1))
            cv::destroyWindow("debug_rune_center_distance");
    }
#endif // DEBUG_MODE
    std::cout<<"RuneCenter's x:"<<rune_center.x<<std::endl;
    std::cout<<"Center distsance: "<<center_distance<<std::endl;
    return rune_center.x;  // 调试距离时将此改成false
//    return false;
}


void DetectRuneCenter::redOrBlueTre(cv::Mat src, cv::Mat &dst, AdaptiveThreshold &threshold)
{
    struct PreprocessInfo preprocess_info;
    if(main_settings->debug.b_thre_mode_rune == 0){
        if(this->main_settings->enemy_color == red){
            threshold.mulAndIterBlueTre(src, dst, preprocess_info);
        }
        else{
            threshold.mulAndIterRedTre(src, dst, preprocess_info);
        }
    }
    if(main_settings->debug.b_thre_mode_rune == 1){
        if(this->main_settings->enemy_color == red){
            threshold.channelSubBlueTre(src, dst, preprocess_info);
        }
        else{
            threshold.channelSubRedTre(src, dst, preprocess_info);
        }
        cv::Mat kernel=cv::getStructuringElement(cv::MORPH_RECT,cv::Size(3,3));
        cv::dilate(dst,dst,kernel);
    }
}

double DetectRuneCenter::xzb(cv::Point2f center, cv::Point2f point) {
    double xz = (point.x - center.x);
    return xz;
}
double DetectRuneCenter::yzb(cv::Point2f center, cv::Point2f point) {
    double yz = -(point.y - center.y);
    return yz;
}
double DetectRuneCenter::dianju(cv::Point2f point1, cv::Point2f point2) {
    double dfang = pow(point1.x - point2.x, 2) + pow(point1.y - point2.y, 2);
    double d = sqrt(dfang);
    return d;
}







std::vector<cv::RotatedRect> RuneDetector::getPrepTargetRotRect()
{
    return this->prep_activate_detection.getTargetRotRect();
}

bool RuneDetector::getCurPos(cv::Mat src, cv::RotatedRect &rot_rect, QuadrilateralPos &cur_pos)
{
    bool is_find_curpos = false;                       // 是否找到装甲板
    cv::Mat dst;                                       // 二值图
    std::vector<std::vector<cv::Point>> contours;      // 轮廓
    std::vector<TargetSize> target_size;               // 目标尺寸

    // 预处理（分非过曝光情况与过曝光情况）
    redOrBlueTre(src, dst, prep_activate_adaptive_threshold);

    //prep_activate_adaptive_threshold.filter(dst);//consider use or not of situation

#ifdef DEBUG_MODE
    cv::Mat src_clone = src.clone();
    cv::Mat rune_cp_result = src.clone();
    if(main_settings->debug.b_show_bin_rune)
    {
        cv::namedWindow("bin");
        cv::imshow("bin", dst);
    }
    else
    {
        if(-1 != cv::getWindowProperty("bin",1))
            cv::destroyWindow("bin");
    }
#endif // DEBUG_MODE

    // 寻找目标装甲板（第一次装甲板筛选）
    prep_activate_detection.targetDetectionProc(dst, contours, target_size,"shanye");

    if(target_size.size() == 1){
#ifdef DEBUG_MODE

       cv::drawContours(rune_cp_result,contours,0,cv::Scalar(0,255,255));

#endif
        is_find_curpos = true;
    }

#ifdef DEBUG_MODE
    else if(target_size.size()>1){
        for(int i=0;i<contours.size();++i){
            cv::drawContours(src_clone,contours,i,cv::Scalar(0,255,255));
        }
    }
    else std::cout<<"No anything meets requirements!"<<std::endl;

#endif

    if (is_find_curpos)
    {
        rot_rect = prep_activate_detection.getTargetRotRect()[0];//rot_rect_temp;
        prep_activate_detection.getTargetRotRect()[0].points(cur_pos.p);
#ifdef DEBUG_MODE
          if(main_settings->debug.b_show_cp_rune)
            {
                cv::namedWindow("debug_cp_rune");
                cv::imshow("debug_cp_rune", src_clone);
            }
            else
            {
                if(-1 != cv::getWindowProperty("debug_cp_rune", 1))
                    cv::destroyWindow("debug_cp_rune");
            }
            if(main_settings->debug.b_show_cp_rune_result)
            {
                cv::namedWindow("cp_rune_result");
                cv::imshow("cp_rune_result", rune_cp_result);
            }
            else
            {
                if(-1 != cv::getWindowProperty("cp_rune_result", 1))
                    cv::destroyWindow("cp_rune_result");
            }
#endif // DEBUG_MODE
        return is_find_curpos;

    }
    else
    {
        std::cout<<"Didn't find rune fan we should attack."<<std::endl;
        return false;
    }
}


bool RuneDetector::getRealCurPos(cv::Mat src,cv::RotatedRect &rot_rect,QuadrilateralPos &cur_pos,cv::Point2f rune_center,cv::Point2d &cur_pos_center){
    //因为getcurpos获得的是扇叶，所以此处转成装甲板，按比例点出装甲板中心点
        cv::Point2f temp_center = rot_rect.center;
//#ifdef DEBUG_MODE
        //cv::Point2f debug_point=rot_rect.center;
//#endif

        double k = yzb(rune_center, temp_center) / xzb(rune_center, temp_center);
        double dist = dianju(rune_center, temp_center);


//        if (std::abs(k) < 0.00001)  k = 0.00;
//        if (std::abs(k) > 10000.00 && k > 0) k = 10000.00;
//        if (std::abs(k) > 10000.00 && k < 0) k = -10000.00;

        double arfa = std::atan(k);
        if(arfa<0) arfa+=6.2830;

        double cosk=cos(arfa);
        double sink=sin(arfa);



        std::cout<<"rune_center "<<rune_center<<std::endl;
        std::cout<<"temp_center "<<temp_center<<std::endl;
//        std::cout<<sink<<std::endl;
//        std::cout<<cosk<<std::endl;
        std::cout<<arfa<<std::endl;



        if (k < 0 && yzb(rune_center, temp_center) > 0) {//此处的700/511为扇叶中心与旋转中心的距离/靶心中心与旋转中心的距离

            arfa-=3.1415;
            cosk=cos(arfa);
            sink=sin(arfa);

            temp_center.x = cosk * dist / 511*900 + rune_center.x;
            temp_center.y = -sink * dist / 511*900 + rune_center.y;
        }
        else if (k > 0 && yzb(rune_center, temp_center) < 0){

            arfa+=3.1415;
            cosk=cos(arfa);
            sink=sin(arfa);

            temp_center.x = cosk * dist / 511*900 + rune_center.x;
            temp_center.y = -sink * dist / 511*900 + rune_center.y;
        }
        else{

            temp_center.x = cosk * dist / 511*900 + rune_center.x;
            temp_center.y = -sink * dist / 511*900 + rune_center.y;
        }



        cur_pos_center=temp_center;
        cv::RotatedRect temp_rot_rect(temp_center,cv::Size(362*dist/511,372*dist/511),-arfa/3.1415*180);
        rot_rect=temp_rot_rect;
        cv::Point2f points[4];
        temp_rot_rect.points(points);
        for(int i=0;i<4;++i)
            cur_pos.p[i]=points[i];
//#ifdef DEBUG
//        cv::Mat real_cur_pos=src.clone();
//        cv::circle(real_cur_pos,rot_rect.center,2,cv::Scalar(0,255,255),3);
//        cv::circle(real_cur_pos,debug_point,2,cv::Scalar(100,255,255),3);
//        cv::circle(real_cur_pos,rune_center,2,cv::Scalar(190,255,255),3);
//        for(int i=0;i<4;++i){
//            cv::line(real_cur_pos,points[i%4],points[(i+1)%4],cv::Scalar(0,255,0),2);
//        }
//        cv::namedWindow("RealCurPos",cv::WINDOW_FREERATIO);
//        cv::imshow("RealCurPos",real_cur_pos);
//        std::cout<<"get real curpos!"<<std::endl;

//#endif
}




void RuneHitLogic::setAngleRecent(cv::Point2f pos)
{
    angle_recent = atan2((pos.y - getRuneCenter().y), (pos.x - getRuneCenter().x));
    // 以x为原点，向右，向上为正轴，逆时针求解angle_recent_2PI
    if(angle_recent > 0 && angle_recent < PI)
        angle_recent_2PI = 2 * PI - angle_recent;
    else
        angle_recent_2PI = -angle_recent;
}



void RuneHitLogic::predictShiftWithCompansate(cv::RotatedRect &rot_rect, AngleSolver &angle_solver,
                                                  PackData *pack_data,long t1_param,
                                                  float rotated_angle, GravityCompensateResolve &gc)
{
    angle_solver.getAngleWithRectify(rot_rect, RUNE_ARMOR_WD_RADIO);
    pit_abs_angle -= angle_solver.getPitRef();
    yaw_abs_angle += angle_solver.getYawRef();

#ifdef DEBUG_MODE
    std::cout << "pit_abs_angle_before_grav: " << pit_abs_angle << std::endl;
#endif // DEBUG_MODE
    //预测云台下一次转到的绝对角度
    predictAbsAngle(pit_abs_angle, yaw_abs_angle, rotated_angle);

    CompensateAngle(pit_abs_angle, yaw_abs_angle);
    setPitAbsAngleWithGavityCompansate(gc);

#ifdef DEBUG_MODE
    std::cout << "pit_abs_angle_after_grav: " << pit_abs_angle << std::endl;
#endif // DEBUG_MODE

    //6.数据发送
#ifdef DEBUG_MODE
    std::cout << "================================================================================" << std::endl;
    std::cout << "pit_send: " << -pit_abs_angle << std::endl;
    std::cout << "yaw_send: " << -yaw_abs_angle << std::endl;
    std::cout << "================================================================================" << std::endl;
#endif // DEBUG_MODE
    //pack_data->setPc2StmMesg()->gimbal_control_data.time = t1_param;
    pack_data->setPc2StmMesg()->gimbal_control_data.aim_pitch = -pit_abs_angle;   // 获取俯仰角
    pack_data->setPc2StmMesg()->gimbal_control_data.aim_yaw = -yaw_abs_angle;   // 获取偏航角
    pack_data->setPc2StmMesg()->gimbal_control_data.mode_Union.info.visual_valid = 1;
}




//解算角度后，传入当前绝对角度，算出预测后的绝对角度，rotated_angle要用弧度
void RuneHitLogic::predictAbsAngle(float &pit_abs_angle, float &yaw_abs_angle, float rotated_angle)
{
    cur_pos_point_3d = calCurPosCoordinate(pit_abs_angle, yaw_abs_angle);

#ifdef DEBUG_MODE
    std::cout << "预测前的pitch绝对角: " << pit_abs_angle << std::endl;
    std::cout << "预测前的yaw绝对角: " << yaw_abs_angle << std::endl;
#endif // DEBUG_MODE

    getPrePosPoint(cur_pos_point_3d, rotated_angle);

    calCoorinateAbsAngle(pre_pos_point_3d, pit_abs_angle, yaw_abs_angle);

#ifdef DEBUG_MODE
    std::cout << "预测的pitch绝对角: " << pit_abs_angle << std::endl;
    std::cout << "预测的yaw绝对角: " << yaw_abs_angle << std::endl;
#endif // DEBUG_MODE
}




//rotated_angle要用弧度
void RuneHitLogic::getPrePosPoint(cv::Point3f cur_pos_point, float rotated_angle)
{
    float x, y;

    //坐标轴中心变换
    x = cur_pos_point.x - center_point_3d.x;
    y = cur_pos_point.y - center_point_3d.y;

    if (is_clockwise)
    {
        //顺时针
        pre_pos_point_3d.x = x * cos(rotated_angle) + y * sin(rotated_angle);
        pre_pos_point_3d.y = y * cos(rotated_angle) - x * sin(rotated_angle);
    }
    else
    {
        //逆时针
        pre_pos_point_3d.x = x * cos(rotated_angle) - y * sin(rotated_angle);
        pre_pos_point_3d.y = y * cos(rotated_angle) + x * sin(rotated_angle);
    }

    pre_pos_point_3d.x = pre_pos_point_3d.x + center_point_3d.x;
    pre_pos_point_3d.y = pre_pos_point_3d.y + center_point_3d.y;
    pre_pos_point_3d.z = cur_pos_point.z;



}

//传入当前目标点和云台原点（绝对坐标系）的绝对角度，位置符合平面四大象限（右上角为第一象限）
cv::Point3f RuneHitLogic::calCurPosCoordinate(float pit_abs_angle, float yaw_abs_angle)
{
    cv::Point3f curPos_3d_ori;

    pit_abs_angle = pit_abs_angle * PI / 180;
    yaw_abs_angle = yaw_abs_angle * PI / 180;

    //相当于车永远正对能量机关，不能玩360度
    curPos_3d_ori.z = PTZ2TARGET_RELATIVE_DIS;
    curPos_3d_ori.x = PTZ2TARGET_RELATIVE_DIS * tan(yaw_abs_angle);
    curPos_3d_ori.y = PTZ2TARGET_RELATIVE_DIS * tan(pit_abs_angle);

    return curPos_3d_ori;
}

//传入当前点的3维坐标，得到当前点相对于云台原点（绝对坐标系）的绝对角度
void RuneHitLogic::calCoorinateAbsAngle(cv::Point3f curPos_3d_ori,
                                        float &pit_abs_angle, float &yaw_abs_angle)
{
    yaw_abs_angle = atan2(curPos_3d_ori.x, curPos_3d_ori.z);
    pit_abs_angle = atan2(curPos_3d_ori.y, curPos_3d_ori.z); //not PI/2

    yaw_abs_angle = yaw_abs_angle * 180 / PI;
    pit_abs_angle = pit_abs_angle * 180 / PI;
}

void RuneHitLogic::CompensateAngle(float &pit_ref_send, float &yaw_ref_send) //offset_angle use
{
    float pitch_offset_angle;
    float yaw_offset_angle;
    int is_ptz_raise = 0;
    int is_ptz_right = 0;


        pitch_offset_angle = rune_settings->rune_param.rune_compensate.pitch_offset / 10.0;
        yaw_offset_angle = rune_settings->rune_param.rune_compensate.yaw_offset / 10.0;
        is_ptz_raise = rune_settings->rune_param.rune_compensate.is_ptz_raise;
        is_ptz_right = rune_settings->rune_param.rune_compensate.is_ptz_right;


    if (is_ptz_raise)
    {
        pit_ref_send = pit_ref_send + pitch_offset_angle; //上为正
        pit_compansate_angle = pitch_offset_angle;
    }
    else
    {
        pit_ref_send = pit_ref_send - pitch_offset_angle;
        pit_compansate_angle = -pitch_offset_angle;
    }

    if (is_ptz_right)
    {
        yaw_ref_send = yaw_ref_send + yaw_offset_angle; //右为正
        yaw_compansate_angle = yaw_offset_angle;
    }
    else
    {
        yaw_ref_send = yaw_ref_send - yaw_offset_angle;
        yaw_compansate_angle = -yaw_offset_angle;
    }
}



void RuneHitLogic::getNormalShiftAngle(cv::RotatedRect &rot_rect, AngleSolver &angle_solver, float ratio)
{
    angle_solver.getAngleWithRectify(rot_rect, ratio);

    pit_abs_angle -= angle_solver.getPitRef();  // 编码位+相对角
    yaw_abs_angle += angle_solver.getYawRef();

#ifdef DEBUG_MODE
    std::cout << "pit_ref_angle: " << angle_solver.getPitRef() << std::endl;
    std::cout << "yaw_ref_angle: " << angle_solver.getYawRef() << std::endl;
    std::cout << "pit_abs_angle: " << pit_abs_angle << std::endl;
    std::cout << "yaw_abs_angle: " << yaw_abs_angle << std::endl;
#endif // DEBUG_MODE
}

void RuneHitLogic::setPitAbsAngleWithGavityCompansate(GravityCompensateResolve& gc)
{
    cv::Point3f target2ptz_Point = calCurPosCoordinate(pit_abs_angle, yaw_abs_angle);
    float target2ptz_dis = sqrt(powf(target2ptz_Point.x, 2) + powf(target2ptz_Point.y, 2) + powf(target2ptz_Point.z, 2)) / 100.0; //平面距离

    float target2bul_dis = (target2ptz_dis * 100.0 - OFFSET_Z_PTZ2BULLET) / 100.0;

    float pit_send_angle = gc.solveAngleWithGravity(pit_abs_angle, target2bul_dis);

    pit_gavity_compansate_angle = pit_send_angle - pit_abs_angle;
    pit_abs_angle = pit_send_angle;
}

void RuneHitLogic::updateRuneRotDirection(bool is_clockwise)
{
    this->is_clockwise = is_clockwise;
    std::cout<<"is_clockwise:"<<is_clockwise<<std::endl;
}

bool RuneHitLogic::selectRuneRotation(void)
{
    int clockwise_num = 0;
    int anticlockwise_num = 0;
    int size = rotation_capacity.size();

    for(int i = 0; i < size; ++i)
    {
        if(rotation_capacity[i])
            clockwise_num++;
        else
            anticlockwise_num++;
    }
    return (clockwise_num > anticlockwise_num); //不包括等于的情况，默认size为奇数
}

cv::Point3f RuneHitLogic::selectRuneCenter3d(void)
{
    cv::Point3f center_3d;
    std::vector<cv::Point3f> center_select_capacity;
    float diff = 9;
    int size = center_capacity.size();

    if(size > 2)
    {
        int i, end = center_capacity.size() - 2;
        for(i = 0; i < end; ++i)
        {
            if(std::abs(center_capacity[i].x - center_capacity[i + 1].x) < diff) //先前两个比较
            {
                center_select_capacity.push_back(center_capacity[i]);
                center_3d.x += center_capacity[i].x;
                center_3d.y += center_capacity[i].y;
            }
            else //出现误差
            {
                ++i;
                if(std::abs(center_capacity[i].x - center_capacity[i + 1].x) < diff) //后一个与再后一个比较
                {
                    center_select_capacity.push_back(center_capacity[i]);
                    center_3d.x += center_capacity[i].x;
                    center_3d.y += center_capacity[i].y;
                }
                else //也有误差
                {
                    if(std::abs(center_capacity[i - 1].x - center_capacity[i + 1].x) < diff) //先比较的前一个与后比较的后一个进行比较
                    {
                        center_select_capacity.push_back(center_capacity[i - 1]);
                        center_3d.x += center_capacity[i - 1].x;
                        center_3d.y += center_capacity[i - 1].y;
                    } //else none，再有误差不管，这次循环后i去到后比较的后一个位置上
                }
            }
        }
        if(i == end)
        {
            // 1：如果是从第一个if选中跳出循环就是用选中的进行比较；
            // 2：如果是从第二个if选中跳出循环就是用选中的进行比较；
            // 3：如果是从第二个else跳出循环就是用三个中间的与新的进行比较
            if(std::abs(center_capacity[i - 1].x - center_capacity[i + 1].x) < diff)
            {
                center_select_capacity.push_back(center_capacity[i + 1]);
                center_3d.x += center_capacity[i + 1].x;
                center_3d.y += center_capacity[i + 1].y;
            }
            center_select_capacity.push_back(center_capacity[i]);
            center_3d.x += center_capacity[i].x;
            center_3d.y += center_capacity[i].y;
        }
        else //最后一个进行筛选
        {
            if(center_select_capacity.size() == 0)
            {
                center_select_capacity.push_back(center_capacity[i]); //前面一个都没选中，只好选最后一个
                center_3d.x += center_capacity[i].x;
                center_3d.y += center_capacity[i].y;
            }
            else
            {
                if(std::abs(center_capacity[i].x - center_select_capacity[0].x) < diff) //最后一个与之前选中的第一个进行比较
                {
                    center_select_capacity.push_back(center_capacity[i]);
                    center_3d.x += center_capacity[i].x;
                    center_3d.y += center_capacity[i].y;
                }
            }
        }
        center_3d.x /= center_select_capacity.size(); //可能有误差
        center_3d.y /= center_select_capacity.size();
        center_3d.z = center_select_capacity[0].z;
    }
    else if(center_capacity.size() == 2)
    {
        if(std::abs(center_capacity[0].x - center_capacity[1].x) < diff) //前两个比较
        {
            //center_select_capacity.push_back(center_capacity[0]);
            center_3d.x = (center_capacity[0].x + center_capacity[1].x) / 2;
            center_3d.y = (center_capacity[0].y + center_capacity[1].y) / 2;
            center_3d.z = center_capacity[0].z;
        }
        else //出现误差,只选第一个
        {
            //center_select_capacity.push_back(center_capacity[0]);
            center_3d.x = center_capacity[0].x;
            center_3d.y = center_capacity[0].y;
            center_3d.z = center_capacity[0].z;
        }
    }
    else if(center_capacity.size() == 1)
    {
        //center_select_capacity.push_back(center_capacity[0]);
        center_3d.x = center_capacity[0].x;
        center_3d.y = center_capacity[0].y;
        center_3d.z = center_capacity[0].z;
    }

    return center_3d;
}

void RuneHitLogic::setRuneRotDirection(void)  // 判断方向，默认转动角为小角，不考虑角转动过大
{
    angle_diff = angle_diff * 180 / PI;

    if (angle_diff < -180)  // 默认转动角为小角，角度差突变太大即特殊情况
        updateRuneRotDirection(true);  // 顺时针
    else if (angle_diff < 0)
        updateRuneRotDirection(false);  // 逆时针
    else if (angle_diff > 180)  // 默认转动角为小角，角度差突变太大即特殊情况
        updateRuneRotDirection(false);  // 逆时针
    else if (angle_diff != 0)  // 为0不更新方向
        updateRuneRotDirection(true);  // 顺时针

}

float RuneHitLogic::makeAngleRegular(float angle)
{
    float angle_tmp;
    angle_tmp=fmod(angle,360);
    if(angle_tmp<0)
        angle_tmp+=360;
    return angle_tmp;
}

void RuneHitLogic::InitKalman()
{
    Eigen::Matrix3d A=Eigen::Matrix3d::Identity();
    Eigen::Matrix3d H;
    for(int i=0;i<3;++i)
    {
        H(i,i)=1;
    }
    Eigen::Matrix3d R;
    R(0,0)=1;
    for(int i=0;i<3;++i)
    {
        R(i,i)=100;
    }
    Eigen::VectorXd Q{4};
    Eigen::Vector3d init{0,0,0};
    kalman = _Kalman(A,H,R,Q,init,0);
}

void RuneHitLogic::smallRuneHitProc(cv::Mat src, AngleSolver &angle_solver,
                                  PackData *pack_data, UnpackData *unpack_data,
                                  Serial *serial, long t1_param, GravityCompensateResolve &gc)
{
    if(main_settings->main_mode_flag != 1)
    {

        frame_count = 0;
    }





        // 寻找符合条件的待激活装甲板
        bool is_find_rune = getCurPos(src, rot_rect, cur_pos);

        if(is_find_rune)
        {
            bool is_find_rune_center = false;

            // 寻找能量机关中心点
            if(getPrepTargetRotRect().size() == 1)
                is_find_rune_center = setRuneCenter(src, rot_rect);
            if(is_find_rune_center)
            {
                cv::Point2d cur_pos_center;
                getRealCurPos(src,rot_rect,cur_pos,rune_center,cur_pos_center);
//                cv::Mat hahaha=cv::Mat::zeros(src.size(),src.type());
//                cv::circle(hahaha,rot_rect.center,2,cv::Scalar(255,255,255),2);
//                cv::imshow("pppppp",hahaha);
                //center_2d = getRuneCenter();
                angle_old = angle_recent;
                setAngleRecent(rot_rect.center);
#ifdef DEBUG_MODE
                std::cout << "angle_recent: " << angle_recent << std::endl;
#endif // DEBUG_MODE

                // 获取电控编码位角度
                pit_abs_angle = -unpack_data->getStm2PcMesg()->stm32_info_data.robot_pitch;
                yaw_abs_angle = -unpack_data->getStm2PcMesg()->stm32_info_data.robot_yaw;

                if(std::isnan(pit_abs_angle) || std::isnan(yaw_abs_angle))
                {
#ifdef DEBUG_MODE
                    std::cout << "fail to get the ptz angle!" << std::endl;
#endif // DEBUG_MODE
                    return;
                }
#ifdef DEBUG_MODE
                std::cout << "pit_code_angle: " << getPitAbsAngle() << std::endl;
                std::cout << "yaw_code_angle: " << getYawAbsAngle() << std::endl;
#endif // DEBUG_MODE
                cv::RotatedRect center_rect(rune_center, cv::Size(15, 15), 0);

                // 求解能量机关世界坐标系的３d点
                getNormalShiftAngle(center_rect, angle_solver, 1);
                center_point_3d = calCurPosCoordinate(pit_abs_angle, yaw_abs_angle);

                if(frame_count == 0)
                {
                    center_capacity.clear();
                    rotation_capacity.clear();
                    center_capacity.push_back(center_point_3d);
                    frame_count++;
                }
                else if(frame_count < FRAME_FILTER_NUM)
                {
                    angle_diff = angle_recent - angle_old;
                    setRuneRotDirection();//方向的稳定性待检测
                    center_capacity.push_back(center_point_3d);
                    rotation_capacity.push_back(getRotation());
                    frame_count++;
                }
                else if(frame_count == FRAME_FILTER_NUM)
                {
                    angle_diff = angle_recent - angle_old;
                    setRuneRotDirection();//方向的稳定性待检测
                    center_capacity.push_back(center_point_3d);
                    rotation_capacity.push_back(getRotation());
                    center_point_3d = selectRuneCenter3d(); //选出中心
                    updateRuneRotDirection(selectRuneRotation()); //选出方向
                    frame_count++;
                }
                else //识别多了一次中心，可优化
                {
                    rotated_angle  = PI / 3 * T;
                    predictShiftWithCompansate(rot_rect, angle_solver, pack_data, t1_param, rotated_angle, gc);

                }
            }
            else
            {
                //pack_data->setPc2StmMesg()->gimbal_control_data.visual_valid = 0;
                pack_data->process(serial);
#ifdef DEBUG_MODE
                std::cout << "not find rune center!" << std::endl;
#endif // DEBUG_MODE
            }
        }
        else
        {
            //pack_data->setPc2StmMesg()->gimbal_control_data.visual_valid = 0;
            pack_data->process(serial);
#ifdef DEBUG_MODE
            std::cout << "not find rune!" << std::endl;
#endif // DEBUG_MODE
        }
}



void RuneHitLogic::bigRuneHitProc_2024(cv::Mat src, AngleSolver &angle_solver,double t2,
                                  PackData *pack_data, UnpackData *unpack_data,
                                  Serial *serial, long t1_param, GravityCompensateResolve &gc,int &rune_flag)
{
    if(main_settings->main_mode_flag != 2)
    {
        frame_count = 0;
    }


        bool is_find_rune = getCurPos(src, rot_rect, cur_pos);

        if(is_find_rune)
        {
            bool is_find_rune_center = false;
            is_find_rune_center = setRuneCenter(src, rot_rect);
            if(is_find_rune_center)
            {
                cv::Point2d cur_pos_center;
                getRealCurPos(src,rot_rect,cur_pos,rune_center,cur_pos_center);
                rune_flag++;
                lost_flag = 0;

                cv::Point2d rc_pos_in_rune=cv::Point2d(rune_center.x-cur_pos_center.x,cur_pos_center.y-rune_center.y);//x轴已翻转

                double k=rc_pos_in_rune.y/rc_pos_in_rune.x;

                std::cout<<"k:"<<k<<"???????????????????????????????????????????"<<std::endl;

                double angle=atan(k);
                if(rc_pos_in_rune.x<0&&k<0) angle+=3.14;
                else if(rc_pos_in_rune.x<0&&k>0) angle+=3.14;
                else if(rc_pos_in_rune.x>0&&k<0) angle+=6.28;


                Eigen::Vector3d source_point;
                source_point<< 0.7*cos(angle),0.7*sin(angle),0;


                angle_old = angle_recent;
                angle_recent = angle;



                //setAngleRecent(rot_rect.center);
                //angle_regular = makeAngleRegular(angle_recent*180/PI);

                //angles.push_back(angle_regular);//三个时刻的能量机关位置，以求两个相邻时刻间的角速度和这两个相邻时间角加速度

                angles.push_back(angle_recent);
                if(angles.size()>=4){
                angles.erase(angles.begin());


                pit_abs_angle = -unpack_data->getStm2PcMesg()->stm32_info_data.robot_pitch;
                yaw_abs_angle = -unpack_data->getStm2PcMesg()->stm32_info_data.robot_yaw;

                if(std::isnan(pit_abs_angle) || std::isnan(yaw_abs_angle))
                {
#ifdef DEBUG_MODE
                    std::cout << "fail to get the ptz angle!" << std::endl;
#endif // DEBUG_MODE
                    return;
                }
#ifdef DEBUG_MODE
                std::cout << "pit_code_angle: " << getPitAbsAngle() << std::endl;
                std::cout << "yaw_code_angle: " << getYawAbsAngle() << std::endl;
#endif // DEBUG_MODE

                double v;

                    if(!std::isnan(unpack_data->getStm2PcMesg()->stm32_info_data.bullet_level) && unpack_data->getStm2PcMesg()->stm32_info_data.bullet_level > 0)
                    {

                        v=unpack_data->getStm2PcMesg()->stm32_info_data.bullet_level;
                        predict_t=RUNE_ARMOR_DIS/unpack_data->getStm2PcMesg()->stm32_info_data.bullet_level+rune_settings->rune_param.rune_time_delay.TRIG2SHOOT2HIT_DELAY * 0.01;
#ifdef DEBUG_MODE
                        std::cout << "predict_t:  " << predict_t << std::endl;
#endif // DEBUG_MODE
                    }
                    else
                    {
                        v=28.0;
                        predict_t=RUNE_ARMOR_DIS/28+rune_settings->rune_param.rune_time_delay.TRIG2SHOOT2HIT_DELAY * 0.01;
#ifdef DEBUG_MODE
                        std::cout << "no bulet speed!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!  " << std::endl;
#endif // DEBUG_MODE
                    }

                    t2+=predict_t;


                    angle_diff=angles[2]-angles[1];
                    angle_diff2=angles[1]-angles[0];




                    if(abs(angle_diff)>4.71){
                        angle_diff=angle_diff-6.29*(angle_diff/abs(angle_diff));
                    }
                    else{
                        angle_diff=(int)floor(angle_diff)%72+angle_diff-floor(angle_diff);
                    }

                    if(abs(angle_diff2)>4.71){
                        angle_diff2=angle_diff2-6.29*(angle_diff2/abs(angle_diff2));
                    }
                    else{
                        angle_diff2=(int)floor(angle_diff2)%72+angle_diff2-floor(angle_diff2);
                    }

                    //对angle_diff做跃变与象限跃变处理
#ifdef DEBUG_MODE
                    std::cout<<"angle_diff--------------------------------------------------------------------------------------------"<<angle_diff<<std::endl;
                    std::cout<<"angle_diff2+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++"<<angle_diff2<<std::endl;
#endif






                        Eigen::Vector3d z_k;
                        if(t2!=0)
                           z_k<<angle_diff ,angle_diff / t2,(angle_diff / t2 - angle_diff2 / t2) / t2;
                        else
                           z_k<<angle_diff ,angle_diff / 50,(angle_diff / 50 - angle_diff2 / 50) / 50;

#ifdef DEBUG_MODE
                        std::cout<<"z_k:"<<z_k<<endl;
#endif

                        Eigen::Vector3d state=kalman.update(z_k,t2);        //更新卡尔曼滤波
                                                      //将角度和时间传入卡尔曼滤波后得到的角速度
                        //kalman_speed=speed_recent;
                        //double kalman_angle_recent=state(0,0);                 //待激活装甲板与中心点角度的滤波值

#ifdef DEBUG_MODE
                        double speed_recent=state[1];
                        std::cout<<"speed_recent:"<<speed_recent<<std::endl;
#endif // DEBUG_MODE



                        rotated_angle=std::abs(state[0]);

                        Eigen::AngleAxisd y_rotated(3.14,Eigen::Vector3d(0,1,0));

                        Eigen::Matrix3d y_rotated_Matrix=y_rotated.matrix();

                        Eigen::Vector3d barrel_to_rune_tvec;
                        barrel_to_rune_tvec<<0,-2.295+1.345+0.4,6.809;

                        Eigen::Vector3d predict_point;

                        if(rotated_angle>2.090*t2||rotated_angle<0)
                        {
#ifdef DEBUG_MODE
                            std::cout << "predict angle is error"  << std::endl;
#endif // DEBUG_MODE
                            predict_point=y_rotated_Matrix.inverse()*(source_point-barrel_to_rune_tvec);
                        }
                        else{
                            double rx=source_point(0,0)*cos(rotated_angle)-source_point(1,0)*sin(rotated_angle);
                            double ry=source_point(1,0)*cos(rotated_angle)+source_point(0,0)*sin(rotated_angle);

                            Eigen::Vector3d point_temp;
                            point_temp<<rx,ry,0;

                            predict_point=y_rotated_Matrix.inverse()*(point_temp-barrel_to_rune_tvec);
                        }

                        double ground_dist=sqrt(pow(predict_point(0,0),2)+pow(predict_point(2,0),2));

                        double pitch_to_angle=angle_solver.trajectoryEstimation(ground_dist,predict_point(1,0),v,9.8,fly_time);
                        double yaw_to_angle=atan(predict_point(0,0)/predict_point(2,0));

                        pitch_to_angle*=(180/3.14);
                        yaw_to_angle*=(180/3.14);

                        double moto_move_pitch=-pitch_to_angle+pit_abs_angle+8;
                        double moto_move_yaw=-yaw_to_angle+yaw_abs_angle;


                        std::cout<<"moto_move_pitch"<<moto_move_pitch<<std::endl;
                        std::cout<<"moto_move_yaw"<<moto_move_yaw<<std::endl;

                        pitch_last=moto_move_pitch;
                        yaw_last=moto_move_yaw;

                        pack_data->setPc2StmMesg()->gimbal_control_data.aim_pitch = moto_move_pitch;   // 获取俯仰角
                        pack_data->setPc2StmMesg()->gimbal_control_data.aim_yaw = moto_move_yaw;   // 获取偏航角
                        pack_data->setPc2StmMesg()->gimbal_control_data.mode_Union.info.visual_valid = 1;

                }
                else{
                    pack_data->setPc2StmMesg()->gimbal_control_data.aim_pitch = 0;   // 获取俯仰角
                    pack_data->setPc2StmMesg()->gimbal_control_data.aim_yaw = 0;   // 获取偏航角
                    pack_data->setPc2StmMesg()->gimbal_control_data.mode_Union.info.visual_valid = 1;
                }
            }
            else
            {
#ifdef DEBUG_MODE
                std::cout << "not find rune center!" << std::endl;
#endif // DEBUG_MODE
                rune_flag=0;

                if(lost_flag<5){
                    pack_data->setPc2StmMesg()->gimbal_control_data.aim_pitch = pitch_last;   // 获取俯仰角
                    pack_data->setPc2StmMesg()->gimbal_control_data.aim_yaw = yaw_last;   // 获取偏航角
                    pack_data->setPc2StmMesg()->gimbal_control_data.mode_Union.info.visual_valid = 0;
                    lost_flag++;
                }
                else{
                    pack_data->setPc2StmMesg()->gimbal_control_data.aim_pitch = 0;   // 获取俯仰角
                    pack_data->setPc2StmMesg()->gimbal_control_data.aim_yaw = 0;   // 获取偏航角
                    pack_data->setPc2StmMesg()->gimbal_control_data.mode_Union.info.visual_valid = 0;
                }

            }
        }
        else
        {
#ifdef DEBUG_MODE
                std::cout << "not find curpos!" << std::endl;
#endif // DEBUG_MODE
                rune_flag=0;
                if(lost_flag<5){
                    pack_data->setPc2StmMesg()->gimbal_control_data.aim_pitch = pitch_last;   // 获取俯仰角
                    pack_data->setPc2StmMesg()->gimbal_control_data.aim_yaw =  yaw_last;   // 获取偏航角
                    pack_data->setPc2StmMesg()->gimbal_control_data.mode_Union.info.visual_valid = 0;
                    lost_flag++;
                }
                else{
                    pack_data->setPc2StmMesg()->gimbal_control_data.aim_pitch = 0;   // 获取俯仰角
                    pack_data->setPc2StmMesg()->gimbal_control_data.aim_yaw = 0;   // 获取偏航角
                    pack_data->setPc2StmMesg()->gimbal_control_data.mode_Union.info.visual_valid = 0;
                }
        }

        //串口发送

        pack_data->process(serial);
}



void RuneHitLogic::staticRuneHitProc(cv::Mat src, AngleSolver &angle_solver,
                                     PackData *pack_data, UnpackData *unpack_data,
                                     Serial *serial, long t1_param, GravityCompensateResolve &gc)
{
    std::cout<<"Now you aim by your hand!!!!!!"<<endl;
}

