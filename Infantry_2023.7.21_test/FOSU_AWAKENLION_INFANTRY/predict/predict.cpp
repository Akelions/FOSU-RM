#include "predict.h"

ArmorPredictTool::ArmorPredictTool(AngleSolver angle_solver,double moto_pitch,double moto_yaw, double *cars_radio,ArmorObject* object_addr,
std::vector<ArmorObject> *cars_map,double bullet_speed,double running_time)
{
    this->cars_radio=cars_radio;
    this->car_radio=*(cars_radio+(*object_addr).cls);
    this->cars_map=cars_map;
    this->angle_solver=angle_solver;
    this->object_addr=object_addr;
    this->moto_pitch=moto_pitch;
    this->moto_yaw=moto_yaw;

    this->running_time=running_time;
    this->bullet_speed=bullet_speed;
//#ifdef DEBUG_MODE
//    std::cout<<"car_radio:"<<car_radio<<std::endl;
//    std::cout<<"(*object_addr).color:"<<(*object_addr).color<<std::endl;
//#endif
}

bool ArmorPredictTool::solveCarRadio()
{
    //长度单位为cm
    int car_cls=(*object_addr).cls;
    Eigen::Vector3d tvec11; Eigen::Vector3d moto_t_11;
    Eigen::Vector3d rvec11;
    ArmorObject obj1=cars_map[car_cls][0];
    cv::Point2f armor_points[4]={obj1.apex[0],obj1.apex[1],obj1.apex[2],obj1.apex[3]};
    double armor_width=(POINT_DIST(armor_points[0],armor_points[3])+POINT_DIST(armor_points[1],armor_points[2]))/2.0;
    double armor_height(POINT_DIST(armor_points[0],armor_points[1])*
            max(POINT_DIST(armor_points[0],armor_points[3]),POINT_DIST(armor_points[1],armor_points[2]))
            /min(POINT_DIST(armor_points[0],armor_points[3]),POINT_DIST(armor_points[1],armor_points[2])));
    double angle=(armor_width/armor_height)/(15.3/5.7)*(M_PI/2);

//    Eigen::Vector3d tvec_lu1;Eigen::Vector3d tvec_rd1;Eigen::Vector3d moto_t_lu1;Eigen::Vector3d moto_t_rd1;
//    Eigen::Vector3d rvec_lu1;Eigen::Vector3d rvec_rd1;Eigen::Vector3d moto_r_lu1;Eigen::Vector3d moto_r_rd1;

//    cv::Point2f lu1[4]={obj1.apex[0],(obj1.apex[0]+obj1.apex[1])/2,(obj1.apex[0]+obj1.apex[2])/2,(obj1.apex[0]+obj1.apex[3])/2};//左上
//    cv::Point2f rd1[4]={(obj1.apex[2]+obj1.apex[0])/2,(obj1.apex[2]+obj1.apex[1])/2,obj1.apex[2],(obj1.apex[2]+obj1.apex[3])/2};//右下
//    angle_solver.getAngle(lu1,tvec_lu1,rvec_lu1);
//    angle_solver.getAngle(rd1,tvec_rd1,rvec_rd1);
//    angle_solver.coordinary_transformation(moto_pitch,moto_yaw,tvec_lu1,rvec_lu1,moto_t_lu1);
//    angle_solver.coordinary_transformation(moto_pitch,moto_yaw,tvec_rd1,rvec_rd1,moto_t_rd1);
//    double k1=(moto_t_lu1(0,0)-moto_t_rd1(0,0))/(moto_t_lu1(2,0)-moto_t_rd1(2,0));
//    this->car_angle=atan(k1)<0?atan(k1)+M_PI:atan(k1);
//    std::cout<<car_angle/3.14*180.0<<std::endl;




    angle_solver.getAngle(armor_points,tvec11,rvec11);
    angle_solver.coordinary_transformation(moto_pitch,moto_yaw,tvec11,rvec11,moto_t_11);
    this->tvec1=moto_t_11;
    std::cout<<armor_height<<"  "<<moto_t_11(2,0)<<" "<<armor_height/moto_t_11(2,0)<<std::endl;
    //std::cout<<angle/3.14*180.0<<std::endl;


//    this->running_time+=sqrt(pow(this->tvec1(2,0),2)+pow(this->tvec1(0,0),2))/100.0/bullet_speed;
//    double car_tvec_z= tvec1(2,0)+car_radio*sin(car_angle);
//    double car_tvec_x= tvec1(0,0)-car_radio*cos(car_angle);

//    this->car_tvec<<car_tvec_x,tvec1(1,0),car_tvec_z;
    return 1;
}

bool ArmorPredictTool::predictRotated()
{

    static std::vector<double> angles;
    angles.push_back(this->car_angle);
//    std::cout<<car_angle<<std::endl;
//    std::cout<<"here?"<<std::endl;

       if(angles.size()>=3){
            angles.erase(angles.begin());
            Eigen::Vector2d _angle;
            Eigen::Vector2d _anglep;


            if(running_time<=0) running_time=1.0;
            _angle << angles[1],(angles[1]-angles[0])/running_time;
            _anglep=angle_kalman.update(_angle,running_time);

           this->angle_predict=_anglep(0,0);
            if(angle_predict<M_PI/4){
                if(angle_predict>0){
                    angle_predict+=M_PI/2;
                    this->switch_y=1;
                }
                else{
                    angle_predict+=M_PI;
                    this->switch_y=0;
                }
            }
            else if(angle_predict>M_PI*3.0/4.0){
                if(angle_predict<M_PI){
                    angle_predict-=M_PI/2;
                    this->switch_y=1;
                }
                else{
                    angle_predict-=M_PI;
                    this->switch_y=0;
                }
            }
    }
}

bool ArmorPredictTool::predictMove()
{

    static std::vector<double> x_store;
    static std::vector<double> y_store;
    static std::vector<double> z_store;
    
       x_store.push_back(car_tvec(0,0));
       y_store.push_back(car_tvec(1,0));
       z_store.push_back(car_tvec(2,0));



       if(x_store.size()>=3){
        x_store.erase(x_store.begin());
        y_store.erase(y_store.begin());
        z_store.erase(z_store.begin());


           Eigen::Vector2d _X;
           Eigen::Vector2d _Y;
           Eigen::Vector2d _Z;
           Eigen::Vector2d _Xp;
           Eigen::Vector2d _Yp;
           Eigen::Vector2d _Zp;

           if(this->running_time<=0) this->running_time=0.5;

           _X<< x_store[1],(x_store[1]-x_store[0])/this->running_time;
           _Y<< y_store[1],(y_store[1]-y_store[0])/this->running_time;
           _Z<< z_store[1],(z_store[1]-z_store[0])/this->running_time;




           _Xp=car_kalman.update(_X,this->running_time);
           _Yp=car_kalman.update(_Y,this->running_time);
           _Zp=car_kalman.update(_Z,this->running_time);

           
           car_predict<< _Xp(0,0),_Yp(0,0),_Zp(0,0);

           cout<<"_Xp(0,0)!!!!!!!!!!!!!!!!"<<_Xp(0,0)<<endl;
           cout<<"_Yp(0,0)!!!!!!!!!!!!!!!!"<<_Yp(0,0)<<endl;
           cout<<"_Zp(0,0)!!!!!!!!!!!!!!!!"<<_Zp(0,0)<<endl;
       }
}

bool ArmorPredictTool::stateAdd()
{
    
    double armor_z=car_predict(2,0)+car_radio*sin(angle_predict);
    double armor_x=car_predict(0,0)+car_radio*cos(angle_predict);
    double armor_y=switch_y?tvec2(1,0):tvec1(1,0);
    tvec_armor<<armor_x,armor_y,armor_z;

}

bool ArmorPredictTool::predictArmor()
{

    if(solveCarRadio()){
//        if(predictRotated()){

//            if(predictMove()){
//                stateAdd();
//                return true;
//            }
//            else return false;
//        }
//        else return false;
        return true;
    }
    else return false;
}

RunePredictTool::RunePredictTool(){
}

RunePredictTool::RunePredictTool(AngleSolver angle_solver,BuffObject object,double moto_pitch,double moto_yaw,double bullet_speed,double running_time){
    
    this->angle_solver=angle_solver;
    this->rune_center=object.apex[0];
    this->moto_pitch=moto_pitch;
    this->moto_yaw=moto_yaw;
    for(int i=1;i<5;i++){
        this->cur_pos_points.push_back(object.apex[i]);
    }
    this->cur_pos_center=(object.apex[1]+object.apex[3])/2;
}
bool RunePredictTool::setRuneCoordinary(){
//                cv::Point2d rc_pos_in_rune=cv::Point2d(rune_center.x-cur_pos_center.x,cur_pos_center.y-rune_center.y);//x轴已翻转
//                for(int i=0;i<4;i++){
//                    cur_pos_points[i].x=rune_center.x-cur_pos_points[i].x;
//                    cur_pos_points[i].y-=rune_center.y;
//                }//将整个靶子转到旋转中心坐标系中

//                double k=rc_pos_in_rune.y/rc_pos_in_rune.x;

//                std::cout<<"k:"<<k<<"???????????????????????????????????????????"<<std::endl;

//                double angle=atan(k);
//                if(rc_pos_in_rune.x<0&&k<0) angle+=3.14;
//                else if(rc_pos_in_rune.x<0&&k>0) angle+=3.14;
//                else if(rc_pos_in_rune.x>0&&k<0) angle+=6.28;

//                angle*=180/3.1415926;// 转成角度制处理


//                // Eigen::Vector3d source_point;
//                // source_point<< 0.7*cos(angle),0.7*sin(angle),0;

//                Eigen::Vector3d source_tvec;
//                Eigen::Vector3d moto_source_tvec;
//                angle_solver.getAngle(pre_cur_pos_points,source_tvec);
//                angle_solver.coordinary_transformation(moto_pitch,moto_yaw,source_tvec,moto_source_tvec);
//                double cur_dist=sqrt(pow(moto_source_tvec(0,2),2)+pow(moto_source_tvec(0,0),2))/100.0;


//                static double angle_recent = angle;
//                static double angle_old = angle_recent;

//                angles.push_back(angle_recent);

//                if(angles.size()>=4){
//                    angles.erase(angles.begin());
//                    double predict_t;

//                    if(bullet_speed > 0)
//                    {
//                        predict_t=cur_dist/bullet_speed;
//                    }
//                    else
//                    {
//                        predict_t=cur_dist/26;
//#ifdef DEBUG_MODE
//                        std::cout << "no bulet speed!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!  " << std::endl;
//#endif // DEBUG_MODE
//                    }

//                    this->running_time+=predict_t;


//                    double angle_diff=angles[2]-angles[1];
//                    double angle_diff2=angles[1]-angles[0];




//                    if(abs(angle_diff)>=180&&angles[2]/angles[1]<0){
//                        angle_diff=angle_diff-360.0*(angle_diff/abs(angle_diff));
//                    }
//                    else{
//                        angle_diff=(int)floor(angle_diff)%72+angle_diff-floor(angle_diff);
//                    }

//                    if(abs(angle_diff2)>=180&&angles[1]/angles[0]<0){
//                        angle_diff2=angle_diff2-360.0*(angle_diff2/abs(angle_diff2));
//                    }
//                    else{
//                        angle_diff2=(int)floor(angle_diff2)%72+angle_diff2-floor(angle_diff2);
//                    }

//                        //对angle_diff做跃变与象限跃变处理
//#ifdef DEBUG_MODE
//                        std::cout<<"angle_diff--------------------------------------------------------------------------------------------"<<angle_diff<<std::endl;
//                        std::cout<<"angle_diff2+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++"<<angle_diff2<<std::endl;
//#endif


//                    Eigen::Vector2d z_k;
//                    if(running_time!=0)
//                        z_k<<angle_diff ,angle_diff / running_time;
//                    else
//                        z_k<<angle_diff ,angle_diff / 1.5;

//#ifdef DEBUG_MODE
//                    std::cout<<"z_k:"<<z_k<<endl;
//#endif

//                    Eigen::Vector2d state=kalman.update(z_k,running_time);        //更新卡尔曼滤波
//                                                      //将角度和时间传入卡尔曼滤波后得到的角速度
//                    //kalman_speed=speed_recent;
//                    //double kalman_angle_recent=state(0,0);                 //待激活装甲板与中心点角度的滤波值

//                    pre_angle=state[0];

//                    for(int i=0;i<4;i++){
//                        cv::Point2f temp_point;
//                        temp_point.x=cur_pos_points[i].x*cos(pre_angle)-cur_pos_points[i].y*sin(pre_angle)+rune_center.x;
//                        temp_point.y=cur_pos_points[i].y*cos(pre_angle)+cur_pos_points[i].x*sin(pre_angle)-rune_center.y;
//                        pre_cur_pos_points.push_back(temp_point);
//                    }
//                    Eigen::Vector3d tvec;Eigen::Vector3d moto_tvec;

//                    angle_solver.getAngle(pre_cur_pos_points,tvec);

//                    angle_solver.coordinary_transformation(moto_pitch,moto_yaw,tvec,moto_tvec);

//                    this->cur_moto_tvec=moto_tvec;
//                    return 1;
//                }
//                else{
//                    return 0;
//                }
}


