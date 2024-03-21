#include "predict.h"

ArmorPredictTool::ArmorPredictTool(){

}


void ArmorPredictTool::inputData(AngleSolver angle_solver,double moto_pitch,double moto_yaw, double *cars_radio,ArmorObject* object_addr,
                                 std::vector<ArmorObject> *cars_map,double bullet_speed,double running_time){
    this->cars_radio=cars_radio;
    this->car_radio=*(cars_radio+(*object_addr).cls);
    this->cars_map=cars_map;
    this->angle_solver=angle_solver;
    this->object_addr=object_addr;
    this->moto_pitch=moto_pitch;
    this->moto_yaw=moto_yaw;

    this->running_time=running_time;
    this->bullet_speed=bullet_speed;
}

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
    if(cars_map[car_cls].size()==2){
        Eigen::Vector3d tvec11; Eigen::Vector3d moto_t_11; Eigen::Vector3d rvec11;
        Eigen::Vector3d tvec22; Eigen::Vector3d moto_t_22; Eigen::Vector3d rvec22;
        ArmorObject obj1;
        ArmorObject obj2;
        if(   (*object_addr).area==cars_map[car_cls][0].area /*&& ((*object_addr).apex[0].y==cars_map[car_cls][0].y)*/){
            obj1=cars_map[car_cls][0];
            obj2=cars_map[car_cls][1];
        }
        else{
            obj1=cars_map[car_cls][1];
            obj2=cars_map[car_cls][0];
        }
        cv::Point2f armor_points1[4]={obj1.apex[0],obj1.apex[1],obj1.apex[2],obj1.apex[3]};
        cv::Point2f armor_points2[4]={obj2.apex[0],obj2.apex[1],obj2.apex[2],obj2.apex[3]};
        double armor_width1=(POINT_DIST(armor_points1[0],armor_points1[3])+POINT_DIST(armor_points1[1],armor_points1[2]))/2.0;
        double armor_width2=(POINT_DIST(armor_points2[0],armor_points2[3])+POINT_DIST(armor_points2[1],armor_points2[2]))/2.0;

        if((tvec11(2,0)+tvec22(2,0))/2.0<=480.0&&car_cls==0){  //近距离无数据则解
            angle_solver.getAngle(armor_points1,tvec11,rvec11);
            angle_solver.getAngle(armor_points2,tvec22,rvec22);
            double armor_height1=6178.0/tvec11(2,0);
            double armor_height2=6178.0/tvec22(2,0);
            int sign_flag1=POINT_DIST(armor_points1[0],armor_points1[1])>POINT_DIST(armor_points1[2],armor_points1[3])?1:-1;
            int sign_flag2=POINT_DIST(armor_points2[0],armor_points2[1])>POINT_DIST(armor_points2[2],armor_points2[3])?1:-1;
            double angle1=(armor_width1/armor_height1)/(15.3/5.7)*(M_PI/2);//弧度制
            double angle2=(armor_width2/armor_height2)/(15.3/5.7)*(M_PI/2);//弧度制
            double car_angle1=sign_flag1==-1?angle1:M_PI-angle1;
            double car_angle2=sign_flag2==-1?angle2:M_PI-angle2;
            this->car_angle=car_angle1;

            angle_solver.coordinary_transformation(moto_pitch,moto_yaw,tvec11,rvec11,moto_t_11);
            angle_solver.coordinary_transformation(moto_pitch,moto_yaw,tvec22,rvec22,moto_t_22);
            this->tvec1=moto_t_11;
            this->tvec2=moto_t_22;
            this->running_time+=sqrt(pow(this->tvec1(2,0),2)+pow(this->tvec1(0,0),2))/100.0/bullet_speed;

            Eigen::Vector3d tvec1_mid;
            tvec1_mid<<tvec1(0,0),(tvec1(1,0)+tvec2(1,0))/2,tvec1(2,0);
            Eigen::Vector3d tvec2_mid;
            tvec2_mid<<tvec2(0,0),(tvec1(1,0)+tvec2(1,0))/2,tvec2(2,0);

            double k_1=atan(angle1);
            double k_2=atan(angle2);

            double car_tvec_z= (k_1*tvec1_mid(2,0)-k_2*tvec2_mid(2,0)-tvec1_mid(0,0)+tvec2_mid(0,0))/(tvec1_mid(2,0)-tvec2_mid(2,0));
            double car_tvec_x= k_1*(car_tvec_z-tvec1_mid(2,0))+tvec1_mid(0,0);

            this->car_tvec<<car_tvec_x,tvec1_mid(1,0),car_tvec_z;

            double delata_x=car_tvec(0,0)-tvec1_mid(0,0);
            double delata_z=car_tvec(2,0)-tvec1_mid(2,0);
            this->car_radio=sqrt(delata_x*delata_x+delata_z*delata_z);
            this->cars_radio[car_cls]= car_radio;
        }

        else{
            angle_solver.getAngle(armor_points1,tvec11,rvec11);
            angle_solver.getAngle(armor_points2,tvec22,rvec22);
            double armor_height=6178.0/tvec11(2,0);
            int sign_flag=POINT_DIST(armor_points1[0],armor_points1[1])>POINT_DIST(armor_points1[2],armor_points1[3])?1:-1;
            double angle=(armor_width1/armor_height)/(15.3/5.7)*(M_PI/2);//弧度制
            this->car_angle=sign_flag==-1?angle:M_PI-angle;

            angle_solver.coordinary_transformation(moto_pitch,moto_yaw,tvec11,rvec11,moto_t_11);
            angle_solver.coordinary_transformation(moto_pitch,moto_yaw,tvec22,rvec22,moto_t_22);

            this->tvec1=moto_t_11;
            this->tvec2=moto_t_22;
            //std::cout<<angle/M_PI*180.0<<std::endl;
            this->running_time+=sqrt(pow(this->tvec1(2,0),2)+pow(this->tvec1(0,0),2))/100.0/bullet_speed;
            if(cars_radio[car_cls]==0)
                this->car_tvec=tvec1;
            else{
                double car_tvec_z= tvec1(2,0)+car_radio*sin(car_angle);
                double car_tvec_x= tvec1(0,0)-car_radio*cos(car_angle);
                this->car_tvec<<car_tvec_x,tvec1(1,0),car_tvec_z;
            }

        }




        return 1;
    }


    else if(cars_map[car_cls].size()==1){
        Eigen::Vector3d tvec11; Eigen::Vector3d moto_t_11;
        Eigen::Vector3d rvec11;
        ArmorObject obj1=cars_map[car_cls][0];
        cv::Point2f armor_points[4]={obj1.apex[0],obj1.apex[1],obj1.apex[2],obj1.apex[3]};
        double armor_width=(POINT_DIST(armor_points[0],armor_points[3])+POINT_DIST(armor_points[1],armor_points[2]))/2.0;
        angle_solver.getAngle(armor_points,tvec11,rvec11);
        double armor_height=6178.0/tvec11(2,0);
        int sign_flag=POINT_DIST(armor_points[0],armor_points[1])>POINT_DIST(armor_points[2],armor_points[3])?1:-1;
        double angle=(armor_width/armor_height)/(15.3/5.7)*(M_PI/2);//弧度制
        this->car_angle=sign_flag==-1?angle:M_PI-angle;

        angle_solver.coordinary_transformation(moto_pitch,moto_yaw,tvec11,rvec11,moto_t_11);
        this->tvec1=moto_t_11;
        //std::cout<<car_angle/M_PI*180.0<<std::endl;//yes!
        this->running_time+=sqrt(pow(this->tvec1(2,0),2)+pow(this->tvec1(0,0),2))/100.0/bullet_speed;
        if(cars_radio[car_cls]==0)
            this->car_tvec=tvec1;
        else{
            double car_tvec_z= tvec1(2,0)+car_radio*sin(car_angle);
            double car_tvec_x= tvec1(0,0)+car_radio*cos(car_angle);
            this->car_tvec<<car_tvec_x,tvec1(1,0),car_tvec_z;
        }
        //std::cout<<"car_tvec"<<car_tvec<<std::endl;
        return 1;
    }
    else
        return 0;
}

bool ArmorPredictTool::predictRotated()
{

    static std::vector<double> angles;
    angles.push_back(this->car_angle);
//    std::cout<<car_angle*180.0/M_PI<<std::endl;

       if(angles.size()>=3){
            angles.erase(angles.begin());
            this->angle_predict=car_angle;

//            Eigen::Vector2d _angle;
//            Eigen::Vector2d _anglep;


//            if(running_time<=0) running_time=1.0;
//            _angle << angles[1]-angles[0],(angles[1]-angles[0])/running_time;
//            _anglep=angle_kalman.update(_angle,running_time);
//            //std::cout<<"_anglep"<<_anglep<<std::endl;
//           this->angle_predict=angles[1]+_anglep(0,0);
//            if(angle_predict<M_PI/4.0){
//                if(_anglep(1,0)<-1.57&&angle_predict>0.0){
//                    angle_predict+=M_PI/2.0;
//                    this->switch_y=1;
//                }
//                else if(_anglep(1,0)<-1.57&&angle_predict<0.0){
//                    angle_predict+=M_PI;
//                    this->switch_y=0;
//                }
//                else if(angle_predict<M_PI/4.3){
//                    angle_predict+=M_PI/2.0;
//                    this->switch_y=1;
//                }
//            }
//            else if(angle_predict>M_PI*3.0/4.0){
//                if(_anglep(1,0)>1.57&&angle_predict<M_PI){
//                    angle_predict-=M_PI/2.0;
//                    this->switch_y=1;
//                }
//                else if(_anglep(1,0)>1.57&&angle_predict>M_PI){
//                    angle_predict-=M_PI;
//                    this->switch_y=0;
//                }
//                else if(angle_predict>M_PI*3.6/4.0){
//                    angle_predict+=M_PI/2.0;
//                    this->switch_y=1;
//                }
//            }
            //std::cout<<"angle_predict"<<angle_predict*180.0/M_PI<<std::endl;
            return true;
    }
       else return false;
}

bool ArmorPredictTool::predictMove()
{

    static std::vector<double> x_store;
    static std::vector<double> y_store;
    static std::vector<double> z_store;
    //std::cout<<"car_tvec"<<car_tvec<<std::endl;
    
       x_store.push_back(this->car_tvec(0,0));
       y_store.push_back(this->car_tvec(1,0));
       z_store.push_back(this->car_tvec(2,0));



       if(x_store.size()>=3){
        x_store.erase(x_store.begin());
        y_store.erase(y_store.begin());
        z_store.erase(z_store.begin());

        car_predict=car_tvec;


//           Eigen::Vector2d _X;
//           Eigen::Vector2d _Y;
//           Eigen::Vector2d _Z;
//           Eigen::Vector2d _Xp;
//           Eigen::Vector2d _Yp;
//           Eigen::Vector2d _Zp;

//           if(this->running_time<=0) this->running_time=0.5;

//           _X<< (x_store[1]-x_store[0]),(x_store[1]-x_store[0])/this->running_time;
//           _Y<< (y_store[1]-y_store[0]),(y_store[1]-y_store[0])/this->running_time;
//           _Z<< (z_store[1]-z_store[0]),(z_store[1]-z_store[0])/this->running_time;




//           _Xp=car_kalman_x.update(_X,this->running_time);
//           _Yp=car_kalman_y.update(_Y,this->running_time);
//           _Zp=car_kalman_z.update(_Z,this->running_time);

//           //std::cout<<"_Xp"<<_Xp<<std::endl;

           
//           car_predict<< car_tvec(0,0)+_Xp(0,0),car_tvec(1,0)+_Yp(0,0),car_tvec(2,0)+_Zp(0,0);
//           std::cout<<"car_predict"<<car_predict<<std::endl;

////           cout<<"_Xp(0,0)!!!!!!!!!!!!!!!!"<<_Xp(0,0)<<endl;
////           cout<<"_Yp(0,0)!!!!!!!!!!!!!!!!"<<_Yp(0,0)<<endl;
////           cout<<"_Zp(0,0)!!!!!!!!!!!!!!!!"<<_Zp(0,0)<<endl;
           return true;
       }
       else return false;
}

bool ArmorPredictTool::stateAdd()
{
    
//    double armor_z=car_predict(2,0)-car_radio*sin(angle_predict);
//    double armor_x=car_predict(0,0)-car_radio*cos(angle_predict);
//    double armor_y=switch_y?tvec2(1,0):tvec1(1,0);
    double armor_z=car_predict(2,0)-car_radio*sin(angle_predict);
    double armor_x=car_predict(0,0)-car_radio*cos(angle_predict);
    double armor_y=tvec1(1,0);
    //std::cout<<"tvec"<<tvec1<<std::endl;
    tvec_armor<<armor_x,armor_y,armor_z;
    //std::cout<<"tvec_armor"<<tvec_armor<<std::endl;

}

void ArmorPredictTool::kalmanInit(){
    static bool init_flag=false;
    if(!init_flag){
        Eigen::Matrix2d A=Eigen::Matrix2d::Identity();
        Eigen::Matrix2d H;
        for(int i=0;i<2;++i)
        {
            H(i,i)=1;
        }
        Eigen::Matrix2d R;
        for(int i=0;i<2;++i)
        {
            R(i,i)=100;
        }
        Eigen::Vector2d Q{0.1,0.1};
        Eigen::Vector2d init{0,0};
        angle_kalman = Armor_Kalman(A,H,R,Q,init,0);
        car_kalman_x = Armor_Kalman(A,H,R,Q,init,0);
        car_kalman_y = Armor_Kalman(A,H,R,Q,init,0);
        car_kalman_z = Armor_Kalman(A,H,R,Q,init,0);
        init_flag=true;
        std::cout<<"init success!"<<std::endl;
    }
}

bool ArmorPredictTool::predictArmor()
{


    if(solveCarRadio()){
       if(predictRotated()){

           if(predictMove()){
               stateAdd();
               return true;
           }
           else return false;
       }
       else return false;
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
//                if(rc_pos_in_rune.x<0&&k<0) angle+=M_PI;
//                else if(rc_pos_in_rune.x<0&&k>0) angle+=M_PI;
//                else if(rc_pos_in_rune.x>0&&k<0) angle+=6.28;

//                angle*=180/M_PI;// 转成角度制处理


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


