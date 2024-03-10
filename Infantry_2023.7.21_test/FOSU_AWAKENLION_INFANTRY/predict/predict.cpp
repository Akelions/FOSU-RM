#include "predict.h"

ArmorPredictTool::ArmorPredictTool(AngleSolver angle_solver,double moto_pitch,double moto_yaw, double *cars_radio,ArmorObject* Object_addr,
std::unordered_map<int,MultipleValue>cars_map,double bullet_speed,double runnig_time)
{
    this->cars_radio=cars_radio;
    this->car_radio=*(cars_radio+object_cls);
    this->cars_map=cars_map;
    this->angle_solver=angle_solver;
    this->object_addr=object_addr;
    this->moto_pitch=moto_pitch;
    this->moto_yaw=moto_yaw;

    this->running_time=runnig_time;
    this->bullet_speed=bullet_speed;
}

bool ArmorPredictTool::solveCarRadio()
{
    //长度单位为cm
    if(car_radio==0){
        if(object_addr!=nullptr){
            if(/*cars_map[&object_addr.cls].second.same_id_object.size()==2*/0){
                ArmorObject obj1;
                ArmorObject obj2;
                if(&cars_map[car_cls].second.same_id_object[0]==object_addr){
                    obj1=cars_map[car_cls].second.same_id_object[0];
                    obj2=cars_map[car_cls].second.same_id_object[1];
                }
                else{
                    obj1=cars_map[car_cls].second.same_id_object[1];
                    obj2=cars_map[car_cls].second.same_id_object[0];
                }
                Eigen::Vector3d tvec_lu1;Eigen::Vector3d tvec_rd1;Eigen::Vector3d moto_t_lu1;Eigen::Vector3d moto_t_rd1;
                Eigen::Vector3d tvec_lu2;Eigen::Vector3d tvec_rd2;Eigen::Vector3d moto_t_lu2;Eigen::Vector3d moto_t_rd2;
                cv::Point2f lu1[4]={obj1.apex[0],(obj1.apex[0]+obj1.apex[1])/2,(obj1.apex[0]+obj1.apex[2])/2,(obj1.apex[0]+obj1.apex[3])/2};//左上
                cv::Point2f rd1[4]={(obj1.apex[2]+obj1.apex[0])/2,(obj1.apex[2]+obj1.apex[1])/2,obj1.apex[2],(obj1.apex[2]+obj1.apex[3])/2};//右下
                cv::Point2f lu2[4]={obj2.apex[0],(obj2.apex[0]+obj2.apex[1])/2,(obj2.apex[0]+obj2.apex[2])/2,(obj2.apex[0]+obj2.apex[3])/2};//左上
                cv::Point2f rd2[4]={(obj2.apex[2]+obj2.apex[0])/2,(obj2.apex[2]+obj2.apex[1])/2,obj2.apex[2],(obj2.apex[2]+obj2.apex[3])/2};//右下
                angle_solver.getAngle(lu1,tvec_lu1);
                angle_solver.getAngle(rd1,tvec_rd1);
                angle_solver.getAngle(lu2,tvec_ld2);
                angle_solver.getAngle(rd2,tvec_rd2);
                angle_solver.coordinary_transformation(moto_pitch,moto_yaw,tvec_lu1,moto_t_lu1);
                angle_solver.coordinary_transformation(moto_pitch,moto_yaw,tvec_rd1,moto_t_rd1);
                angle_solver.coordinary_transformation(moto_pitch,moto_yaw,tvec_lu2,moto_t_lu2);
                angle_solver.coordinary_transformation(moto_pitch,moto_yaw,tvec_rd2,moto_t_rd2);

                this->tvec1=(moto_t_lu1+moto_t_rd1)/2;
                this->running_time+=sqrt(pow(this->tvec1(2,0),2)+pow(this->tvec1(0,0),2))/100.0/bullet_speed;
                double k1=(moto_t_lu1(0,0)-moto_t_rd1(0,0))/(moto_t_lu1(2,0)-moto_t_rd1(2,0));
                this->car_angle=atan(k1)<0?atan(k1)+M_PI:atan(k1);

                this->tvec2=(moto_t_lu2+moto_t_rd2)/2;
                double k2=(moto_t_lu2(0,0)-moto_t_rd2(0,0))/(moto_t_lu2(2,0)-moto_t_rd1(2,0));

                Eigen::Vector3d tvec1_mid;
                tvec1_mid<<tvec1(0,0),(tvec1(1,0)+tvec2(1,0))/2,tvec1(2,0);
                Eigen::Vector3d tvec2_mid;
                tvec2_mid<<tvec2(0,0),(tvec1(1,0)+tvec2(1,0))/2,tvec2(2,0);

                double 1_k=-1/k1;
                double 2_k=-1/k2;

                double car_tvec_z= (1_k*tvec1_mid(2,0)-2_k*tvec2_mid(2,0)-tvec1_mid(0,0)+tvec2_mid(0,0))/(tvec1_mid(2,0)-tvec2_mid(2,0));
                double car_tvec_x= 1_k*(car_tvec_z-tvec1_mid(2,0))+tvec1_mid(0,0);

                this->car_tvec<<car_tvec_x,tvec1_mid(1,0),car_tvec_z;

                double delata_x=car_tvec(0,0)-tvec1_mid(0,0);
                double delata_z=car_tvec(2,0)-tvec1_mid(2,0);
                this->car_radio=sqrt(delata_x*delata_x+delata_z*delata_z);
                this->*(cars_radio+car_cls)= car_radio;
            }
            else{

                ArmorObject obj1=cars_map[car_cls].second.same_id_object[0];
                Eigen::Vector3d tvec_lu1;Eigen::Vector3d tvec_rd1;Eigen::Vector3d moto_t_lu1;Eigen::Vector3d moto_t_rd1;
                
                cv::Point2f lu1[4]={obj1.apex[0],(obj1.apex[0]+obj1.apex[1])/2,(obj1.apex[0]+obj1.apex[2])/2,(obj1.apex[0]+obj1.apex[3])/2};//左上
                cv::Point2f rd1[4]={(obj1.apex[2]+obj1.apex[0])/2,(obj1.apex[2]+obj1.apex[1])/2,obj1.apex[2],(obj1.apex[2]+obj1.apex[3])/2};//右下
                angle_solver.getAngle(lu1,tvec_lu1);
                angle_solver.getAngle(rd1,tvec_rd1);

                angle_solver.coordinary_transformation(moto_pitch,moto_yaw,tvec_lu1,moto_t_lu1);
                angle_solver.coordinary_transformation(moto_pitch,moto_yaw,tvec_rd1,moto_t_rd1);

                this->tvec1=(moto_t_lu1+moto_t_rd1)/2;
                this->running_time+=sqrt(pow(this->tvec1(2,0),2)+pow(this->tvec1(0,0),2))/100.0/bullet_speed;
                double k1=(moto_t_lu1(0,0)-moto_t_rd1(0,0))/(moto_t_lu1(2,0)-moto_t_rd1(2,0));
                this->car_angle=atan(k1)<0?atan(k1)+M_PI:atan(k1);

                // int sum=0;
                // for(int i=0;i<6;i++){
                //     if(this->*(cars_radio+i)==0){
                //         continue;
                //     }
                //     this->car_radio+=this->*(cars_radio+i);
                //     sum++;
                // }
                // this->car_radio/=sum;
                this->car_radio=0.0;

                double car_tvec_z= tvec1(0,2)+car_radio*sin(car_angle);
                double car_tvec_x= tvec1(0,0)-car_radio*cos(car_angle);

                this->car_tvec<<car_tvec_x,tvec1(1,0),car_tvec_z;
            }
            return 1;
        }
        else{
            std::cout<<"no object find!"<<std::endl;
            return 0;
        }    
    }
    else{
            if(object_addr!=nullptr){
                if(/*cars_map[&object_addr.cls].second.same_id_object.size()==2*/0){
                ArmorObject obj1;
                ArmorObject obj2;
                if(&cars_map[car_cls].second.same_id_object[0]==object_addr){
                    obj1=cars_map[car_cls].second.same_id_object[0];
                    obj2=cars_map[car_cls].second.same_id_object[1];
                }
                else{
                    obj1=cars_map[car_cls].second.same_id_object[1];
                    obj2=cars_map[car_cls].second.same_id_object[0];
                }
                Eigen::Vector3d tvec_lu1;Eigen::Vector3d tvec_rd1;Eigen::Vector3d moto_t_lu1;Eigen::Vector3d moto_t_rd1;
                Eigen::Vector3d tvec_lu2;Eigen::Vector3d tvec_rd2;Eigen::Vector3d moto_t_lu2;Eigen::Vector3d moto_t_rd2;
                cv::Point2f lu1[4]={obj1.apex[0],(obj1.apex[0]+obj1.apex[1])/2,(obj1.apex[0]+obj1.apex[2])/2,(obj1.apex[0]+obj1.apex[3])/2};//左上
                cv::Point2f rd1[4]={(obj1.apex[2]+obj1.apex[0])/2,(obj1.apex[2]+obj1.apex[1])/2,obj1.apex[2],(obj1.apex[2]+obj1.apex[3])/2};//右下
                cv::Point2f lu2[4]={obj2.apex[0],(obj2.apex[0]+obj2.apex[1])/2,(obj2.apex[0]+obj2.apex[2])/2,(obj2.apex[0]+obj2.apex[3])/2};//左上
                cv::Point2f rd2[4]={(obj2.apex[2]+obj2.apex[0])/2,(obj2.apex[2]+obj2.apex[1])/2,obj2.apex[2],(obj2.apex[2]+obj2.apex[3])/2};//右下
                angle_solver.getAngle(lu1,tvec_lu1);
                angle_solver.getAngle(rd1,tvec_rd1);
                angle_solver.getAngle(lu2,tvec_ld2);
                angle_solver.getAngle(rd2,tvec_rd2);
                angle_solver.coordinary_transformation(moto_pitch,moto_yaw,tvec_lu1,moto_t_lu1);
                angle_solver.coordinary_transformation(moto_pitch,moto_yaw,tvec_rd1,moto_t_rd1);
                angle_solver.coordinary_transformation(moto_pitch,moto_yaw,tvec_lu2,moto_t_lu2);
                angle_solver.coordinary_transformation(moto_pitch,moto_yaw,tvec_rd2,moto_t_rd2);

                this->tvec1=(moto_t_lu1+moto_t_rd1)/2;
                this->running_time+=sqrt(pow(this->tvec1(2,0),2)+pow(this->tvec1(0,0),2))/100.0/bullet_speed;
                double k1=(moto_t_lu1(0,0)-moto_t_rd1(0,0))/(moto_t_lu1(2,0)-moto_t_rd1(2,0));
                this->car_angle=atan(k1)<0?atan(k1)+M_PI:atan(k1);

                this->tvec2=(moto_t_lu2+moto_t_rd2)/2;
                double k2=(moto_t_lu2(0,0)-moto_t_rd2(0,0))/(moto_t_lu2(2,0)-moto_t_rd1(2,0));

                double car_tvec_z= tvec1(0,2)+car_radio*sin(car_angle);
                double car_tvec_x= tvec1(0,0)-car_radio*cos(car_angle);

                this->car_tvec<<car_tvec_x,tvec1(1,0),car_tvec_z;
            }
            else{

                ArmorObject obj1=cars_map[car_cls].second.same_id_object[0];
                Eigen::Vector3d tvec_lu1;Eigen::Vector3d tvec_rd1;Eigen::Vector3d moto_t_lu1;Eigen::Vector3d moto_t_rd1;
                
                cv::Point2f lu1[4]={obj1.apex[0],(obj1.apex[0]+obj1.apex[1])/2,(obj1.apex[0]+obj1.apex[2])/2,(obj1.apex[0]+obj1.apex[3])/2};//左上
                cv::Point2f rd1[4]={(obj1.apex[2]+obj1.apex[0])/2,(obj1.apex[2]+obj1.apex[1])/2,obj1.apex[2],(obj1.apex[2]+obj1.apex[3])/2};//右下
                angle_solver.getAngle(lu1,tvec_lu1);
                angle_solver.getAngle(rd1,tvec_rd1);

                angle_solver.coordinary_transformation(moto_pitch,moto_yaw,tvec_lu1,moto_t_lu1);
                angle_solver.coordinary_transformation(moto_pitch,moto_yaw,tvec_rd1,moto_t_rd1);

                this->tvec1=(moto_t_lu1+moto_t_rd1)/2;

                this->running_time+=sqrt(pow(this->tvec1(2,0),2)+pow(this->tvec1(0,0),2))/100.0/bullet_speed;

                double k1=(moto_t_lu1(0,0)-moto_t_rd1(0,0))/(moto_t_lu1(2,0)-moto_t_rd1(2,0));
                this->car_angle=atan(k1)<0?atan(k1)+M_PI:atan(k1);

                double car_tvec_z= tvec1(0,2)+car_radio*sin(car_angle);
                double car_tvec_x= tvec1(0,0)-car_radio*cos(car_angle);

                this->car_tvec<<car_tvec_x,tvec1(1,0),car_tvec_z;
            }
        }
        return 1
    }
}

bool ArmorPredictTool::predictRotated()
{
    static std::queue<double> angles;
    angles.push(this->car_angle);

       if(x_store.size()>=3){
            angles.pop();
            Eigen::Vector2d _angle;
            Eigen::Vector2d _anglep;


            if(t2<=0) t2=200;
            _angle << angles[1],(_angle[1]-_angle[0])/t2;
            _anglep=angle_kalman.update(_angle,t2);

           this->angle_predict=_anglep(0,0);
            if(angle_predict<PI/4){
                if(angle_predict>0){
                    angle_predict+=PI/2;
                    this->switch_y=1;
                }
                else{
                    angle_predict+=PI;
                    this->switch_y=0;
                }
            }
            else if(angle_predict>PI*3/4){
                if(angle_predict<PI){
                    angle_predict-=PI/2;
                    this->switch_y=1;
                }
                else{
                    angle_predict-=PI;
                    this->switch_y=0;
                }
            }
    }
}

bool ArmorPredictTool::predictMove()
{

    static std::queue<double> x_store;
    static std::queue<double> y_store;
    static std::queue<double> z_store;
    
       x_store.push(car_tvec(0,0));
       y_store.push(car_tvec(1,0));
       z_store.push(car_tvec(2,0));



       if(x_store.size()>=3){
        x_store.pop();
        y_store.pop();
        z_store.pop();


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
    
    double armor_z=car_predict+car_radio*sin(angle_predict);
    double armor_x=car_predict+car_radio*cos(angle_predict);
    double armor_y=switch_y?tvec2(1,0):tvec1(1,0);
    tvec_armor<<armor_x,armor_y,armor_z;

}

bool ArmorPredictTool::predictArmor()
{
    if(solveCarRadio()){
        if(predictRotated()){
            if(predictMove()){
                stateAdd();
                return true;
            }
        }
    }
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
                cv::Point2d rc_pos_in_rune=cv::Point2d(rune_center.x-cur_pos_center.x,cur_pos_center.y-rune_center.y);//x轴已翻转
                for(int i=0;i<4;i++){
                    cur_pos_points[i].x=rune_center.x-cur_pos_points[i].x;
                    cur_pos_points[i].y-=rune_center.y;
                }//将整个靶子转到旋转中心坐标系中

                double k=rc_pos_in_rune.y/rc_pos_in_rune.x;

                std::cout<<"k:"<<k<<"???????????????????????????????????????????"<<std::endl;

                double angle=atan(k);
                if(rc_pos_in_rune.x<0&&k<0) angle+=3.14;
                else if(rc_pos_in_rune.x<0&&k>0) angle+=3.14;
                else if(rc_pos_in_rune.x>0&&k<0) angle+=6.28;

                angle*=180/3.1415926;// 转成角度制处理


                // Eigen::Vector3d source_point;
                // source_point<< 0.7*cos(angle),0.7*sin(angle),0;

                Eigen::Vector3d source_tvec;
                Eigen::Vector3d moto_source_tvec;
                angle_solver.getAngle(pre_cur_pos_points,source_tvec);
                angle_solver.coordinary_transformation(moto_pitch,moto_yaw,source_tvec,moto_source_tvec);
                double cur_dist=sqrt(pow(moto_source_tvec(0,2),2)+pow(moto_source_tvec(0,0),2))/100.0;

                angle_old = angle_recent;
                angle_recent = angle;

                angles.push(angle_recent);

                if(angles.size()>=4){
                    angles.pop();
                    double predict_t;

                    if(bullet_speed > 0)
                    {
                        predict_t=cur_dist/bullet_speed;
                    }
                    else
                    {
                        predict_t=cur_dist/26;
#ifdef DEBUG_MODE
                        std::cout << "no bulet speed!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!  " << std::endl;
#endif // DEBUG_MODE
                    }

                    this->running_time+=predict_t;


                    angle_diff=angles[2]-angles[1];
                    angle_diff2=angles[1]-angles[0];




                    if(abs(angle_diff)>=180&&angles[2]/angles[1]<0){
                        angle_diff=angle_diff-360.0*(angle_diff/abs(angle_diff));
                    }
                    else{
                        angle_diff=(int)floor(angle_diff)%72+angle_diff-floor(angle_diff);
                    }

                    if(abs(angle_diff2)>=180&&angles[1]/angles[0]<0){
                        angle_diff2=angle_diff2-360.0*(angle_diff2/abs(angle_diff2));
                    }
                    else{
                        angle_diff2=(int)floor(angle_diff2)%72+angle_diff2-floor(angle_diff2);
                    }

                        //对angle_diff做跃变与象限跃变处理
#ifdef DEBUG_MODE
                        std::cout<<"angle_diff--------------------------------------------------------------------------------------------"<<angle_diff<<std::endl;
                        std::cout<<"angle_diff2+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++"<<angle_diff2<<std::endl;
#endif


                    Eigen::Vector2d z_k;
                    if(t2!=0)
                        z_k<<angle_diff ,angle_diff / t2;
                    else
                        z_k<<angle_diff ,angle_diff / 1.5;

#ifdef DEBUG_MODE
                    std::cout<<"z_k:"<<z_k<<endl;
#endif

                    Eigen::Vector2d state=kalman.update(z_k,t2);        //更新卡尔曼滤波
                                                      //将角度和时间传入卡尔曼滤波后得到的角速度
                    //kalman_speed=speed_recent;
                    //double kalman_angle_recent=state(0,0);                 //待激活装甲板与中心点角度的滤波值

                    pre_angle=state[0];

                    for(int i=0;i<4;i++){
                        cv::Point2f temp_point;
                        temp_point.x=cur_pos_points[i].x*cos(pre_angle)-cur_pos_points[i].y*sin(pre_angle)+rune_center.x;
                        temp_point.y=cur_pos_points[i].y*cos(pre_angle)+cur_pos_points[i].x*sin(pre_angle)-rune_center.y;
                        pre_cur_pos_points.push_back(temp_point);
                    }
                    Eigen::Vector3d tvec;Eigen::Vector3d moto_tvec;

                    angle_solver.getAngle(pre_cur_pos_points,tvec);

                    angle_solver.coordinary_transformation(moto_pitch,moto_yaw,tvec,moto_tvec);

                    this->cur_moto_tvec=moto_tvec;
                    return 1;
                }
                else{
                    return 0;
                }
}


