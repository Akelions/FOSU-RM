#include "PnpSolver.h"

AngleSolver::AngleSolver(const char *camera_param_file_name, double z_scale)
{
    rectPnpSolver(camera_param_file_name);
    scale_z = z_scale;

    rot_camera2ptz = cv::Mat::eye(3, 3, CV_64FC1);
    trans_camera2ptz = cv::Mat::zeros(3, 1, CV_64FC1);
    offset_y_barrel_ptz = 0;
}

double AngleSolver::getPitRef()
{
    return pit_ref;
}

double AngleSolver::getYawRef()
{
    return yaw_ref;
}

void AngleSolver::setRelationPoseCameraPTZ(const double ptz_camera_x, const double ptz_camera_y,
                                           const double ptz_camera_z, double y_offset_barrel_ptz)
{

    double r_data[] = {1, 0, 0,
                       0, 1, 0,
                       0, 0, 1};

    cv::Mat rot_camera_ptz(3, 3, CV_64FC1, r_data);
    rot_camera_ptz.copyTo(rot_camera2ptz);

    // 云台相对于相机的偏移向量和平移向量
    double t_data[] = {ptz_camera_x, ptz_camera_y, -ptz_camera_z};
    cv::Mat trans_camera_ptz(3, 1, CV_64FC1, t_data);
    trans_camera_ptz.copyTo(trans_camera2ptz);

    offset_y_barrel_ptz = y_offset_barrel_ptz;
}

cv::Point2f AngleSolver::getImageCenter()
{
    return cv::Point2f(cam_matrix.at<double>(0, 2), cam_matrix.at<double>(1, 2));
}

//能量机关专用
bool AngleSolver::getAngleWithRectify(const cv::RotatedRect &rect,
                                      double wh_ratio,
                                      const cv::Point2f &offset)
{
    cv::RotatedRect rect_rectifid = rect;

    rect_rectifid.size.width = rect_rectifid.size.height / wh_ratio;
    setTargetSize(rect_rectifid.size.height, rect_rectifid.size.width);

    std::vector<cv::Point2f> target2d;
    //求物体坐标系相对于相机坐标系，求向量矩阵
    getTarget2dPoinstion(rect_rectifid, target2d, offset);
    return getAngleWithoutGavity(target2d);
}

//能量机关专用
void AngleSolver::adjustPTZ2BarrelWithoutGavity(const cv::Mat &pos_in_ptz, double &angle_x, double &angle_y)
{
    const double *xyz = (const double *)pos_in_ptz.data;
    double alpha = 0.0, theta = 0.0;

    angle_x = atan2(xyz[0], xyz[2]);

    alpha = asin(offset_y_barrel_ptz / sqrt(xyz[1] * xyz[1] + xyz[2] * xyz[2]));

    if (xyz[1] < 0)
    {
        theta = atan(-xyz[1] / xyz[2]);
        angle_y = - alpha - theta;
    }
    else
    {
        theta = atan(xyz[1] / xyz[2]);
        angle_y = -alpha + theta;
    }
    angle_x = angle_x * 180.0 / PI;
    angle_y = angle_y * 180.0 / PI;
}


void AngleSolver::solvePnP4Points_rune(const std::vector<cv::Point2f> &points2d, cv::Mat &rot, cv::Mat &trans)
{
    cv::Mat rvec;
    cv::solvePnP(point3d, points2d, cam_matrix, distortion_coeff, rvec, trans); //能量机关
    cv::Rodrigues(rvec, rot);
}

//能量机关专用
bool AngleSolver::getAngleWithoutGavity(std::vector<cv::Point2f> target2d)
{
    //根据检测出的目标在图像中的二维坐标，算出旋转矩阵与位移向量
    solvePnP4Points_rune(target2d, this->rot, position_in_camera);
    tranformationCamera2PTZ(position_in_camera, position_in_ptz);
    // 根据目标在PTZ坐标中的位置，计算偏移角度，使枪管瞄准目标
    adjustPTZ2BarrelWithoutGavity(position_in_ptz, yaw_ref,pit_ref);
    return true;
}

void AngleSolver::getTarget2dPoinstion(const cv::RotatedRect &rect,
    std::vector<cv::Point2f> &target2d,
    const cv::Point2f &offset)
{
    cv::Point2f vertices[4];
    rect.points(vertices);
    cv::Point2f lu, ld, ru, rd;
    std::sort(vertices, vertices + 4, [](const cv::Point2f & p1, const cv::Point2f & p2) { return p1.x < p2.x; });
    if (vertices[0].y < vertices[1].y)
    {
        lu = vertices[0];
        ld = vertices[1];
    }
    else
    {
        lu = vertices[1];
        ld = vertices[0];
    }
    if (vertices[2].y < vertices[3].y)
    {
        ru = vertices[2];
        rd = vertices[3];
    }
    else
    {
        ru = vertices[3];
        rd = vertices[2];
    }

    target2d.clear();
    target2d.push_back(lu + offset);
    target2d.push_back(ru + offset);
    target2d.push_back(rd + offset);
    target2d.push_back(ld + offset);
}

void AngleSolver::rectPnpSolver(const char *camera_param_file_name)
{
    cv::FileStorage fs(camera_param_file_name, cv::FileStorage::READ);
    if(!fs.isOpened())
        std::cout << "Could not open the configuration file" << std::endl;

    fs["Camera_Matrix"] >> cam_matrix;
    fs["Distortion_Coefficients"] >> distortion_coeff;
    fs["board_Width"] >> width_target;
    fs["board_Height"] >> height_target;

    if(cam_matrix.empty() || distortion_coeff.empty())
    {
        std::cout << "cam_matrix or distortion_coeff is empty!!!" << std::endl;
        return;
    }

    //根据目标矩形的宽高设置三维坐标
    double half_x = width_target / 2.0;
    double half_y = height_target / 2.0;

    point3d.push_back(cv::Point3f(-half_x, -half_y, 0));
    point3d.push_back(cv::Point3f(half_x, -half_y, 0));
    point3d.push_back(cv::Point3f(half_x, half_y, 0));
    point3d.push_back(cv::Point3f(-half_x, half_y, 0));
}

void AngleSolver::setTargetSize(double width, double height)
{
    width_target = width;
    height_target = height;

    //根据目标矩形的宽高设置三维坐标
    double half_x = width_target / 2.0;
    double half_y = height_target / 2.0;

    point3d.clear();
    point3d.push_back(cv::Point3f(-half_x, -half_y, 0));
    point3d.push_back(cv::Point3f(-half_x, half_y, 0));
    point3d.push_back(cv::Point3f(half_x, half_y, 0));
    point3d.push_back(cv::Point3f(half_x, -half_y, 0));
}

bool AngleSolver::getAngle(cv::Point2f *target2d,Eigen::Vector3d &tvec)
{
    //根据检测出的目标在图像中的二维坐标，算出旋转矩阵与位移向量
    Eigen::Vector3d rvec;
    solvePnP4Points(target2d,tvec,rvec);

    //tvec<<position_in_camera.at<double>(0, 0),position_in_camera.at<double>(1, 0),position_in_camera.at<double>(2, 0);
//    // 相机坐标转换到PTZ坐标
//    tranformationCamera2PTZ(position_in_camera, position_in_ptz);
//    // 根据目标在PTZ坐标中的位置，计算偏移角度，使枪管瞄准目标
//    adjustPTZ2Barrel(position_in_ptz, yaw_ref,pit_ref, bullet_speed, current_ptz_angle);
    return true;
}

void AngleSolver::tranformationCamera2PTZ(const cv::Mat &pos, cv::Mat &transed_pos)
{
    transed_pos = rot_camera2ptz * pos - trans_camera2ptz;
}

void AngleSolver::adjustPTZ2Barrel(const cv::Mat &pos_in_ptz,
    double &angle_x, double &angle_y,
    double bullet_speed,
    double current_ptz_angle)
{
    const double *_xyz = (const double *)pos_in_ptz.data;
    angle_x = atan2(_xyz[0], _xyz[2]);

    double xyz[3] = { _xyz[0], _xyz[1], _xyz[2] };

    double alpha = 0.0, theta = 0.0;

    alpha = asin(offset_y_barrel_ptz / sqrt(xyz[1] * xyz[1] + xyz[2] * xyz[2]));

    if (xyz[1] < 0)
    {
        theta = atan(-xyz[1] / xyz[2]);
        angle_y = - alpha - theta;
    }
    else
    {
        theta = atan(xyz[1] / xyz[2]);
        angle_y = -alpha + theta;
    }
    angle_x = angle_x * 180.0 / PI;
    angle_y = angle_y * 180.0 / PI;
}

void AngleSolver::solvePnP4Points(const cv::Point2f *points2d, Eigen::Vector3d &trans,Eigen::Vector3d &rvec)
{
    cv::solvePnP(point3d, points2d, cam_matrix, distortion_coeff, rvec, trans/*, true, CV_P3P*/); //自瞄
}
/*
void AngleSolver::getDistanceDanmu(std::vector<cv::Point2f>armor_rect, double &dist)
{
    cv::RotatedRect armor = cv::minAreaRect(armor_rect);
    float p_w = std::max(armor.size.width, armor.size.height);
    float p_h = std::min(armor.size.width, armor.size.height);

    float fx_w = cam_matrix.at<double>(0, 0) * width_target;
    float fx_h = cam_matrix.at<double>(1, 1) * height_target;

    float dist_w = fx_w / p_w;
    float dist_h = fx_h / p_h;

    dist = (dist_w + dist_h) / 2;
}
*/

/*2023赛季新单目测距*/
/*
    解决的问题：
    1.4m内测距误差5cm内
    2.5m内测距误差10cm内
    尚未解决的问题：
    1.左右倾斜不准
    2.超过6m测距不准
    3.装甲板偏移屏幕中心点不准
    4.像素点抖动导致测距不准
*/
/*pnp测距*/
void AngleSolver::getDistanceDanmuPnP(const std::vector<cv::Point2f> &points2d,
                                    double &dist){
    cv::Mat rvec = cv::Mat::zeros(3, 1, CV_64FC1);
//    std::cout << "2d点 = " << std::endl << points2d <<std::endl;

    cv::solvePnP(point3d, points2d, cam_matrix, distortion_coeff, rvec, position_in_camera, cv::SOLVEPNP_ITERATIVE);
    double x_pos =  position_in_camera.at<double>(0, 0);
    double y_pos =  position_in_camera.at<double>(1, 0);
    double z_pos =  position_in_camera.at<double>(2, 0);
//    dist = sqrt(x_pos * x_pos + y_pos * y_pos + z_pos * z_pos);
    dist = z_pos * 1.0;

    std::cout << "当前使用pnp测距" << std::endl;
    //std::cout << "旋转向量 = " << std::endl << rvec << std::endl;
    std::cout << "x_pos = " << x_pos << std::endl;
    std::cout << "y_pos = " << y_pos << std::endl;
    std::cout << "z_pos = " << z_pos << std::endl;
    //std::cout << "dist = " << dist << std::endl;

//    double angle_x = std::atan(x_pos/z_pos);
//    angle_x = angle_x * 180.0 / PI;
//    std::cout << "pnp: angle_x = " << angle_x << std::endl;
}

/*小孔成像测距new*/
void AngleSolver::getDistanceDanmu(std::vector<cv::Point2f>armor_rect, double &dist)
{
    cv::RotatedRect armor = cv::minAreaRect(armor_rect);
//    std::cout << "armor_rect = " << std::endl << armor_rect <<std::endl;

    float p_w = std::max(armor.size.width, armor.size.height);
    float p_h = std::min(armor.size.width, armor.size.height);

    float fx_w = cam_matrix.at<double>(0, 0) * width_target; //* SHOW_WIDTH / 750;    //750是因为标定时大小为750（600同理）
    float fx_h = cam_matrix.at<double>(1, 1) * height_target; //* SHOW_HEIGHT / 600;

    float dist_w = fx_w / p_w;
    float dist_h = fx_h / p_h;

    //装甲板倾斜的时候dist_w偏大,此处为30度时dist_w大于dist_h的距离
    if(dist_w - dist_h > 15  )
    {
        dist = dist_h;
    }
    else
    {
        //正对装甲板时，削弱高比例（高波动大，不准）
        dist = (4*dist_w + dist_h) / 5;
    }


    std::cout << "当前使用小孔成像测距"<<std::endl;
//    std::cout << "灯条像素宽 = " << p_w << std::endl;
//    std::cout << "灯条像素高 = " << p_h << std::endl;
//    std::cout << "fx_w = " << fx_w << std::endl;
//    std::cout << "fx_h = " << fx_h << std::endl;
    std::cout << "dist_w = " << dist_w << std::endl;
    std::cout << "dist_h = " << dist_h << std::endl;
    std::cout << "dist = " << dist << std::endl;

}


void AngleSolver::Camera2Moto(double moto_pitch, double moto_yaw , Eigen::Vector3d tvec, double &moto_move_pitch, double &moto_move_yaw,double v,double g)
{
    v*=100.0;
    g*=100.0;//输入用国际单位制，但是运算建议用量纲长度cm，时间用s即可
    moto_pitch/=(180.0/3.14);//电控收发都是角度制，需要转成弧度制处理
    double z=tvec(1,0);
    double y=abs(tvec(2,0));
    //y=sqrt(pow(x,2)+pow(y,2));
    //上述已转完坐标系

     double k;//zy枪管
     double vz1,vy1,fly_time;
     fly_time = sqrt(1.0 / (g * g) * (g * z + sqrt(v * v * v * v - (g * g) * (y * y) - g * (v * v) * z * 2.0) - v * v) * -2.0);

     if (/*std::isnan(fly_time)*/1) {
         k = y / z;
     }
     else {
         vz1 = +(g * (y * y) * sqrt(-1.0 / (g * g) * (g * z * 2.0 + sqrt(v * v * v * v - (g * g) * (y * y) - g * (v * v) * z * 2.0) * 2.0 - (v * v) * 2.0)) * 2.0 - g * (z * z) * sqrt(-1.0 / (g * g) * (g * z * 2.0 + sqrt(v * v * v * v - (g * g) * (y * y) - g * (v * v) * z * 2.0) * 2.0 - (v * v) * 2.0)) * 2.0 - (g * g) * z * pow(-1.0 / (g * g) * (g * z * 2.0 + sqrt(v * v * v * v - (g * g) * (y * y) - g * (v * v) * z * 2.0) * 2.0 - (v * v) * 2.0), 3.0 / 2.0) + (v * v) * z * sqrt(-1.0 / (g * g) * (g * z * 2.0 + sqrt(v * v * v * v - (g * g) * (y * y) - g * (v * v) * z * 2.0) * 2.0 - (v * v) * 2.0)) * 4.0) / ((y * y) * 4.0 + (z * z) * 4.0);
         vy1 = -((g * g) * y * pow(-1.0 / (g * g) * (g * z * 2.0 + sqrt(v * v * v * v - (g * g) * (y * y) - g * (v * v) * z * 2.0) * 2.0 - (v * v) * 2.0), 3.0 / 2.0) - (v * v) * y * sqrt(-1.0 / (g * g) * (g * z * 2.0 + sqrt(v * v * v * v - (g * g) * (y * y) - g * (v * v) * z * 2.0) * 2.0 - (v * v) * 2.0)) * 4.0 + g * y * z * sqrt(-1.0 / (g * g) * (g * z * 2.0 + sqrt(v * v * v * v - (g * g) * (y * y) - g * (v * v) * z * 2.0) * 2.0 - (v * v) * 2.0)) * 4.0) / ((y * y) * 4.0 + (z * z) * 4.0);
         k = vy1 / vz1;

     }
    double angle=atan(-1.0/k);
    double moto_to_pitch=angle;

    moto_move_pitch=moto_to_pitch-moto_pitch;//电控收角度制，发给他们前需要转成角度制
   // std::cout<<"moto_move_pitch"<<moto_move_pitch<<std::endl;
    moto_move_pitch*=(180.0/3.14);


    //以下都是yaw的与pitch无关

    int moto_yaw_int=floor(moto_yaw);
    double moto_yaw_flo=moto_yaw-moto_yaw_int;

    if(moto_yaw_int>0) moto_yaw_int%=360;
    else moto_yaw_int=moto_yaw_int%360+360;
    moto_yaw=moto_yaw_int+moto_yaw_flo;

        std::cout<<"moto_yaw!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"<<moto_yaw<<std::endl;

    moto_yaw/=(180.0/3.14);//电控收发都是角度制，需要转成弧度制处理


    double _x2=tvec(0,0);
    double _z2=tvec(2,0);

    double moto_to_yaw=atan(_x2/_z2);

    //std::cout<<"zr2!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"<<zr2<<std::endl;
    //std::cout<<"xr2!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"<<xr2<<std::endl;

    if(_z2<0&&_x2>0){
        moto_to_yaw+=3.14;
    }
    else if(_z2<0&&_x2<0){
        moto_to_yaw+=3.14;
    }

   else if(_z2>0&&_x2<0){
       moto_to_yaw+=6.28;
   }

   std::cout<<"moto_to_yaw!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"<<moto_to_yaw*(180.0/3.14)<<std::endl;


    moto_move_yaw=-(moto_to_yaw-moto_yaw);
    moto_move_yaw*=(180.0/3.14);//已转成角度制
    std::cout<<"moto_move_yaw!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"<<moto_move_yaw<<std::endl;

    if(abs(moto_move_yaw)>270){
        std::cout<<"moto_move_yaw error!!!!!!!!"<<std::endl;
        moto_move_yaw=-moto_move_yaw/abs(moto_move_yaw)*360.0+moto_move_yaw;
    }
    //moto_move_yaw=0;
    //需要相机和电机的位置,不懂就去翻机械原理！！！(或者理论力学动力学部分)
}

double AngleSolver::trajectoryEstimation(double ground_dist, double height, double v, double g,double &fly_time)
{
    double standar_angle=atan(height/ground_dist);
    std::cout<<"standar_angle"<<standar_angle<<endl;
    double x=ground_dist;
    double y=height;

    double x_2=pow(x,2);
    double y_2=pow(y,2);
    double g_2=pow(g,2);
    double v_2=pow(v,2);
    double v_4=pow(v,4);

    double a;
    double b;
    double c;
    double b_2;
    double b_3;
    double c_2;
    double c_3;


    if(-g_2*x_2-2*y*g*v_2+v_4>0){
       a=sqrt(-g_2*x_2-2*y*g*v_2+v_4);
       b=2*(a-g*y+v_2)/g_2;
       c=-2*(g*y+a-v_2)/g_2;
       if(b>0&&c>0){
         b_2=sqrt(b);
         b_3=pow(b,1.5);
         c_2=sqrt(c);
         c_3=pow(c,1.5);

         double k1=(x*a+v_2*x-g*x*y)/(y*a+g*x_2+v_2*y);
         double k2=-k1;
         double k3=(g_2*x*b_3-4*v_2*x*b_2+4*g*x*y*b_2)/(2*g*x_2*c_2-2*g*y_2*c_2-g_2*y*c_3+4*v_2*y*c_2);
         double k4=-k3;


         double angle[4];

         angle[0]=atan(k1);
         angle[1]=atan(k2);
         angle[2]=atan(k3);
         angle[3]=atan(k4);

         double minn=2.0;
         double ans=0.0;

         for(int i=0;i<4;i++){

             std::cout<<"angle["<<i<<"]"<<angle[i]<<endl;
             if(abs(angle[i]-standar_angle)<minn){
                 ans=angle[i];
                 minn=abs(angle[i]-standar_angle);
             }
         }
         fly_time=std::min(b_2,c_2);
         return ans;

       }
       else{

           double k1=(x*a+v_2*x-g*x*y)/(y*a+g*x_2+v_2*y);
           double k2=-k1;
           double angle1=atan(k1);
           double angle2=atan(k2);
           std::cout<<"angle1"<<angle1<<std::endl;
           std::cout<<"angle2"<<angle2<<std::endl;
           fly_time=0.0;
           return abs(angle1-standar_angle)<abs(angle2-standar_angle)?angle1:angle2;
       }
    }
    else return standar_angle;




}

void AngleSolver::coordinary_transformation(double moto_pitch, double moto_yaw, Eigen::Vector3d tvec, Eigen::Vector3d &moto_tvec)
{

    moto_pitch/=(180.0/3.14);//电控收发都是角度制，需要转成弧度制处理


    double _z=sqrt(pow(tvec(2,0),2)+pow(tvec(0,0),2));
    double _y=tvec(1,0);


    double d_c=sqrt(pow(PBMD,2)+pow(PCBD,2));
    double arfa=atan(PCBD/PBMD);

    double zc=cos(3.14/2-moto_pitch-arfa)*d_c;
    double yc=sin(3.14/2-moto_pitch-arfa)*d_c;

    double threta=3.14/2+moto_pitch;

    double x=tvec(0,0);
    double z=zc+_z*cos(threta)-_y*sin(threta)-PBMD;
    double y=yc+_y*cos(threta)+_z*sin(threta)-PCBD;
    //y=sqrt(pow(x,2)+pow(y,2));
    //上述已转完坐标系


//    double z_f=pow(z,2);
//    double x_f=pow(x,2);
//    double y_f=pow(y,2);
//    double l_f=pow(PBMD,2);
//    double y_t=pow(y,3);
//    double v_f=pow(v,2);
    //以下都是yaw的与pitch无关

    int moto_yaw_int=floor(moto_yaw);
    double moto_yaw_flo=moto_yaw-moto_yaw_int;

    if(moto_yaw_int>=0) moto_yaw_int%=360;
    else moto_yaw_int=moto_yaw_int%360+360;
    moto_yaw=moto_yaw_int+moto_yaw_flo;
    moto_yaw/=(180.0/3.14);//电控收发都是角度制，需要转成弧度制处理

    ///////////////////////////////////////////
    double d_c2=sqrt(pow(YBMD,2)+pow(YCBD,2));
    double arfa2=atan(YCBD/YBMD);

    double temp_angle=moto_yaw-arfa2+1.57;
    if(temp_angle<0.0){
        temp_angle+=3.14*2;
    }
    else if(temp_angle>=3.14*2){
        temp_angle-=3.14*2;
    }

    double xc2=sin(temp_angle)*d_c2;
    std::cout<<"xc2!!!!!!!!!!!!!"<<xc2<<std::endl;
    double zc2=cos(temp_angle)*d_c2;
    //////////////////////////////////////////

//  double xc2=sin(moto_yaw)*YCBD;
//  double zc2=cos(moto_yaw)*YCBD;

    double xr2=tvec(0,0);
       std::cout<<"xr2!!!!!!!!!!!!!"<<xr2<<std::endl;
    double zr2=tvec(2,0);

    double threta2=moto_yaw;

    double _x2=xc2+xr2*cos(threta2)+zr2*sin(threta2);
    double _z2=zc2+zr2*cos(threta2)-xr2*sin(threta2);

    moto_tvec<<_x2,z,_z2;
}

