#ifndef RUNEDETECTOR_H
#define RUNEDETECTOR_H

#include "Settings/Settings.h"
#include "Serial/Serial.h"
#include "Serial/PackData.h"
#include "AngleSolver/PnpSolver.h"
#include "RuneDetector/inference_api2.hpp"
#include "Preprocessing/Preprocessing.h"
#include "kalman/armor_kalman.h"

// 宏定义尽量放在非Settings.h的头文件，这样编译时间会小
//#define TRIG2SHOOT2HIT_DELAY 0.6 //需要超前转动的时间：下位机响应时间 + 发弹延时 + 子弹发出到击中的时间（S）：0.1（假设） + 0.145（大概） + 0.25（接近）//0.482
//#define PTZ_SHIFT_DELAY 0.3 // 允许云台移动的时间（云台一定要响应完成并到达）：下位机响应时间 + 云台移动延时 + 到达后的缓冲时间（S）：0.1（假设） + 0.47（大概） + 0.038 //0.65
//#define SHOOT2HIT_DELAY 0.82 // 等待子弹发射到装甲点亮的时间：子弹发出到击中的时间 + 到达后的缓冲时间（注：包括打中装甲到点亮过程的延迟，赛场跟实验室的延迟不一样）（S）：0.25（接近） + 0.04
#define SHOOT_NUM 5 // (8 + 1) 9times
#define PTZ2TARGET_RELATIVE_DIS 686 // 原点到目标水平相对距离，即z轴上的距离，子弹发射初始位置再到目标距离：644.0 + 42 = 686
#define FRAME_FILTER_NUM 20 // 大能量机关模式开始时先读的FRAME_FILTER_NUM + 1帧，过滤筛选合适的需要的数据
#define RUNE_ARMOR_WD_RADIO 1.0215 // 1.923077 //1.80212 // 1.7857 //1.0215
#define OFFSET_Z_PTZ2BULLET 7.5 // 云台中心与子弹发射初始位置的偏移量
#define RUNE_ARMOR_DIS 6.63701740  //待激活装甲板到云台的大概距离

using namespace buff_detector;

class RuneDetector
{
public:
    RuneDetector();
    RuneDetector(BuffObject object);
    void detectorRune();

};


#endif // RUNEDETECTOR_H
