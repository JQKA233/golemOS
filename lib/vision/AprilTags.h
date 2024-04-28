#pragma once

#include <frc/smartdashboard/SmartDashboard.h>

#include "networktables/NetworkTable.h"
#include "networktables/NetworkTableInstance.h"
#include "networktables/NetworkTableEntry.h"
#include "networktables/NetworkTableValue.h"

class AprilTags
{
private:
    std::shared_ptr<nt::NetworkTable> table;

    std::vector<double> botPose;
    std::vector<double> TagID;

    double botPoseX, botPoseZ, botPoseYawn, errAng, errX, errZ, speedAng, speedX, speedZ, lastErrA;
    
    bool flag_vision;

    double Angle_0;

public:
    AprilTags(){
        table = nt::NetworkTableInstance::GetDefault().GetTable("limelight");
        botPose = {0,0,0,0,0,0};
        flag_vision = 0;
        Angle_0 = 0;
        TagID = {0};
    }
    ~AprilTags(){
    }
    
    /**
     * @brief 四月标签识别初始化函数
     * @attention 在RoboInit中执行
    */
    void April_Init(){
        table->PutNumber("pipeline",0);
    }

    /**
     * @brief 获取标志位
    */
    bool GetFlag(){
        return flag_vision;
    }

    /**
     * @brief 设置标志位
     * @param 设置标志位值（布尔值）
    */
    void SetFlag(bool state){
        if(state){
            flag_vision = true;
        }
        else{
            flag_vision = false;
        }
    }

    /**
     * @brief 设置底盘瞄准前IMU数据，作为目标值
    */
    void SetAngle_0(double angle){
        Angle_0 = angle;
    }

    /**
     * @brief 获取底盘瞄准前IMU数据
    */
    double GetAngle_0(){
        return Angle_0;
    }

    /**
     * @brief 获取视觉信息
    */
    void GetVisionInfo(){
        botPose = table->GetNumberArray("targetpose_robotspace",std::vector<double>(6));
        TagID = table->GetNumberArray("tid",std::vector<double>(1));
    }

    /**
     * @brief 获取极坐标中辐角值
    */
    double GetAngle_inPolar_Vision(){
        return atan2(botPose.at(0), botPose.at(2));
    }

    /**
     * @brief 获取极坐标中极径值
    */
    double GetDistance_inPolar_Vision(){
        return sqrt(botPose.at(0) * botPose.at(0) + botPose.at(2) * botPose.at(2));
    }

    double GetError_inLeftRight(){
        return botPose.at(0);
    }
    /**
     * @brief 是否瞄准
    */
    bool isAiming(){
        if(botPose.at(0)!=0){
            return true;
        }
        else{
            return false;
        }
    }
    
    double Get_AprilID_Vision(){
        return TagID.at(0);
    }
    double GetX_Vision(){
        return botPose.at(0);
    }
    double GetY_Vision(){
        return botPose.at(1);
    }
    double GetZ_Vision(){
        return botPose.at(2);
    }
    double GetPitch_Vision(){
        return botPose.at(3);
    }
    double GetYaw_Vision(){
        return botPose.at(4);
    }
    double GetRoll_Vision(){
        return botPose.at(5);
    }


};
