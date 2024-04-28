#include <frc/smartdashboard/SmartDashboard.h>

#include "networktables/NetworkTable.h"
#include "networktables/NetworkTableInstance.h"
#include "networktables/NetworkTableEntry.h"
#include "networktables/NetworkTableValue.h"

class GoalTracker
{
private:


    std::shared_ptr<nt::NetworkTable> table;
    bool isDetected;
    double target_X;
    double target_Y;
    double target_Area;
    double target_Skew;
    double target_Long;

    std::vector<double> botPose;
    double botPoseX, botPoseZ, botPoseYawn, speedX, speedY, speedAng;


public:
    GoalTracker(){
        table = nt::NetworkTableInstance::GetDefault().GetTable("limelight");
        botPose = {0,0,0,0,0,0};
    }
    ~GoalTracker(){

    }
    void Tracker_Init(){
        table->PutNumber("pipeline",0);
    }
    void GetMessage(){
        // 创建网络表实例
        table = nt::NetworkTableInstance::GetDefault().GetTable("limelight");

        isDetected = table->GetNumber("tv",0.0);
        target_X = table->GetNumber("tx",0.0);
        target_Y = table->GetNumber("ty",0.0);
        target_Area = table->GetNumber("ta",0.0);
        target_Skew = table->GetNumber("ts",0.0);
        target_Long = table->GetNumber("tlong",0.0);
    }
    double Yaw_Follow(){
        
        // botPose = table->GetNumberArray("targetpose_robotspace",std::vector<double>(6));
        // botPoseX = botPose[0];
        // botPoseZ = botPose[2];
        // botPoseYawn = botPose[4];

        // speedAng = botPoseYawn * (-0.02);

        // frc::SmartDashboard::PutNumber("X",botPoseX);
        // frc::SmartDashboard::PutNumber("Z",botPoseZ);
        // frc::SmartDashboard::PutNumber("Yawn",botPoseYawn);

        // frc::SmartDashboard::PutNumber("speedAng",speedAng);

        // speedAng = Cap(speedAng, 0.5);

    }
    bool isAiming(){
        return botPose[0];
    }
        /**
     * @brief 峰值滤波
    */
    double Cap(double value, double peak) {
        if (value < -peak)
            return -peak;
        if (value > +peak)
            return +peak;
        return value;
    }
    
};

