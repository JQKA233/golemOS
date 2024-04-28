#pragma once

#include <cmath>
#include "ctre/Phoenix.h"
#include "ctre/phoenix6/TalonFX.hpp"
#include "../drivers/LazyTalonFX.h"
#include "../util/Functions.h"

using namespace ctre::phoenix6;


#define MOTOR_ENCODER_RESOLUTION 2048.0
#define CANCODER_RESOLUTION 4096.0

#define WHEEL_DIAMETER 99.0

#define SWERVE_GEAR_RATIO 0
#define SWERVE_GEAR_RATIO_NUMERATOR 150.0
#define SWERVE_GEAR_RATIO_DENOMINATOR 7.0

#define DRIVE_GEAR_RATIO 3.93518519
#define DRIVE_GEAR_RATIO_NUMERATOR 0
#define DRIVE_GEAR_RATIO_DENOMINATOR 0


class WheelModule
{
private:
    // 驱动电机对象
    hardware::TalonFX *drive_moto1;
    configs::MotorOutputConfigs drive_config;
    controls::PositionDutyCycle position{0_tr};
    controls::MotionMagicDutyCycle motionMagic{0_tr};
    controls::DutyCycleOut persentOutput{0};
    controls::VelocityDutyCycle velocity{0_tps};
    controls::Follower follower{4,0};
    
    // 转向电机对象
    LazyTalonFX *servo_moto;

    // CANCoder对象
    CANCoder *enco_senc;

    // 舵轮模块CANCoder传感器的磁柱绝对角度零点偏移量
    double offset;
    double offset_Inter;

    bool m_isSwerveMotorInvert;
    bool m_isDriveMotorInvert;
    
public:    
    WheelModule(int driveID1, int servoID, int encoID, double _offset, std::string canbus = "",
                bool isSwerveMotorInvert = false, bool isDriveMotorInvert = false)
    {
        offset = _offset;
        m_isSwerveMotorInvert = isSwerveMotorInvert;
        m_isDriveMotorInvert = isDriveMotorInvert;

        drive_moto1 = new hardware::TalonFX(driveID1,canbus);
        servo_moto = new LazyTalonFX(servoID,canbus);
        enco_senc = new CANCoder(encoID,canbus);

        persentOutput.UpdateFreqHz = 1000_Hz;
    }

    ~WheelModule(){}

    /**
     * @brief 轮组初始化
    */
    void Wheel_Init(){
        double AbsPosition = enco_senc->GetAbsolutePosition();
        enco_senc->SetPosition(AbsPosition);
    }

    /**
     * @brief 速度闭环控制双电机
     * @param speed 旋转速度单位RPM
     * @bug 实际上车效果不好，加速过强导致抓地力变小，失控
    */
    void WheelSpeedPIDCtrl(double speed){
        speed = speed / 60.0;
        if(m_isDriveMotorInvert){
            drive_moto1->SetControl(velocity.WithVelocity(units::angular_velocity::turns_per_second_t(-speed)));
        }
        else{
            drive_moto1->SetControl(velocity.WithVelocity(units::angular_velocity::turns_per_second_t(speed)));
        }
        
    }

    /**
     * @brief 开环控制双电机
     * @param persent 输出百分数
    */
    void WheelPersentCtrl(double persent){
        // 提高刷新率，默认100_Hz提高到1000_Hz
        persentOutput.UpdateFreqHz = 1000_Hz;

        if(m_isDriveMotorInvert){
            drive_moto1->SetControl(persentOutput.WithOutput(-persent));
        }
        else{
            drive_moto1->SetControl(persentOutput.WithOutput(persent));
        }
        
    }

    /**
     * @brief 设置全向轮转角
     * @warning 注意angle是否用到了encoder
    */
    void setTargetAngle(double angle){
        double targetAngle = angle - offset;
        servo_moto->set(ControlMode::Position,targetAngle / 360.0 * CANCODER_RESOLUTION);
    }
    /**
     * @brief 设置全向轮转角
     * @warning 同时利用CANCoder和built-in Encoder进行转向
    */
    void setTargetAngle_Zero(double angle){
        // 如果用的是MK4I
        //（根据转向电机的安装进行正负调整） (ErrinDeg/360.0) * MOTOR_ENCODER_RESOLUTION * (转向减速比)
        if(m_isSwerveMotorInvert){
            double ErrinDeg = angle - enco_senc->GetAbsolutePosition() + offset;
            double Tar = servo_moto->superTalonFX->GetSelectedSensorPosition()
                        + (ErrinDeg/360.0) * MOTOR_ENCODER_RESOLUTION * (SWERVE_GEAR_RATIO_NUMERATOR / SWERVE_GEAR_RATIO_DENOMINATOR);
            servo_moto->set(ControlMode::Position,Tar);
            offset_Inter = Tar;
        }
        // 如果用的是MK4
        else{
            double ErrinDeg = angle - enco_senc->GetAbsolutePosition() + offset;
            double Tar = servo_moto->superTalonFX->GetSelectedSensorPosition()
                        - (ErrinDeg/360.0) * MOTOR_ENCODER_RESOLUTION * (SWERVE_GEAR_RATIO_NUMERATOR / SWERVE_GEAR_RATIO_DENOMINATOR);
            servo_moto->set(ControlMode::Position,Tar);
            offset_Inter = Tar;
        }
        
    }
    void setTargetAngle_withBicontrolAlgorithm(double angle){
        // 角度转换，angle*(转向减速比)
        if(m_isSwerveMotorInvert){
            angle = -1.0 * angle * (SWERVE_GEAR_RATIO_NUMERATOR / SWERVE_GEAR_RATIO_DENOMINATOR);
            servo_moto->set(ControlMode::Position, angle / 360.0 * MOTOR_ENCODER_RESOLUTION + offset_Inter);
        }
        else{
            angle = angle * (SWERVE_GEAR_RATIO_NUMERATOR / SWERVE_GEAR_RATIO_DENOMINATOR);
            servo_moto->set(ControlMode::Position, angle / 360.0 * MOTOR_ENCODER_RESOLUTION + offset_Inter);
        }
    }

    /**
     * @brief 电机运转模式，刹车
    */
    void Drive_Brake(){
        drive_moto1->SetNeutralMode(1);
    }

    /**
     * @brief 电机运转模式，滑行
    */
    void Drive_Coast(){
        drive_moto1->SetNeutralMode(0);
    }

    /**
     * @brief 设置底盘驱动电机位置
     * @param positionCounter 位置计数器
     * @param isMotionMagic 是否使用运动魔术（默认不使用）
    */
    void SetDriveMotorPosition(double positionCounter,bool isMotionMagic = false){
        if(isMotionMagic){
            motionMagic.Slot = 1;
            
            drive_moto1->SetControl(motionMagic.WithPosition(units::angle::turn_t(positionCounter/MOTOR_ENCODER_RESOLUTION)));
            

        }
        else{
            position.Slot = 0;
            drive_moto1->SetControl(position.WithPosition(units::angle::turn_t(positionCounter/MOTOR_ENCODER_RESOLUTION)));
            
        }
    }

    void SetDriveMotorPositionSlot(double positionCounter,int pidSlot){
        position.Slot = pidSlot;
        drive_moto1->SetControl(position.WithPosition(units::angle::turn_t(positionCounter/MOTOR_ENCODER_RESOLUTION)));
        
    }

    void SetDriveMotorFollow(int followId,bool ifInverse){
        follower.MasterID = followId;
        if(m_isDriveMotorInvert){
            follower.OpposeMasterDirection = ifInverse;
        }
        else{
            follower.OpposeMasterDirection = !ifInverse;
        }
        drive_moto1->SetControl(follower);
    }
    
    /**
     * @brief 更改驱动电机PID参数槽位
     * @param slotID 电机槽位
     * @param isMotionMagic 此修改是否用于运动魔术
    */
    void ChangeDrivePositionPIDSlot(int slotID,bool isMotionMagic = false){
        if(isMotionMagic){
            motionMagic.Slot = slotID;
        }
        else{
            position.Slot = slotID;
        }
    }

    /**
     * @brief 获取底盘信息
    */
    double GetCanCoderAngle(){
        return (enco_senc->GetAbsolutePosition()-offset);
    }
    double GetDriveConderVolecity(){
        return (drive_moto1->GetVelocity().GetValueAsDouble() / DRIVE_GEAR_RATIO * PI * WHEEL_DIAMETER / 1000.0);
    }
    double GetDriveMotorCurrent(){
        return drive_moto1->GetSupplyCurrent().GetValueAsDouble();
    }
    double GetDriveBusVoltage(){
        return drive_moto1->GetSupplyVoltage().GetValueAsDouble();
    }
    double GetDrivePosition(){
        return (drive_moto1->GetPosition().GetValueAsDouble()*MOTOR_ENCODER_RESOLUTION);
    }
    bool GetDriveIsInvert(){
        return m_isDriveMotorInvert;
    }

};