#pragma once

#include "frc/DigitalInput.h"
#include "ctre/Phoenix.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/Joystick.h>
#include "rev/CANSparkMax.h"
#include "frc/Servo.h"
#include "team8011/lib/swerve/AutoChassis.h"
#include "team8011/lib/controlboard/JoyStickPorts.h"
#include "frc/Servo.h"


class BallManager
{
private:
    /* data */
    LazyTalonFX shoot1{10};
    LazyTalonFX shoot2{11};
    LazyTalonFX shootPath{16};
    LazyTalonFX Pitch{21};
    LazyTalonFX Intake{22};
    LazyTalonFX Elevator{23};
    LazyTalonFX Elevator2{24};
    LazyTalonFX Trip{31};
    
    frc::Servo eleLock{7};
    frc::Servo elePush{0};
    frc::Servo eleTrmp{3};

    rev::CANSparkMax sparkMax{12, rev::CANSparkMax::MotorType::kBrushless};

    frc::Joystick sub_joy{1};
    frc::Joystick test_main_joy{0};

    frc::DigitalInput EveLim{0};
    frc::DigitalInput BallPipe{1};
    frc::DigitalInput ballTrumpLim{4};

    double centerAngUnit = 1613;
    double offsett = 0;
    bool isReset = 0;
    bool ClimberAMP = 0;
    double sparkPosition = 0;
    bool issubready = 0;

    bool isFarShoot = 0;

    double talonPosition;

    // 是否由吸球机构吸入，吸球后第一次碰到传感器
    bool isBallIn = 0;

    // 球是否第一次经过传感器
    bool isBallPass = 0;

    // 是否吸球，吸球许可
    bool isIntake = 0;

    bool istrumpballIn = 0;
    bool istrumpballPass = 0;

    double pi=3.1415926535;

    double Speed,Angle,SpeedErr,lastSpeed,lastAngle;

public:
    BallManager(){
        centerAngUnit = 1613;

    }
    ~BallManager(){

    }
    void Subsys_Init(){
        talonPosition = shootPath.superTalonFX->GetSelectedSensorPosition();
        isBallIn = 0;
        isBallPass = 0;
        isIntake = 0;
    }
    void Subsys_Dis(){
        isReset = 0;
    }

    void Climber(){
        double Climb = sub_joy.GetRawAxis(JoyStickPorts::Axis::RA);
        bool ClimberEnable = sub_joy.GetRawButton(JoyStickPorts::Button::RB);
        ClimberAMP = sub_joy.GetRawButton(JoyStickPorts::Button::LB);
        if(!EveLim.Get()){
            offsett = Elevator.superTalonFX->GetSelectedSensorPosition();
            isReset = 1;
        }

        // 爬升程序，复位并使能，并且不是AMP情况
        if(isReset&&(!ClimberAMP)&&(sub_joy.GetPOV() != -1)){
            if(sub_joy.GetPOV() == 0){
                Elevator.ChangePID_Slot(0);
                Elevator2.ChangePID_Slot(0);
                Elevator.set(ControlMode::MotionMagic,offsett+45000);
                Elevator2.set(ControlMode::Follower,23);

                Pitch.set(ControlMode::Position,centerAngUnit+45.0/360.0*4096);

                shoot1.set(ControlMode::Disabled,0);
                shoot2.set(ControlMode::Disabled,0);
                shootPath.set(ControlMode::Disabled,0);

                elePush.Set(0.5);

            }
            else if(sub_joy.GetPOV() == 180){
                Pitch.set(ControlMode::Position,centerAngUnit+45.0/360.0*4096);

                shoot1.set(ControlMode::Disabled,0);
                shoot2.set(ControlMode::Disabled,0);
                shootPath.set(ControlMode::Disabled,0);

                if(Pitch.superTalonFX->GetSelectedSensorPosition()>centerAngUnit+2.0/360.0*4096&&Pitch.superTalonFX->GetSelectedSensorPosition()<centerAngUnit+60/360.0*4096){
                    Elevator.ChangePID_Slot(1);
                    Elevator2.ChangePID_Slot(1);
                    Elevator.set(ControlMode::Position,offsett);
                    Elevator2.set(ControlMode::Follower,23);
                }
                else{
                    Elevator.ChangePID_Slot(1);
                    Elevator2.ChangePID_Slot(1);
                    Elevator.set(ControlMode::Position,offsett+10000);
                    Elevator2.set(ControlMode::Follower,23);
                }
                if(sub_joy.GetRawButton(JoyStickPorts::Button::THREE_LINES)){
                    eleLock.Set(0.5);
                }
                else{
                    eleLock.SetDisabled();
                }
            }
            else{
                Elevator.ChangePID_Slot(0);
                Elevator2.ChangePID_Slot(0);
                Elevator.set(ControlMode::Disabled,0);
                Elevator2.set(ControlMode::Follower,23);
                elePush.SetDisabled();
            }
        }
        // AMP程序，复位并使能，并且不是爬升情况
        else if(isReset&&ClimberAMP&&(sub_joy.GetPOV() == -1)){
            if(Climb>0.2){
                // 电梯向上
                Elevator.ChangePID_Slot(0);
                Elevator2.ChangePID_Slot(0);
                Elevator.set(ControlMode::MotionMagic,offsett+45000);
                Elevator2.set(ControlMode::Follower,23);

                // 已经升高向下射球
                if(Elevator.superTalonFX->GetSelectedSensorPosition()>offsett+10000){
                    Pitch.set(ControlMode::Position,centerAngUnit-25.0/360.0*4096);
                    shoot1.set(ControlMode::Velocity,9500);
                    shoot2.set(ControlMode::Velocity,9500);
                    
                }

                // 未升高，向上12度
                else{
                    Pitch.set(ControlMode::Position,centerAngUnit+15.0/360.0*4096);
                }

                if(sub_joy.GetRawButton(JoyStickPorts::Button::Y)){

                    shootPath.set(ControlMode::PercentOutput,0.4);
                    // 射球预热
                    shoot1.set(ControlMode::Velocity,9500);
                    shoot2.set(ControlMode::Velocity,9500);
                    isIntake = 0;
                    isBallIn = 0;
                    isBallPass = 0;

                }

            }
            else{
                Pitch.set(ControlMode::Position,centerAngUnit+15.0/360.0*4096);

                shoot1.set(ControlMode::Disabled,0);
                shoot2.set(ControlMode::Disabled,0);
                shootPath.set(ControlMode::Disabled,0);

                if(Pitch.superTalonFX->GetSelectedSensorPosition()>centerAngUnit+2.0/360.0*4096&&Pitch.superTalonFX->GetSelectedSensorPosition()<centerAngUnit+60/360.0*4096){
                    Elevator.ChangePID_Slot(0);
                    Elevator2.ChangePID_Slot(0);
                    Elevator.set(ControlMode::MotionMagic,offsett);
                    Elevator2.set(ControlMode::Follower,23);
                }
                else{
                    Elevator.ChangePID_Slot(0);
                    Elevator2.ChangePID_Slot(0);
                    Elevator.set(ControlMode::MotionMagic,offsett+10000);
                    Elevator2.set(ControlMode::Follower,23);
                }
            }
        }
        else{
            Elevator.ChangePID_Slot(0);
            Elevator2.ChangePID_Slot(0);
            Elevator.set(ControlMode::Disabled,0);
            Elevator2.set(ControlMode::Follower,23);
        }
    }

    /**
     * @param b 射球预热键，边瞄准边预热，未瞄准，副操预热
    */
    void shootball(double distance,double b = -1){
        /**
         * @brief fire test
         */
        frc::SmartDashboard::PutNumber("distance",distance);
        
        // double b = sub_joy.GetRawAxis(2);
        // 默认情况副操预热
        if(b == -1){
            b = sub_joy.GetRawAxis(JoyStickPorts::Axis::LA);
            issubready = 1;
        }
        else{
            issubready = 0;
        }

        // 角度调节拟合
        Angle = 50;

        Speed = 12000;
        
        SpeedErr = 200;

        // distance = 5;

        // 设置速度区间
        if(distance != 0 && distance < 9.2){

            // 速度线性递增表达式
            Speed = (10000 + (distance - 1.6) * 1000)*1.21;
            // Angle = 244.1*sin(0.1138*distance+2.4)+78.85*sin(0.3221*distance+4.399);
            Angle = 161.7*sin(0.1054*distance+2.262)+50.73*sin(0.342*distance+3.899);
            // Angle = Angle-((5-distance)*(5-distance))*0.5-(5-distance);

            frc::SmartDashboard::PutNumber("Angle",Angle);
            frc::SmartDashboard::PutNumber("dis",distance);

            if(issubready){
                // 默认速度
                if(test_main_joy.GetRawButton(JoyStickPorts::Button::RB)){
                    Speed = 11100;
                    Angle = 60;
                }
                if(test_main_joy.GetRawButton(JoyStickPorts::Button::LB)){
                    Speed = 16000;
                    Angle = 50;
                }
            }

        }
        else{
            
            // 默认速度
            Angle = lastAngle;
            Speed = lastSpeed;
            if(test_main_joy.GetRawButton(JoyStickPorts::Button::RB)){
                Speed = 11100;
                Angle = 60;
            }
            if(test_main_joy.GetRawButton(JoyStickPorts::Button::LB)){
                Speed = 16000;
                Angle = 50;
            }
            // 3500 60
            frc::SmartDashboard::PutNumber("Angle",Angle);
            frc::SmartDashboard::PutNumber("dis",distance);
        }

        /**
         * @brief B键吸球 自动代码（按一次一直转，直到识别,或者按下A）
        */

        if(test_main_joy.GetRawButton(JoyStickPorts::Button::LB)){
            isFarShoot = 1;
        }
        if(test_main_joy.GetRawButton(JoyStickPorts::Button::RB)){
            isFarShoot = 0;
        }
        if(sub_joy.GetRawButton(JoyStickPorts::Button::B))
        {
            isIntake = 1;
        }
        if(sub_joy.GetRawButton(JoyStickPorts::Button::A)){
            isIntake = 0;
        }

        // 可以吸球
        if(isIntake){
            // 未吸到球，吸球，如果吸到停止吸球
            if(!isBallIn)
            {
                Intake.set(ControlMode::PercentOutput,1);
            }
            else{
                Intake.set(ControlMode::Disabled,0);
            }
            
            // 默认吸球角度
            if(!ClimberAMP){
                Pitch.set(ControlMode::Position,centerAngUnit+30/360.0*4096);
            }

            // 如果传感器第一次检测到了，说明球进入射球装置
            if(!BallPipe.Get()){
                if (!isBallIn) 
                {
                    isBallIn = 1;
                }
            }

            // 如果检测到球经过传感器，记录电机位置
            if(BallPipe.Get()&&isBallIn){
                if(!isBallPass){
                    isBallPass = 1;
                    talonPosition = shootPath.superTalonFX->GetSelectedSensorPosition()-500;
                }
            }
            // 如果球未经过传感器，继续向前送球，如经过，锁住球
            if(!isBallPass){
                shootPath.set(ControlMode::PercentOutput,0.2);
            }
            else{
                shootPath.set(ControlMode::Position,talonPosition);
            }

        }
        else{
            // 停止吸球
            Intake.set(ControlMode::Disabled,0);

            // 球道解锁
            if(!ClimberAMP){
                shootPath.set(ControlMode::Disabled,0);
            }
        }

        lastSpeed = Speed;
        lastAngle = Angle;

        /**
         * @brief 强制预热，射球
        */
        if(b>0.2&&(!ClimberAMP)){
            isBallIn = 0;
            isBallPass = 0;
            isIntake = 0;
            
            // 67.5上限，10下限,超出限制默认45
            if(Angle <=67.5 && Angle>10){
                Angle = Angle;
            }
            else{
                Angle = 45;
            }

            Pitch.set(ControlMode::Position,centerAngUnit+Angle/360.0*4096);

            shoot1.set(ControlMode::Velocity,Speed);
            shoot2.set(ControlMode::Velocity,Speed);

            if(Speed - shoot1.superTalonFX->GetSelectedSensorVelocity() < SpeedErr){
                frc::SmartDashboard::PutNumber("Can Shoot ?",1);
                frc::SmartDashboard::PutNumber("velocity",shoot1.superTalonFX->GetSelectedSensorVelocity());

                // 射球
                if(sub_joy.GetRawButton(JoyStickPorts::Button::Y)){
                    shootPath.set(ControlMode::PercentOutput,0.4);
                }
                else{
                    shootPath.set(ControlMode::PercentOutput,0);
                }
                if(test_main_joy.GetRawAxis(JoyStickPorts::Axis::RA)){
                    shootPath.set(ControlMode::PercentOutput,0.4);
                }
                
            }
            else{
                frc::SmartDashboard::PutNumber("Can Shoot ?",0);
            }
            
        }
        else if(!ClimberAMP)
        {
            if(sub_joy.GetRawButton(JoyStickPorts::Button::RB)){
                shoot1.set(ControlMode::Velocity,-10000);
                shoot2.set(ControlMode::Velocity,-10000);
                shootPath.set(ControlMode::PercentOutput,-1);
                Intake.set(ControlMode::PercentOutput,-1);
            }else{
                shoot1.set(ControlMode::PercentOutput,-0.06);
                shoot2.set(ControlMode::PercentOutput,-0.06);
            }
            
        }

        //重置isBallIn
        if(sub_joy.GetRawButton(JoyStickPorts::Button::X)){
            isBallIn = 0;
            isBallPass = 0;
        }
        Trump();
    }


    void setBallInOut(bool ballPos){
        isBallIn = ballPos;
    }
    /**
     * @brief 检测球道是否有球
    */
    bool Ready2shoot(){
        return isBallPass;
    }
    void Trump(){
        if(isReset){
            if(sub_joy.GetRawButton(JoyStickPorts::Button::TWO_WINDOWS)){
                eleTrmp.Set(0.5);
                Elevator.ChangePID_Slot(2);
                Elevator2.ChangePID_Slot(2);
                Elevator.set(ControlMode::Position,offsett+16000);
                Elevator2.set(ControlMode::Follower,23);
                if(Elevator.superTalonFX->GetSelectedSensorPosition()>offsett+12500){
                    // Pitch.set(ControlMode::Position,1250);
                    Pitch.set(ControlMode::Position,centerAngUnit+-10/360.0*4096);

                    // 如果球没过去，开启小Neo
                    if(!istrumpballPass){
                        sparkMax.Set(-0.25);
                    }
                    // 球已经过去了，关闭小Neo，Pitch上扬，电梯下滑
                    else{
                        sparkMax.Set(0);
                        Pitch.set(ControlMode::Position,centerAngUnit+45.0/360.0*4096);
                    }

                    // 如果球碰到了限位开关，球已进入，球道停止
                    if(!ballTrumpLim.Get()){
                        shootPath.set(ControlMode::Disabled,0);
                        istrumpballIn = 1;
                    }
                    // 如果没有进入，球道反转使其进入，注意姿态
                    else{
                        if((Elevator.superTalonFX->GetSelectedSensorPosition()>offsett+15500)&&(fabs(Pitch.superTalonFX->GetSelectedSensorPosition()-(centerAngUnit+-10/360.0*4096))<50)){
                            shootPath.set(ControlMode::PercentOutput,-1);
                        }
                    }

                    // 在已经进入球道前提下，检测球是否通过
                    if(istrumpballIn){
                        if(ballTrumpLim.Get()){
                            istrumpballPass = 1;
                        }
                    }
                }
                else{
                    Pitch.set(ControlMode::Position,centerAngUnit+15.0/360.0*4096);
                }
                
            }
            else{
                sparkMax.Set(0);
                eleTrmp.SetDisabled();
                istrumpballIn = 0;
                istrumpballPass = 0;
            }
        }

        // 陷阱零位，最下方
        double trumpcenter = 2454;
        // 向上打陷阱
        if(sub_joy.GetPOV() == 180){
            // 向上打陷阱
            Trip.set(ControlMode::MotionMagic,trumpcenter-2370);
            if(Trip.superTalonFX->GetSelectedSensorPosition()<(trumpcenter-2370) + 50){
                Trip.set(ControlMode::Disabled,0);
            }
            if(test_main_joy.GetPOV() == 0){
                sparkMax.Set(1);
            }
            else{
                sparkMax.Set(0);
            }
            istrumpballIn = 0;
            istrumpballPass = 0;
        }
        else{
            // 回收陷阱
            if(fabs(Trip.superTalonFX->GetSelectedSensorPosition()-trumpcenter)>20){
                Trip.set(ControlMode::MotionMagic,trumpcenter);
            }
            else{
                Trip.set(ControlMode::Disabled,0);
            }
        }
    }

};
