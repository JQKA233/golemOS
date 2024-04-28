#include <frc/Joystick.h>
#include <frc/Compressor.h>
#include <frc/Solenoid.h>
#include <frc/DigitalInput.h>
#include <frc/motorcontrol/PWMTalonFX.h>
#include <rev/ColorSensorV3.h>
#include "ctre/Phoenix.h"
#include "networktables/NetworkTable.h"
#include "networktables/NetworkTableInstance.h"
#include "networktables/NetworkTableEntry.h"
#include "networktables/NetworkTableValue.h"
#include "team8011/lib/controlboard/JoyStickports.h"

#define PICH_MAX_ABSDEGREE -53
#define PICH_MIN_ABSDEGREE -81
#define YAW_MAX_ABSDEGREE 90
#define YAW_MIN_ABSDEGREE -90
#define FOUC_BY_PIX 1000


enum BallColor{
    BLUE_BALL,
    RED_BALL
};
class BallPipe
{
private:
    frc::Joystick sub_joy{1};
    frc::DigitalInput inp{5};
    frc::Compressor pump_8011{18,frc::PneumaticsModuleType::CTREPCM};
    frc::Solenoid sole_8011{18,frc::PneumaticsModuleType::CTREPCM,1};
    frc::PWMTalonFX collect_moto{8};
    frc::I2C::Port portI2C;
    rev::ColorSensorV3 col_sen{frc::I2C::kOnboard};
    bool color_ball;//存储球的颜色
    frc::PWMTalonFX color_moto{9};//向上吸球
    frc::PWMTalonFX shoot_moto1{0};
    frc::PWMTalonFX shoot_moto2{1};
    TalonFX *yaw_moto;
    frc::DigitalInput swti1{9};
    double pich;
    double yaw;
    std::shared_ptr<nt::NetworkTable> table;
    bool IfAim;
    double targetOffsetAngle_Horizontal;
    double targetOffsetAngle_Vertical;
    double targetArea;
    double targetSkew;
    double tarRecipDistance;

    double ptz_last_yaw;
    double ptz_pre_yaw;
    
    double last_yaw;
    double pre_yaw;

    double last_pitch;
    double pre_pitch;

public:
    double yaw_offset;
    double pich_offset;
    BallPipe(bool _color_ball){
        color_ball = _color_ball;
        yaw_moto = new TalonFX(30);
    }
    ~BallPipe(){
    }
    bool IsThereBall_U(){
        if(!inp.Get()){
            return false;
        }
        else{
            return true;
        }
    }
    void JudgeBalls(){
        if (col_sen.IsConnected())
        {
            if(color_ball==BLUE_BALL){
                // frc::SmartDashboard::PutNumber("put",col_sen.GetColor().red);
                if(col_sen.GetColor().red>0.3){
                    color_moto.Set(0);
                }
                else if(col_sen.GetColor().blue>0.3){
                    color_moto.Set(-1);
                }
            }
            if(color_ball==RED_BALL){
                if(col_sen.GetColor().blue>0.3){
                    color_moto.Set(0);
                }
                else if(col_sen.GetColor().red>0.3){
                    color_moto.Set(-1);
                }
            }
        }
        else{
            ;// 可能出现的问题
        }
    }
    bool IsThereBall_D(){
        if(col_sen.IsConnected()){
            if(color_ball==BLUE_BALL){
                if(col_sen.GetColor().blue>0.3){
                    return true;
                }
                else{
                    return false;
                }
            }
            if(color_ball==RED_BALL){
                if(col_sen.GetColor().red>0.3){
                    return true;
                }
                else{
                    return false;
                }
            }
        }
        else{
            return false;
        }
    }
    void MoveBallsDown(){
        color_moto.Set(0.1);
    }
    void MoveBallsUp(){
        color_moto.Set(-1);
    }
    void StopMoveBallsUp(){
        color_moto.Set(0);
    }
    void ReadyShoot_Half(){
        shoot_moto1.Set(-0.8);
        shoot_moto2.Set(0.8);
    }
    void ShootBalls(){
        shoot_moto1.Set(-0.8);
        shoot_moto2.Set(0.8);
    }
    void StopBalls(){
        shoot_moto1.Set(0);
        shoot_moto2.Set(0);
    }
    /**
     * @brief 收球，启动收球电机和气缸，使收球机构伸出
    */
    void ReadyCollect(double W = 0){
        collect_moto.Set(W);
        
    }
    /**
     * @brief 停止收球回收收球机构并停止电机运行
    */
    void StopCollect(){
        collect_moto.Set(0);
    }
    void GetVisionInfo(){
        table = nt::NetworkTableInstance::GetDefault().GetTable("limelight");//创建网络表实例

        IfAim = table->GetNumber("tv",0.0);
        targetOffsetAngle_Horizontal = table->GetNumber("tx",0.0);
        targetOffsetAngle_Vertical = table->GetNumber("ty",0.0);
        targetArea = table->GetNumber("ta",0.0);
        targetSkew = table->GetNumber("ts",0.0);
        tarRecipDistance = table->GetNumber("tlong",0.0) / 127 / FOUC_BY_PIX;//计算距离倒数（一定有意义）
    }
    void SetYaw_Vision(){
        double yaw_absADDtar = -targetOffsetAngle_Horizontal;//pich轴绝对位置加目标点位移，即目标位置
        double tar = yaw_absADDtar*2048*(250*52)/(14*14)/360+yaw_moto->GetSelectedSensorPosition();
        if(IfAim){
            if( tar > (43000+yaw_offset)-41000 && tar < (43000+yaw_offset)+41000 )//目标位置在区间内
            {
                    yaw_moto->Set(ControlMode::Position,tar);
            }
            else{
                if(tar < (43000+yaw_offset)-41000){
                    yaw_moto->Set(ControlMode::Position,43000+yaw_offset);
                }
                if(tar > (43000+yaw_offset)+41000){
                    yaw_moto->Set(ControlMode::Position,43000+yaw_offset);
                }
            }
        }
        else{
            // 摇头
        }
    }
    void BallsManager(){
        if(sub_joy.GetRawAxis(JoyStickPorts::Axis::RA)>0.05){
            if((!IsThereBall_U())){//上无球
                ReadyCollect(0.8);
                sole_8011.Set(true);
                JudgeBalls();
            }
            if((IsThereBall_U()) && (!IsThereBall_D())){//上有球，但下无球
                ReadyCollect(0.8);
                sole_8011.Set(true);
                StopMoveBallsUp();
            }
            if((IsThereBall_U()) && (IsThereBall_D())){//上下都有球
                StopCollect();
                sole_8011.Set(false);
                StopMoveBallsUp();
            }
        }
        else{
            StopCollect();
            sole_8011.Set(false);
            StopMoveBallsUp();
        }
        if(sub_joy.GetRawAxis(JoyStickPorts::Axis::LA)){//ready to shoot
            ReadyShoot_Half();
            ShootBalls();
            if(sub_joy.GetRawButton(JoyStickPorts::Button::B)){//shoot
                ReadyCollect(1);
                sole_8011.Set(true);
                MoveBallsUp();
            }
        }
        else{
            StopBalls();
        }
        if(0){
            //定Pich动Yaw
            // cha.ptz.SetPich_M();
            // cha.ptz.SetYaw_M(sub_joy.GetX());
        }
        else{
            GetVisionInfo();
            SetYaw_Vision();
            // cha.ptz.SetPich_Vision();
        }
        if(sub_joy.GetRawButton(JoyStickPorts::Button::RB)){
            if(sub_joy.GetRawButton(JoyStickPorts::Button::LB)){
                ReadyCollect(-0.7);
                sole_8011.Set(true);
            }
            else{
                ReadyCollect(0.7);
                sole_8011.Set(true);
            }
        }
        if(sub_joy.GetRawButton(JoyStickPorts::Button::A)){
            MoveBallsDown();
            ReadyShoot_Half();
        }
    }
    void PTZmove_Init(){
        // pich_offset = pich_moto->GetSelectedSensorPosition();
        while(swti1.Get()){
            yaw_moto->Set(ControlMode::PercentOutput,-0.07);
        }
        yaw_moto->Set(ControlMode::PercentOutput,0);
        yaw_offset = yaw_moto->GetSelectedSensorPosition();
        // frc::SmartDashboard::PutNumber("uorP",yaw_offset);
        // ptz_last_yaw = gyro_ptz->GetYaw();//后面还要加上归中角度
        // 2048u/_deg
        // 13.7549 9117         -36.5625   56062      22.8076      46945
        yaw_moto->Set(ControlMode::Position,(43000+yaw_offset));//归中
    }
};

