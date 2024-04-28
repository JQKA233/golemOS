#pragma once

#include <frc/Joystick.h>
#include <frc/AddressableLED.h>
#include <cmath>
#include <frc/smartdashboard/SmartDashboard.h>
#include "ctre/Phoenix.h"
#include "WheelModule.h"
#include <frc/Timer.h>
#include "team8011/lib/controlboard/JoyStickPorts.h"


#define CAR_LENGTH 0.5//车两轮中心长 m
#define CAR_WIDTH 0.5//车两轮中心宽 m
#define SPEED_V_LIMIT 1//最大速度限制%
#define SPEED_W_LIMIT 1//最大角速度限制%


enum DriveControlState {
    FORCE_ORIENT,
    OPEN_LOOP,
    HEADING_CONTROL,
    VELOCITY,
    PATH_FOLLOWING,
    AUTO_BALANCE
};
//定义速度矢量
typedef struct 
{
    double v;
    double yaw;
}SpeedVec;

//wheel handle
// 记录上一次和本次舵轮模组矢量
typedef struct
{
    SpeedVec perVector;
    SpeedVec lastVector;
}Wheel_h;

SpeedVec add(SpeedVec *a,SpeedVec *b);//定义矢量相加

class Chassis
{
private:
    double car_V;
    double car_yaw;

    //轮子的last值与persent值
    Wheel_h Wheel_lp[4];

    //轮子速度与转角
    SpeedVec Wheel_info[4];

    //自转时轮偏角，正方形时为45°atan(CAR_LENGTH / CAR_WIDTH)
    double Circle_rad = PI/4;

    //底盘与云台yaw偏差
    double yaw_dif;

    //底盘yaw
    double Chassis_yaw;

    //云台yaw
    double PTZ_yaw;

    // TurnToCertainAngle积分器
    double IntB = 0;

    double Int_fastTurn = 0;
public:
    //BL ,BR
    //FL ,FR
    //四个全向轮对象
    /**
     * @param _offset WheelModule中的构造函数中的_offset参数，
     *                在CANcoder的self-test中读取Position角度
    */
    WheelModule GIMwheel[4]={WheelModule(4,5,2,209.785,"drive",false,false),WheelModule(6,7,3,344.619,"drive",false,false)
            ,WheelModule(0,1,0,247.676,"drive",true,false),WheelModule(2,3,1,119.268,"drive",false,true)};

    //陀螺仪对象
    WPI_Pigeon2 gyro{33,"drive"};


    double gyro_last_yaw;
    double gyro_pre_yaw;
    double first_yaw_maga;
    double targetAngle_gyro = 0;
    double pre_speed_w = 0;
    bool isReset = 0;

    Chassis()
    {
        PTZ_yaw=0;
        Chassis_yaw=0;
        yaw_dif=0;
        Circle_rad=atan(CAR_LENGTH / CAR_WIDTH);
        gyro.ConfigFactoryDefault();
        gyro.ZeroGyroBiasNow();
    }

    ~Chassis()
    {
        
    }

    SpeedVec add(SpeedVec *a,SpeedVec *b)
    {
        SpeedVec temp;
        double x = a->v * cos(a->yaw) + b->v * cos(b->yaw);
        double y = a->v * sin(a->yaw) + b->v * sin(b->yaw);
        temp.v = sqrt(x * x + y * y);
        temp.yaw = atan2(y, x);
        return temp;
    }

    /**
     * @brief 初始化磁场定向控制，以第三人称视角设置
    */
    void InitMega()
    {
        gyro.Reset();
        gyro.SetYaw(0);
        isReset = 1;
    }
    /**
     * @brief 初始化磁场定向控制，以第三人称视角设置,反向
    */
    void InitMega_R(){
        gyro.Reset();
        gyro.SetYaw(180);
        isReset = 1;
    }

    /**
     * @brief 初始化底盘，舵向归中
    */
    void Chassis_Init()
    {
        for (int i = 0; i < 4; i++)
        {
            GIMwheel[i].Wheel_Init();
            Wheel_lp[i].lastVector.v = 0;
            Wheel_lp[i].perVector.v = 0;
        }
        
        //舵轮底盘归零
        SetChassisZero();

        //初始化磁场定向控制，第三人称
        gyro.Reset();
        gyro.SetYaw(0);
        isReset = 1;

        //初始化陀螺仪
        gyro_last_yaw = gyro.GetYaw();
    }

    /**
     * @brief 直线陀螺仪纠偏，防止在直线模式下自旋
    */
    double CorrectYaw(double targetAngle,double currentAngle,double currentAngularRate)
    {
        /* grab some input data from Pigeon and gamepad*/
        double RCW_error= (targetAngle - currentAngle) * (-0.009) - (currentAngularRate) * (-0.001);
        RCW_error=Cap(RCW_error,0.75);
        RCW_error=DB(RCW_error);
        return RCW_error;
    }

    /**
     * @brief 输入底盘运动状态参量，设置底盘运动状态(舵轮运动学)
     * @param speed_x x方向运动速度百分比
     * @param speed_y y方向运动速度百分比
     * @param speed_w 旋转角速度百分比
     * @param extra_w 额外角速度，可以替换speed_w，自瞄角速度
    */
    void CalSpeed(double speed_x,double speed_y,double speed_w,bool isDeadArea,double extra_w)
    {
        // 如果启用了自瞄模式，则extra_w大于零，手动旋转摇杆（右摇杆）失效
        // 考虑需不需要死区限位，如果操作手按下了自瞄，证明不管额外角速度多小他都想去使用
        // 而不用自瞄，则extra_w一定为零
        if(extra_w != 0){
            speed_w = extra_w;
            double currentAngle = gyro.GetAngle() - 90;

            //得到角度变化率，用于PID的D参数
            double currentAngularRate = gyro.GetRate();

            //将角度转化为弧度，移植代码时注意根据硬件情况调整正负
            double angle =+ (currentAngle * PI) / 180;

            // 计算摇杆旋转量
            double temp = speed_y * cos(angle) + speed_x * sin(angle);
            speed_x = -1 * speed_y * sin(angle) + speed_x * cos(angle);
            speed_y = temp;

            // 旋转停止后更新陀螺仪角度
            if(speed_w == 0.0 && pre_speed_w != 0.0){
                targetAngle_gyro = currentAngle;
            }
            // 重新确定磁场定向驱动正方向
            if(isReset){
                isReset = 0;
                targetAngle_gyro = currentAngle;
            }
            //更新角速度
            pre_speed_w = speed_w;
            // 如果是直线模式，给出角速度修正量，以保证不自旋
            if(speed_w == 0.0){
                speed_w = CorrectYaw(targetAngle_gyro,currentAngle,currentAngularRate);
            }

            if(!isDeadArea){
                for (int i = 0; i < 4; i++) {

                    //记录上次速度
                    Wheel_lp[i].lastVector = Wheel_lp[i].perVector;
                }
            }

            car_V = sqrt(speed_x * speed_x + speed_y * speed_y);
                
            //最大速度限制
            if (car_V > SPEED_V_LIMIT) car_V = SPEED_V_LIMIT;
                
            car_yaw = R2DEG(atan2(speed_y, speed_x));

            //角度处理
            if (abs(car_yaw > 90)){
                car_yaw = car_yaw - 180;
                car_V = -car_V;
            }
            if (abs(car_yaw < -90)){
                car_yaw = car_yaw + 180;
                car_V = -car_V;
            } 
            
            car_yaw = DEG2R(car_yaw);

            for (int i = 0; i < 4; i++)
            {
                Wheel_lp[i].perVector.yaw = car_yaw;
                Wheel_lp[i].perVector.v = car_V;
            }

            if (speed_w != 0) 
            {
                //速度赋值PTZ_yaw
                Wheel_info[0].v = -speed_w;
                Wheel_info[1].v = speed_w;
                Wheel_info[2].v = -speed_w;
                Wheel_info[3].v = speed_w;
            
                //计算轮子垂直向量坐标的角度朝向，在需要控制的云台坐标系中,use for circle self
                Wheel_info[0].yaw = PTZ_yaw + Circle_rad;
                Wheel_info[1].yaw = PTZ_yaw - Circle_rad;
                Wheel_info[2].yaw = PTZ_yaw - Circle_rad;
                Wheel_info[3].yaw = PTZ_yaw + Circle_rad;

        /*********************************坐标系转换**************************************/

                // 现在，车的移动方向是云台的方向
                for (int i = 0; i < 4; i++) {
                    Wheel_lp[i].perVector.yaw += PTZ_yaw;
                }

                // 分别在云台坐标系中直线与垂直向量相加得到和矢量，然后换算回电机坐标系
                for (int i = 0; i < 4; i++) {
                    Wheel_lp[i].perVector = add(&(Wheel_lp[i].perVector), &(Wheel_info[i]));

                    //切换回电机yaw的坐标系
                    Wheel_lp[i].perVector.yaw = Wheel_lp[i].perVector.yaw - PTZ_yaw;
                }
            }
        }
        else
        {
            double currentAngle = gyro.GetAngle()-90;

            //得到角度变化率，用于PID的D参数
            double currentAngularRate = gyro.GetRate();

            //将角度转化为弧度，移植代码时注意根据硬件情况调整正负
            double angle =+ (currentAngle * PI) / 180.0;

            // 计算摇杆旋转量
            double temp = speed_y * cos(angle) + speed_x * sin(angle);
            speed_x = -1 * speed_y * sin(angle) + speed_x * cos(angle);
            speed_y = temp;

            // 旋转停止后更新陀螺仪角度
            if(speed_w == 0.0 && pre_speed_w != 0.0){
                targetAngle_gyro = currentAngle;
            }
            // 重新确定磁场定向驱动正方向
            if(isReset){
                isReset = 0;
                targetAngle_gyro = currentAngle;
            }
            //更新角速度
            pre_speed_w = speed_w;
            // 如果是直线模式，给出角速度修正量，以保证不自旋
            if(speed_w == 0.0){
                speed_w = CorrectYaw(targetAngle_gyro,currentAngle,currentAngularRate);
            }

            if(!isDeadArea){
                for (int i = 0; i < 4; i++) {

                    //记录上次速度
                    Wheel_lp[i].lastVector = Wheel_lp[i].perVector;
                }
            }

            car_V = sqrt(speed_x * speed_x + speed_y * speed_y);
                
            //最大速度限制
            if (car_V > SPEED_V_LIMIT) car_V = SPEED_V_LIMIT;
                
            car_yaw = R2DEG(atan2(speed_y, speed_x));

            //角度处理
            if (abs(car_yaw > 90)){
                car_yaw = car_yaw - 180;
                car_V = -car_V;
            }
            if (abs(car_yaw < -90)){
                car_yaw = car_yaw + 180;
                car_V = -car_V;
            } 
            
            car_yaw = DEG2R(car_yaw);

            for (int i = 0; i < 4; i++)
            {
                Wheel_lp[i].perVector.yaw = car_yaw;
                Wheel_lp[i].perVector.v = car_V;
            }

            if (speed_w != 0) 
            {
                //速度赋值PTZ_yaw
                speed_w *= 1.0;
                Wheel_info[0].v = -speed_w;
                Wheel_info[1].v = speed_w;
                Wheel_info[2].v = -speed_w;
                Wheel_info[3].v = speed_w;
            
                //计算轮子垂直向量坐标的角度朝向，在需要控制的云台坐标系中,use for circle self
                Wheel_info[0].yaw = PTZ_yaw + Circle_rad;
                Wheel_info[1].yaw = PTZ_yaw - Circle_rad;
                Wheel_info[2].yaw = PTZ_yaw - Circle_rad;
                Wheel_info[3].yaw = PTZ_yaw + Circle_rad;

        /*********************************坐标系转换**************************************/

                // 现在，车的移动方向是云台的方向
                for (int i = 0; i < 4; i++) {
                    Wheel_lp[i].perVector.yaw += PTZ_yaw;
                }

                // 分别在云台坐标系中直线与垂直向量相加得到和矢量，然后换算回电机坐标系
                for (int i = 0; i < 4; i++) {
                    Wheel_lp[i].perVector = add(&(Wheel_lp[i].perVector), &(Wheel_info[i]));

                    //切换回电机yaw的坐标系
                    Wheel_lp[i].perVector.yaw = Wheel_lp[i].perVector.yaw - PTZ_yaw;
                }
            }
        } 
    }

    /**
     * @brief 执行底盘速度值及舵向转角
     * @param speedScale 速度倍率，[0,1]，正常比赛速度置为1，调试可适当乘倍率
     * @param isDeadArea 是否位于死区
    */
    void RunSpeed(double speedScale ,bool isDeadArea)
    {
        for (int i = 0; i < 4; i++)
        {	
            //如果变换角度大于90°，反复循环直至小于90° 
            while(abs(Wheel_lp[i].perVector.yaw - Wheel_lp[i].lastVector.yaw) > DEG2R(90))
            {
                if (Wheel_lp[i].perVector.yaw < Wheel_lp[i].lastVector.yaw) {
                    Wheel_lp[i].perVector.yaw += DEG2R(180);
                }
                else {
                    Wheel_lp[i].perVector.yaw -= DEG2R(180);
                }
                Wheel_lp[i].perVector.v = -Wheel_lp[i].perVector.v;
            }
            
            if(isDeadArea){
                GIMwheel[i].WheelPersentCtrl(0);
                GIMwheel[i].setTargetAngle_withBicontrolAlgorithm(R2DEG(Wheel_lp[i].lastVector.yaw));
            }
            else{
                GIMwheel[i].WheelPersentCtrl(speedScale * Wheel_lp[i].perVector.v);
                GIMwheel[i].setTargetAngle_withBicontrolAlgorithm(R2DEG(Wheel_lp[i].perVector.yaw));
            }
        }
        
        frc::SmartDashboard::PutNumber("BL Ang",GIMwheel[0].GetCanCoderAngle());
        frc::SmartDashboard::PutNumber("BR Ang",GIMwheel[1].GetCanCoderAngle());
        frc::SmartDashboard::PutNumber("FL Ang",GIMwheel[2].GetCanCoderAngle());
        frc::SmartDashboard::PutNumber("FR Ang",GIMwheel[3].GetCanCoderAngle());

        frc::SmartDashboard::PutNumber("BL Vel",GIMwheel[0].GetDriveConderVolecity());
        frc::SmartDashboard::PutNumber("BR Vel",GIMwheel[1].GetDriveConderVolecity());
        frc::SmartDashboard::PutNumber("FL Vel",GIMwheel[2].GetDriveConderVolecity());
        frc::SmartDashboard::PutNumber("FR Vel",GIMwheel[3].GetDriveConderVolecity());
        
        frc::SmartDashboard::PutNumber("Velocity",(fabs(GIMwheel[0].GetDriveConderVolecity())+
                                                    fabs(GIMwheel[1].GetDriveConderVolecity())+
                                                    fabs(GIMwheel[2].GetDriveConderVolecity())+
                                                    fabs(GIMwheel[3].GetDriveConderVolecity()))/4.0);
    }

    /**
     * @brief 归零底盘时底盘舵向归中
    */
    void SetChassisZero()
    {
        for (int i = 0; i < 4; i++)
        {
            GIMwheel[i].setTargetAngle_Zero(0);
        }
    }
    void DisableControl(){
        for (int i = 0; i < 4; i++)
        {
            Wheel_lp[i].lastVector.yaw = 0;
            Wheel_lp[i].lastVector.v = 0;
            Wheel_lp[i].perVector.v = 0;
        }
        InitMega();
    }

    /**
     * @brief 转到特定角度，用于手动阶段对位，输出控制器
    */
    double TurnToCertainAngle(double TargetAngle_bootToZero,double angle){
        double tar = TargetAngle_bootToZero + angle;
        double err =  tar - gyro.GetAngle();

        // I Zoon 防止积分过饱和
        if(fabs(err)<2.5){
            IntB += err;
        }
        else{
            IntB = 0;
        }
        
        // 引入P控制，加限制幅值
        double ans = - 0.02 * err - 0.002 * IntB;
        return Cap(ans,0.5);
    }
    /**
     * @brief POV快速转向
     * @param TargetAngle_discrete_inPOV 目标角度，由POV给定
     * @remarks POV给出以90为倍数的角度，一般由ControlBoard::GetPOV_Pose()给出
    */
    double FastTurn(double TargetAngle_discrete_inPOV){
        if(TargetAngle_discrete_inPOV == -1){
            return 0;
        }
        else{
            double err =  TargetAngle_discrete_inPOV - gyro.GetAngle();
            double errtime = gyro.GetRate();
            while(fabs(err)>180){
                if(err > 180){
                    err -= 360;
                }
                if(err < -180){
                    err += 360;
                }
            }
            // // I Zoon
            // if(fabs(err)<90){
            //     Int_fastTurn += err;
            // }
            // else{
            //     Int_fastTurn = 0;
            // }
            
            // 引入P控制，加限制幅值
            double ans = - 0.0065 * err - 0 * Int_fastTurn + 0.0005 * errtime;
            return Cap(ans,0.7);
        }
    }
    /**
     * @brief ABXY转向
    */
    double FastTurn_withABXY(int abxy){
        if(abxy == JoyStickPorts::Button::A){
            return FastTurn(0);
        }
        if(abxy == JoyStickPorts::Button::Y){
            return FastTurn(180);
        }
        if(abxy == JoyStickPorts::Button::X){
            return FastTurn(90);
        }
        if(abxy == JoyStickPorts::Button::B){
            return FastTurn(-90);
        }
        if(abxy == 270){
            return FastTurn(-120);
        }
        if(abxy == 90){
            return FastTurn(120);
        }
        else{
            return 0;
        }
    }


};
