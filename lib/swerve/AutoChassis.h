#pragma once

#include "WheelModule.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/Timer.h>
#include "team8011/lib/swerve/Chassis.h"
#include "team8011/lib/vision/AprilTags.h"
#include "frc/DigitalInput.h"
#include "team8011/frc2024/subsystems/AutoSub.h"
#include "frc/Timer.h"

/**
 * 底盘朝向示意图（俯视图）
 * 
 * FL               FR
 *         0
 *     45  | -45
 *       \   /
 *   90 -     - -90
 *       /   \
 *    135  |  -135
 *       +-180
 * BL               BR
 * 
 * 自动程序编程注意事项，目前自动程序可以进行平移、旋转、自旋校正等单一运动，
 * 单平移、旋转精度较高，但复位精度较低（物理因素），由于底盘复位时。
 * 舵向电机会带动平移电机一起运动，导致底盘移动。
 * 
 * 平移：SetChassisTranslation(double direct,double lenth)
 * 第一个参数是角度deg，如上图。第二个参数是距离mm。
 * 
 * 旋转：Turn_withIncrementalMode(double degrees)
 * 输入一个增量角度值deg，正方向按照上图。
 * 
 * 自旋矫正：CorrectYaw()
 * 校正因误差引起的角度值漂移。
 * 
*/


// 允许误差 单位：units
#define ALLOW_ERROR 60

// 旋转允许误差 单位：units
#define ALLOW_ERROR_FOR_ROTA 80

// 旋转允许误差 单位：deg
#define ALLOW_ERROR_FOR_ROTA_IN_DEG 1

// 舵向定位时间 单位：sec
#define RELAXATION_TIME 0.1

// 旋转因子
#define RELAXATION_TIME_FOR_ROTA 0.008

// 循环时间
#define ONE_LOOP_TIME 0.020

// Wheel movement and chassis orientation Angle coefficient
#define WHEEL_CHASSIS_COEFFICIENT 317.75556

class AutoChassis
{
private:

    frc::DigitalInput BallPipeAuto{2};

    Chassis chassis_inauto;

    AprilTags autoApriltag;

    AutoSub autosubtemp;

    TalonFX shootPathAuto{16};

    //BL ,BR
    //FL ,FR
    //四个全向轮对象
    /**
     * @param _offset WheelModule中的构造函数中的_offset参数，
     *                在CANcoder的self-test中读取Position角度
    */
    WheelModule AutoWheel[4]={WheelModule(4,5,2,209.785,"drive",false,false),WheelModule(6,7,3,344.619,"drive",false,false)
            ,WheelModule(0,1,0,247.676,"drive",true,false),WheelModule(2,3,1,119.268,"drive",false,true)};
    //陀螺仪对象
    WPI_Pigeon2 gyro_auto{33,"drive"};
    double gyro_last_yaw_auto;
    bool isReset_auto = 0;

    // 上次轮向，单位deg
    double wheel_last_yaw[4];
    // 当前轮向，单位deg
    double wheel_pre_yaw[4];

    double wheel_fix_yaw[4];

    // 本次轮行程，单位mm
    int wheel_path[4];

    // 旋转模式轮向参数
    // double w_mode_param[4] = {45.0, -45.0, -45.0, 45.0};
    double w_mode_param[4] = {225.0, -45.0, 135.0, 45.0};
    double w_mode_sign[4] = {1.0, 1.0, 1.0, 1.0};

    // 轮里程计数器
    double path_counter[4];

    //速度
    double lastErr = 0;

    //加速度
    double intAuto = 0;

    double angleTimes = 5;

    double initGyroAngle;

    bool isBallInAuto = 0;

    double pathPos;

    double originalCounter;

    int pathStepCounter = 0;

    int shootStepCounter = 0;

    units::time::second_t timerOffset = units::second_t(0);
    units::time::second_t waitBallTimer = units::second_t(0);



public:
    AutoChassis(){

    }
    ~AutoChassis(){
        
    }
    /**
     * @brief 初始化磁场定向控制，以第三人称视角设置
    */
    void InitMega_auto()
    {
        gyro_auto.Reset();
        gyro_auto.SetYaw(0);
        isReset_auto = 1;
    }
    /**
     * @brief 归零底盘时底盘舵向归中
    */
    void SetChassisZero_auto()
    {
        for (int i = 0; i < 4; i++)
        {
            AutoWheel[i].setTargetAngle_Zero(0);
            wheel_last_yaw[i]=0;

        }
        frc::Wait((units::time::second_t)0.1);
    }
    /**
     * @brief 初始化底盘，舵向归中
    */
    void Chassis_Init_auto()
    {
        // 获取自动初始时间戳
        timerOffset = frc::Timer::GetFPGATimestamp();
        //舵轮底盘归零
        SetChassisZero_auto();

        //初始化磁场定向控制，第三人称
        InitMega_auto();

        SetDrivePID_Slot(0);

        isBallInAuto = 0;

        //初始化陀螺仪
        gyro_last_yaw_auto = gyro_auto.GetYaw();

        initGyroAngle = gyro_auto.GetAngle();

        // 初始化时，把驱动电机的初始偏差写入计数器
        for (int i = 0; i < 4; i++)
        {
            path_counter[i] = AutoWheel[i].GetDrivePosition();
        }
    }

    void setInitAngle(double initAngle){
        initGyroAngle = initAngle;
    }

    
    /**
     * @brief 正常底盘移动函数
     * @param direct 前行方向degree 绝对值
     * @param lenth 前行距离mm
     * @param turnToPitchAngle 射球角度degree
     * @param ifShootBall 是否行进前半段射球 开球道
     * @param ifGetBall 是否开吸球和球道 0不开 1后半段开 2全开
     * @param motorSpeedMode 电机速度槽 0为短距离 1为中 2为长
     * @param ifCorrectYaw 是否使用矫正角度函数
     * @param motionMagicShootSpeed motionMagic跑法射球速度 默认为-1 不射
    */
    void SetChassisTranslationAbs(double direct,double lenth, double turnToPitchAngle, bool ifShootBall, int ifGetBall,
                                    int motorSpeedMode, bool ifCorrectYaw, double ShootSpeed = -1, double secondShootSpeed = -1, bool ifCurve = 0, std::vector<std::vector<double>> path = {{0}}){
        for (int i = 0; i < 4; i++)
        {
            wheel_path[i]=mm2unit(lenth);
            if(AutoWheel[i].GetDriveIsInvert()){
                wheel_path[i] = - wheel_path[i];
            }
        }

        // 设置轮组朝向
        if(frc::Timer::GetFPGATimestamp() - timerOffset < units::second_t(16)){
            SetWheelOrientAbs(direct);
        }
        
        // 行进
        if(motorSpeedMode == 0){
            SetChassisGoStriaghtFast(wheel_path,turnToPitchAngle,ifShootBall, ifGetBall, ifCorrectYaw, 0, ifCurve, path);
        }
        else{
            SetChassisGoStriaghtLong(wheel_path,turnToPitchAngle,ifShootBall, ifGetBall, ShootSpeed, secondShootSpeed, ifCorrectYaw, motorSpeedMode, ifCurve, path);
        }
    }

    /**
     * @brief 单轮判定底盘移动函数 使用autowheel[0] id4 drive判定
     * @param direct 前行方向degree 绝对值
     * @param lenth 前行距离mm
     * @param turnToPitchAngle 射球角度degree
     * @param ifShootBall 是否行进前半段射球 开球道
     * @param ifGetBall 是否开吸球和球道 0不开 1后半段开 2全开
     * @param motorSpeedMode 电机速度槽 0为短距离 1为中 2为长
     * @param ifCorrectYaw 是否使用矫正角度函数
     * @param motionMagicShootSpeed motionMagic跑法射球速度 默认为-1 不射
    */
    void SetChassisTranslationAbsSingle(double direct,double lenth, double turnToPitchAngle, bool ifShootBall, int ifGetBall,
                                    int motorSpeedMode, bool ifCorrectYaw, double ShootSpeed = -1, bool ifCurve = 0, std::vector<std::vector<double>> path = {{0}}){
        for (int i = 0; i < 4; i++)
        {
            wheel_path[i]=mm2unit(lenth);
        }

        // 设置轮组朝向
        if(frc::Timer::GetFPGATimestamp() - timerOffset < units::second_t(16)){
            SetWheelOrientAbs(direct);
        }
        
        // 行进
        if(motorSpeedMode == 0){
            SetChassisGoStriaghtFastSingle(wheel_path,turnToPitchAngle,ifShootBall, ifGetBall, ifCorrectYaw, 0, ifCurve, path);
        }
        else{
            SetChassisGoStriaghtLongSingle(wheel_path,turnToPitchAngle,ifShootBall, ifGetBall, ShootSpeed, ifCorrectYaw, motorSpeedMode, ifCurve, path);
        }
    }

    /**
     * @brief 折线底盘移动函数
     * @param initialDirect 初始前行方向degree 绝对值
     * @param totalLenth 总行程距离mm
     * @param motorSpeedMode 电机速度槽 0为短距离 1为中 2为长
     * @param ifCorrectYaw 是否使用矫正角度函数
     * @param path 轮向折线路线矩阵
     * @param shootAction 上层机构路线矩阵
    */
    void SetChassisTranslationAbsCurve(double initialDirect,double totalLenth, int motorSpeedMode, bool ifCorrectYaw, std::vector<std::vector<double>> path, std::vector<std::vector<double>> shootAction){
        for (int i = 0; i < 4; i++)
        {
            wheel_path[i]=mm2unit(totalLenth);
        }

        // 设置轮组朝向
        if(frc::Timer::GetFPGATimestamp() - timerOffset < units::second_t(16)){
            SetWheelOrientAbs(initialDirect);
        }
        
        // 行进
        if(motorSpeedMode == 0){
            SetChassisGoStriaghtCurve(wheel_path,ifCorrectYaw,path,shootAction);
        }
        else{
            // SetChassisGoStriaghtLong(wheel_path,turnToPitchAngle,ifShootBall, ifGetBall, ShootSpeed, ifCorrectYaw, motorSpeedMode, ifCurve, path);
        }
    }

    /**
     * @brief 单轮判定底盘折线移动函数 使用autowheel[0] id4 drive判定
     * @param initialDirect 初始前行方向degree 绝对值
     * @param totalLenth 总行程距离mm
     * @param motorSpeedMode 电机速度槽 0为短距离 1为中 2为长
     * @param ifCorrectYaw 是否使用矫正角度函数
     * @param path 轮向折线路线矩阵
     * @param shootAction 上层机构路线矩阵
    */

    void SetChassisTranslationAbsCurveSingle(double initialDirect,double totalLenth, int motorSpeedMode, bool ifCorrectYaw, std::vector<std::vector<double>> path, std::vector<std::vector<double>> shootAction){
        for (int i = 0; i < 4; i++)
        {
            wheel_path[i]=mm2unit(totalLenth);
        }

        // 设置轮组朝向
        if(frc::Timer::GetFPGATimestamp() - timerOffset < units::second_t(16)){
            SetWheelOrientAbs(initialDirect);
        }
        
        // 行进
        if(motorSpeedMode == 0){
            SetChassisGoStriaghtCurveSingle(wheel_path,ifCorrectYaw,path,shootAction);
        }
        else{
            // SetChassisGoStriaghtLong(wheel_path,turnToPitchAngle,ifShootBall, ifGetBall, ShootSpeed, ifCorrectYaw, motorSpeedMode, ifCurve, path);
        }
    }

    /**
     * @brief 设置轮子朝向 绝对值 基于初始的角度 初始化时射球为0° 逆时针-角度 顺时针+角度
    */

    void SetWheelOrientAbs(double degree){
        double degreeRota = gyro_auto.GetAngle() - initGyroAngle + degree;
        for (int i = 0; i < 4; i++)
        {
            wheel_pre_yaw[i] = degreeRota;
        }
        for (int i = 0; i < 4; i++)
        {
            while(abs(wheel_pre_yaw[i] - wheel_last_yaw[i]) > 90)
            {
                if (wheel_pre_yaw[i] < wheel_last_yaw[i]) {
                    wheel_pre_yaw[i] += 180;
                }
                else {
                    wheel_pre_yaw[i] -= 180;
                }
                // 轮速反向
                wheel_path[i] = - wheel_path[i];
            }

            
            // 轮向角更新
            wheel_last_yaw[i] = wheel_pre_yaw[i];
            gyro_last_yaw_auto = gyro_auto.GetAngle();

            // 轮向角动作
            AutoWheel[i].setTargetAngle_withBicontrolAlgorithm(wheel_pre_yaw[i]);

        }
        frc::Wait((units::time::second_t)0.08);
    }

    void SetWheelOrientCorrecting(){
        for (int i = 0; i < 4; i++)
        {
            wheel_fix_yaw[i] = gyro_auto.GetAngle() - gyro_last_yaw_auto + wheel_last_yaw[i];
            AutoWheel[i].setTargetAngle_withBicontrolAlgorithm(wheel_fix_yaw[i]);
        }
    }

    void SetWheelOrientMoving(double degree){
        double degreeRota = gyro_auto.GetAngle() - initGyroAngle + degree;
        for (int i = 0; i < 4; i++)
        {
            wheel_pre_yaw[i] = degreeRota;
        }
        for (int i = 0; i < 4; i++)
        {
            while(abs(wheel_pre_yaw[i] - wheel_last_yaw[i]) > 90)
            {
                if (wheel_pre_yaw[i] < wheel_last_yaw[i]) {
                    wheel_pre_yaw[i] += 180;
                }
                else {
                    wheel_pre_yaw[i] -= 180;
                }
                // 轮速反向
                // wheel_path[i] = - wheel_path[i];
            }
            
            // 轮向角更新
            wheel_last_yaw[i] = wheel_pre_yaw[i];
            gyro_last_yaw_auto = gyro_auto.GetAngle();

            // 轮向角动作
            AutoWheel[i].setTargetAngle_withBicontrolAlgorithm(wheel_pre_yaw[i]);
        }
    }

    void updateLastYaw(){
        for (int i = 0; i < 4; i++)
        {
            wheel_last_yaw[i] = wheel_fix_yaw[i];
        }
    }

    /**
     * @brief 设置轮向为旋转模式
    */
    void SetChassisRotateMode(){
        for (int i = 0; i < 4; i++)
        {
            while(abs(w_mode_param[i] - wheel_last_yaw[i]) > 90)
            {
                if (w_mode_param[i] < wheel_last_yaw[i]) {
                    w_mode_param[i] += 180;
                }
                else {
                    w_mode_param[i] -= 180;
                }
                // 轮速反向
                w_mode_sign[i] = - w_mode_sign[i];
            }
            // 轮向角动作
            AutoWheel[i].setTargetAngle_withBicontrolAlgorithm(w_mode_param[i]);

            // 轮向角更新
            wheel_last_yaw[i] = w_mode_param[i];
        }
        frc::Wait((units::time::second_t)0.08);
        
    }

    void SetChassisGoStriaghtFast(int setIncremental[], double turnToPitchAngle, bool ifShootBall, int ifGetBall, bool ifCorrectAngle, int pidSlot, bool ifCurve, std::vector<std::vector<double>> path){
        // 更新目标距离

        if(ifCurve){
            originalCounter = path_counter[0];
            pathStepCounter = 0;
        }

        for (int i = 0; i < 4; i++)
        {
            path_counter[i] += setIncremental[i];
        }
        
        while (frc::Timer::GetFPGATimestamp() - timerOffset < units::second_t(16))
        {
            for (int i = 0; i < 4; i++)
            {
                AutoWheel[i].SetDriveMotorPositionSlot(path_counter[i],pidSlot);
            }

            if(ifGetBall==1){
                if(fabs(AutoWheel[0].GetDrivePosition()-path_counter[0])<(fabs(setIncremental[0])/2)){
                    getBallOnly();
                }
            }
            if(ifGetBall==2){
                getBallOnly();
            }

            if(fabs(AutoWheel[0].GetDrivePosition()-path_counter[0])<(fabs(setIncremental[0])/2)){
                if(turnToPitchAngle>0){
                    autosubtemp.TurnPitchAngle(turnToPitchAngle);
                }
            }
            else{
                if(ifShootBall){
                    autosubtemp.Open_Path(0.4);
                }
            }

            if(ifCorrectAngle){
                SetWheelOrientCorrecting();
            }

            if(ifCurve){
                if(pathStepCounter < path.size() && fabs(AutoWheel[0].GetDrivePosition() - originalCounter) > mm2unit(path[pathStepCounter][0])){
                    updateLastYaw();
                    SetWheelOrientMoving(path[pathStepCounter][1]);
                    pathStepCounter++;
                }
            }

            // 用path差值跳出循环
            if
            // ((fabs(AutoWheel[0].GetDrivePosition()-path_counter[0]) < 5000 && 
            // fabs(AutoWheel[1].GetDrivePosition()-path_counter[1]) < 5000 &&
            // fabs(AutoWheel[2].GetDrivePosition()-path_counter[2]) < 5000 &&
            // fabs(AutoWheel[3].GetDrivePosition()-path_counter[3]) < 5000))
            (((setIncremental[0]>0 && AutoWheel[0].GetDrivePosition()-path_counter[0] > -5000)||(setIncremental[0]<0 && AutoWheel[0].GetDrivePosition()-path_counter[0] < 5000))&&
            ((setIncremental[1]>0 && AutoWheel[1].GetDrivePosition()-path_counter[1] > -5000)||(setIncremental[1]<0 && AutoWheel[1].GetDrivePosition()-path_counter[1] < 5000))&&
            ((setIncremental[2]>0 && AutoWheel[2].GetDrivePosition()-path_counter[2] > -5000)||(setIncremental[2]<0 && AutoWheel[2].GetDrivePosition()-path_counter[2] < 5000))&&
            ((setIncremental[3]>0 && AutoWheel[3].GetDrivePosition()-path_counter[3] > -5000)||(setIncremental[3]<0 && AutoWheel[3].GetDrivePosition()-path_counter[3] < 5000)))
            {
                // 稍等片刻
                frc::Wait((units::time::second_t)0.02);
                if(ifCorrectAngle){
                    updateLastYaw();
                }
                break;
            }
        }
    }

    void SetChassisGoStriaghtFastSingle(int setIncremental[], double turnToPitchAngle, bool ifShootBall, int ifGetBall, bool ifCorrectAngle, int pidSlot, bool ifCurve, std::vector<std::vector<double>> path){
        // 更新目标距离

        if(ifCurve){
            originalCounter = path_counter[0];
            pathStepCounter = 0;
        }

        for (int i = 0; i < 4; i++)
        {
            path_counter[i] += setIncremental[i];
        }
        
        while (frc::Timer::GetFPGATimestamp() - timerOffset < units::second_t(16))
        {
            AutoWheel[0].SetDriveMotorPositionSlot(path_counter[0],pidSlot);
            
            for (int i = 1; i < 4; i++)
            {
                AutoWheel[i].SetDriveMotorFollow(4,setIncremental[0]==setIncremental[i]);
            }

            if(ifGetBall==1){
                if(fabs(AutoWheel[0].GetDrivePosition()-path_counter[0])<(fabs(setIncremental[0])/2)){
                    getBallOnly();
                }
            }
            if(ifGetBall==2){
                getBallOnly();
            }

            if(fabs(AutoWheel[0].GetDrivePosition()-path_counter[0])<(fabs(setIncremental[0])/2)){
                if(turnToPitchAngle>0){
                    autosubtemp.TurnPitchAngle(turnToPitchAngle);
                }
            }
            else{
                if(ifShootBall){
                    autosubtemp.Open_Path(0.4);
                }
            }

            if(ifCorrectAngle){
                SetWheelOrientCorrecting();
            }

            if(ifCurve){
                if(pathStepCounter < path.size() && fabs(AutoWheel[0].GetDrivePosition() - originalCounter) > mm2unit(path[pathStepCounter][0])){
                    updateLastYaw();
                    SetWheelOrientMoving(path[pathStepCounter][1]);
                    pathStepCounter++;
                }
            }

            // 用path差值跳出循环
            frc::SmartDashboard::PutNumber("setInc",setIncremental[0]);
            frc::SmartDashboard::PutNumber("drivePos",AutoWheel[0].GetDrivePosition());
            frc::SmartDashboard::PutNumber("pathCount",path_counter[0]);
            if((setIncremental[0]>0 && AutoWheel[0].GetDrivePosition()-path_counter[0] > -5000)||(setIncremental[0]<0 && AutoWheel[0].GetDrivePosition()-path_counter[0] < 5000)){
                // 稍等片刻
                frc::Wait((units::time::second_t)0.02);
                if(ifCorrectAngle){
                    updateLastYaw();
                }
                break;
            }
        }
    }

    void SetChassisGoStriaghtCurve(int setIncremental[], bool ifCorrectAngle, std::vector<std::vector<double>> path, std::vector<std::vector<double>> shootAction){
        
        // 更新目标距离
        originalCounter = path_counter[0];
        pathStepCounter = 0;
        shootStepCounter = 0;

        for (int i = 0; i < 4; i++)
        {
            path_counter[i] += setIncremental[i];
        }
        
        while (frc::Timer::GetFPGATimestamp() - timerOffset < units::second_t(16))
        {
            for (int i = 0; i < 4; i++)
            {
                AutoWheel[i].SetDriveMotorPositionSlot(path_counter[i],0);
            }
            
            if((shootStepCounter+1) < shootAction.size() && fabs(AutoWheel[0].GetDrivePosition() - originalCounter) > mm2unit(shootAction[shootStepCounter+1][0])){
                shootStepCounter++;
            }
            
            autosubtemp.TurnPitchAngle(shootAction[shootStepCounter][1]);

            if(shootAction[shootStepCounter][2]>0){
                getBallOnly();
            }
            else{
                setBallInOutAuto(0);
                autosubtemp.Open_Path(0.4);
            }

            if(ifCorrectAngle){
                SetWheelOrientCorrecting();
            }

            if(pathStepCounter < path.size() && fabs(AutoWheel[0].GetDrivePosition() - originalCounter) > mm2unit(path[pathStepCounter][0])){
                updateLastYaw();
                SetWheelOrientMoving(path[pathStepCounter][1]);
                pathStepCounter++;
            }

            // 用path差值跳出循环

            frc::SmartDashboard::PutNumber("lengthPos",fabs(AutoWheel[0].GetDrivePosition()-path_counter[0]));
            if
            // ((fabs(AutoWheel[0].GetDrivePosition()-path_counter[0]) < 5000 && 
            // fabs(AutoWheel[1].GetDrivePosition()-path_counter[1]) < 5000 &&
            // fabs(AutoWheel[2].GetDrivePosition()-path_counter[2]) < 5000 &&
            // fabs(AutoWheel[3].GetDrivePosition()-path_counter[3]) < 5000))
            (((setIncremental[0]>0 && AutoWheel[0].GetDrivePosition()-path_counter[0] > -5000)||(setIncremental[0]<0 && AutoWheel[0].GetDrivePosition()-path_counter[0] < 5000))&&
            ((setIncremental[1]>0 && AutoWheel[1].GetDrivePosition()-path_counter[1] > -5000)||(setIncremental[1]<0 && AutoWheel[1].GetDrivePosition()-path_counter[1] < 5000))&&
            ((setIncremental[2]>0 && AutoWheel[2].GetDrivePosition()-path_counter[2] > -5000)||(setIncremental[2]<0 && AutoWheel[2].GetDrivePosition()-path_counter[2] < 5000))&&
            ((setIncremental[3]>0 && AutoWheel[3].GetDrivePosition()-path_counter[3] > -5000)||(setIncremental[3]<0 && AutoWheel[3].GetDrivePosition()-path_counter[3] < 5000)))
            
            {
                // 稍等片刻
                frc::Wait((units::time::second_t)0.02);
                if(ifCorrectAngle){
                    updateLastYaw();
                }
                break;
            }
        }
    }

    void SetChassisGoStriaghtCurveSingle(int setIncremental[], bool ifCorrectAngle, std::vector<std::vector<double>> path, std::vector<std::vector<double>> shootAction){
        
        // 更新目标距离
        originalCounter = path_counter[0];
        pathStepCounter = 0;
        shootStepCounter = 0;

        for (int i = 0; i < 4; i++)
        {
            path_counter[i] += setIncremental[i];
        }
        
        while (frc::Timer::GetFPGATimestamp() - timerOffset < units::second_t(16))
        {
            AutoWheel[0].SetDriveMotorPositionSlot(path_counter[0],0);
            
            for (int i = 1; i < 4; i++)
            {
                AutoWheel[i].SetDriveMotorFollow(4,setIncremental[0]==setIncremental[i]);
            }
            
            if((shootStepCounter+1) < shootAction.size() && fabs(AutoWheel[0].GetDrivePosition() - originalCounter) > mm2unit(shootAction[shootStepCounter+1][0])){
                shootStepCounter++;
            }
            
            autosubtemp.TurnPitchAngle(shootAction[shootStepCounter][1]);

            if(shootAction[shootStepCounter][2]>0){
                getBallOnly();
            }
            else{
                setBallInOutAuto(0);
                autosubtemp.Open_Path(0.4);
            }

            if(ifCorrectAngle){
                SetWheelOrientCorrecting();
            }

            if(pathStepCounter < path.size() && fabs(AutoWheel[0].GetDrivePosition() - originalCounter) > mm2unit(path[pathStepCounter][0])){
                updateLastYaw();
                SetWheelOrientMoving(path[pathStepCounter][1]);
                pathStepCounter++;
            }

            // 用path差值跳出循环

            frc::SmartDashboard::PutNumber("lengthPos",fabs(AutoWheel[0].GetDrivePosition()-path_counter[0]));

            if((setIncremental[0]>0 && AutoWheel[0].GetDrivePosition()-path_counter[0] > -5000)||(setIncremental[0]<0 && AutoWheel[0].GetDrivePosition()-path_counter[0] < 5000))
            {
                // 稍等片刻
                frc::Wait((units::time::second_t)0.02);
                if(ifCorrectAngle){
                    updateLastYaw();
                }
                break;
            }
        }
    }

    void SetChassisGoStriaghtLong(int setIncremental[], double turnToPitchAngle, bool ifShootBall, int ifGetBall, double shootSpeed, double secondShootSpeed, bool ifCorrectAngle, int pidSlot, bool ifCurve, std::vector<std::vector<double>> path){
        // 更新目标距离

        if(ifCurve){
            originalCounter = path_counter[0];
            pathStepCounter = 0;
        }

        for (int i = 0; i < 4; i++)
        {
            path_counter[i] += setIncremental[i];
        }
        
        while (frc::Timer::GetFPGATimestamp() - timerOffset < units::second_t(16))
        {
            for (int i = 0; i < 4; i++)
            {
                AutoWheel[i].SetDriveMotorPositionSlot(path_counter[i],pidSlot);
            }

            if(ifGetBall==1){
                if(fabs(AutoWheel[0].GetDrivePosition()-path_counter[0])<(fabs(setIncremental[0])/2)){
                    getBallOnlyNew();
                }
            }
            if(ifGetBall==2){
                getBallOnlyNew();
            }

            if(shootSpeed>0){
                if((secondShootSpeed<0)||((secondShootSpeed>0)&&(fabs(AutoWheel[0].GetDrivePosition()-path_counter[0])<(fabs(setIncremental[0])*1/4)))){
                    autosubtemp.shootball_auto(shootSpeed);
                }
                else if((secondShootSpeed>0)&&(fabs(AutoWheel[0].GetDrivePosition()-path_counter[0])>(fabs(setIncremental[0])*1/4))){
                    autosubtemp.shootball_auto(secondShootSpeed);
                }
                if((ifShootBall)&&(fabs(AutoWheel[0].GetDrivePosition()-path_counter[0])>(fabs(setIncremental[0])*3/4))){
                    autosubtemp.Open_Path(0.4);
                    frc::SmartDashboard::PutNumber("shootMagic",fabs(AutoWheel[0].GetDrivePosition()-path_counter[0]));
                }
            }

            if(turnToPitchAngle>0&&fabs(AutoWheel[0].GetDrivePosition()-path_counter[0])<(fabs(setIncremental[0])/2)){
                autosubtemp.TurnPitchAngle(turnToPitchAngle);
            }

            if(ifCorrectAngle){
                SetWheelOrientCorrecting();
            }

            if(ifCurve){
                if(pathStepCounter < path.size() && fabs(AutoWheel[0].GetDrivePosition() - originalCounter) > mm2unit(path[pathStepCounter][0])){
                    updateLastYaw();
                    SetWheelOrientMoving(path[pathStepCounter][1]);
                    pathStepCounter++;
                }
            }

            // 用path差值跳出循环

            if
            // ((fabs(AutoWheel[0].GetDrivePosition()-path_counter[0]) < 5000 && 
            // fabs(AutoWheel[1].GetDrivePosition()-path_counter[1]) < 5000 &&
            // fabs(AutoWheel[2].GetDrivePosition()-path_counter[2]) < 5000 &&
            // fabs(AutoWheel[3].GetDrivePosition()-path_counter[3]) < 5000))
            (((setIncremental[0]>0 && AutoWheel[0].GetDrivePosition()-path_counter[0] > -5000)||(setIncremental[0]<0 && AutoWheel[0].GetDrivePosition()-path_counter[0] < 5000))&&
            ((setIncremental[1]>0 && AutoWheel[1].GetDrivePosition()-path_counter[1] > -5000)||(setIncremental[1]<0 && AutoWheel[1].GetDrivePosition()-path_counter[1] < 5000))&&
            ((setIncremental[2]>0 && AutoWheel[2].GetDrivePosition()-path_counter[2] > -5000)||(setIncremental[2]<0 && AutoWheel[2].GetDrivePosition()-path_counter[2] < 5000))&&
            ((setIncremental[3]>0 && AutoWheel[3].GetDrivePosition()-path_counter[3] > -5000)||(setIncremental[3]<0 && AutoWheel[3].GetDrivePosition()-path_counter[3] < 5000)))
            {
                // 稍等片刻
                frc::Wait((units::time::second_t)0.02);
                if(ifCorrectAngle){
                    updateLastYaw();
                }
                break;
            }
        }
    }

    void SetChassisGoStriaghtLongSingle(int setIncremental[], double turnToPitchAngle, bool ifShootBall, int ifGetBall, double shootSpeed, bool ifCorrectAngle, int pidSlot, bool ifCurve, std::vector<std::vector<double>> path){
        // 更新目标距离

        if(ifCurve){
            originalCounter = path_counter[0];
            pathStepCounter = 0;
        }

        for (int i = 0; i < 4; i++)
        {
            path_counter[i] += setIncremental[i];
        }
        
        while (frc::Timer::GetFPGATimestamp() - timerOffset < units::second_t(16))
        {
            AutoWheel[0].SetDriveMotorPositionSlot(path_counter[0],pidSlot);
            
            for (int i = 1; i < 4; i++)
            {
                AutoWheel[i].SetDriveMotorFollow(4,setIncremental[0]==setIncremental[i]);
            }

            if(ifGetBall==1){
                if(fabs(AutoWheel[0].GetDrivePosition()-path_counter[0])<(fabs(setIncremental[0])/2)){
                    getBallOnlyNew();
                }
            }
            if(ifGetBall==2){
                getBallOnlyNew();
            }

            if(shootSpeed>0){
                if(!ifShootBall){
                    autosubtemp.shootball_auto(shootSpeed);
                }
                else if(fabs(AutoWheel[0].GetDrivePosition()-path_counter[0])>(fabs(setIncremental[0])*3/4)){
                    autosubtemp.Open_Path(0.4);
                    autosubtemp.shootball_auto(shootSpeed);
                    frc::SmartDashboard::PutNumber("shootMagic",fabs(AutoWheel[0].GetDrivePosition()-path_counter[0]));
                }
                else{
                    autosubtemp.shootPercent(-0.05);
                }
            }

            if(turnToPitchAngle>0&&fabs(AutoWheel[0].GetDrivePosition()-path_counter[0])<(fabs(setIncremental[0])/2)){
                autosubtemp.TurnPitchAngle(turnToPitchAngle);
            }

            if(ifCorrectAngle){
                SetWheelOrientCorrecting();
            }

            if(ifCurve){
                if(pathStepCounter < path.size() && fabs(AutoWheel[0].GetDrivePosition() - originalCounter) > mm2unit(path[pathStepCounter][0])){
                    updateLastYaw();
                    SetWheelOrientMoving(path[pathStepCounter][1]);
                    pathStepCounter++;
                }
            }

            // 用path差值跳出循环
            if
            // (fabs(AutoWheel[0].GetDrivePosition()-path_counter[0]) < 5000)
            ((setIncremental[0]>0 && AutoWheel[0].GetDrivePosition()-path_counter[0] > -5000)||(setIncremental[0]<0 && AutoWheel[0].GetDrivePosition()-path_counter[0] < 5000))
            {
                // 稍等片刻
                frc::Wait((units::time::second_t)0.02);
                if(ifCorrectAngle){
                    updateLastYaw();
                }
                break;
            }
        }
    }


    /**
     * @brief 单位转换，将驱动电机编码器值，转换为实际行走值，纯滚动假设
     * @param _unit 驱动电机传感器单位增量值
     * @details 前减速比6.74603
     * @param WDLC Wheel diameter loss coefficient，轮径损耗系数，趋向于1的数，默认为1
     * @return 实际行进毫米数
    */
    double unit2mm(double _unit, double WDLC = 1){
        return _unit / MOTOR_ENCODER_RESOLUTION / DRIVE_GEAR_RATIO * PI * WHEEL_DIAMETER * WDLC;
    }

    /**
     * @brief 单位转换，将驱动电机编码器值，转换为实际行走值，纯滚动假设
     * @param _mm 实际行进毫米数
     * @details 前减速比6.74603
     * @param WDLC Wheel diameter loss coefficient，轮径损耗系数，趋向于1的数，默认为1
     * @return 驱动电机传感器单位增量值
    */
    double mm2unit(double _mm, double WDLC = 1){
        return _mm * MOTOR_ENCODER_RESOLUTION * DRIVE_GEAR_RATIO / PI / WHEEL_DIAMETER / WDLC;
    }

    void PutWheelInfo(){
        frc::SmartDashboard::PutNumber("BL Ang",AutoWheel[0].GetCanCoderAngle());
        frc::SmartDashboard::PutNumber("BR Ang",AutoWheel[1].GetCanCoderAngle());
        frc::SmartDashboard::PutNumber("FL Ang",AutoWheel[2].GetCanCoderAngle());
        frc::SmartDashboard::PutNumber("FR Ang",AutoWheel[3].GetCanCoderAngle());

        frc::SmartDashboard::PutNumber("BL Vel",AutoWheel[0].GetDriveConderVolecity());
        frc::SmartDashboard::PutNumber("BR Vel",AutoWheel[1].GetDriveConderVolecity());
        frc::SmartDashboard::PutNumber("FL Vel",AutoWheel[2].GetDriveConderVolecity());
        frc::SmartDashboard::PutNumber("FR Vel",AutoWheel[3].GetDriveConderVolecity());
        
        frc::SmartDashboard::PutNumber("Velocity",(fabs(AutoWheel[0].GetDriveConderVolecity())+
                                                    fabs(AutoWheel[1].GetDriveConderVolecity())+
                                                    fabs(AutoWheel[2].GetDriveConderVolecity())+
                                                    fabs(AutoWheel[3].GetDriveConderVolecity()))/4.0);
    }

    void Turn_withAbsoluteModeNew(double degrees, bool ifGetBall,bool ifShootBall, double angleErr = 2.5){
        double tar = initGyroAngle + degrees;
        double err = tar - gyro_auto.GetAngle();
        angleTimes = 5;

        // 开始执行，切换至旋转模式
        if(frc::Timer::GetFPGATimestamp() - timerOffset < units::second_t(16)){
            SetChassisRotateMode();
        }

        while(angleTimes > 0 && (frc::Timer::GetFPGATimestamp() - timerOffset < units::second_t(16)))
        {
            // 计算误差值
            err = tar - gyro_auto.GetAngle();

            double errDiff = gyro_auto.GetRate();

            if(fabs(err)<3){
                intAuto += err;
            }
            else{
                intAuto = 0;
            }

            double pid_val = Cap(-0.019 * err -0.0035*intAuto + 0.0015*errDiff,0.55);
            // 执行控制器方案
            for (int i = 0; i < 4; i++)
            {
                AutoWheel[i].WheelPersentCtrl(pid_val * w_mode_sign[i]);
            }

            if(ifGetBall){
                getBallOnly();
            }

            if(ifShootBall&&angleTimes<2){
                autosubtemp.Open_Path(0.5);
            }

            // 等待一次循环时间
            frc::Wait((units::time::second_t)ONE_LOOP_TIME);
            lastErr = err;
            if (fabs(err) < angleErr && fabs(lastErr-err) < 1){
                angleTimes = angleTimes - 1;
            }
            else{
                angleTimes = 5;
            }
        }

        // 获取旋转后各个电机编码器值，并更新到计数器
        for (int i = 0; i < 4; i++)
        {
            path_counter[i] = AutoWheel[i].GetDrivePosition();
            AutoWheel[i].WheelPersentCtrl(0);
        }
    }


    /*
    *牛逼算法算出来的速度转向函数
    *靠惯性转的 提前停止转向
    *测试大角度差不多可以 能用 误差非常大 小角度基本不会动 但是快
    */

    void Turn_withAbsFast(double degrees, bool ifGetBall){
        double tar = initGyroAngle + degrees;
        double err = tar - gyro_auto.GetAngle();
        // angleTimes = 4;

        // 开始执行，切换至旋转模式
        SetChassisRotateMode();

        double stopErr = err/3;

        // fabs(err) > ALLOW_ERROR_FOR_ROTA_IN_DEG || fabs(lastErr-err) > 1

        while(fabs(err) > fabs(stopErr))
        {
            // 计算误差值
            err = tar - gyro_auto.GetAngle();

            double pid_val = Cap(-0.01 * err,0.5);
            // 执行控制器方案
            for (int i = 0; i < 4; i++)
            {
                AutoWheel[i].WheelPersentCtrl(Cap(pid_val * w_mode_sign[i],0.4));
            }

            if(ifGetBall){
                getBallOnly();
            }

            // 等待一次循环时间
            frc::Wait((units::time::second_t)ONE_LOOP_TIME);

            // lastErr = err;
        }

        // 获取旋转后各个电机编码器值，并更新到计数器
        for (int i = 0; i < 4; i++)
        {
            path_counter[i] = AutoWheel[i].GetDrivePosition();
            AutoWheel[i].WheelPersentCtrl(0);
            AutoWheel[i].Drive_Brake();
        }
    }


    void getBallOnly()
    {
        if(!isBallInAuto)
        {
            autosubtemp.Open_Intake();
            autosubtemp.Open_Path(0.2);
        }
        // 到达传感器
        if(!BallPipeAuto.Get()){
            // 记录第一次到达数值
            if (!isBallInAuto)
            {
                isBallInAuto = 1;
                pathPos = shootPathAuto.GetSelectedSensorPosition()+400;
            }
        }
        if(isBallInAuto){
            autosubtemp.Close_Intake();
            shootPathAuto.Set(ControlMode::Position,pathPos);
        }
    }

    void getBallOnlyNew()
    {
        if(!isBallInAuto)
        {
            autosubtemp.Open_Intake();
            autosubtemp.Open_Path(0.2);
        }
        // 到达传感器
        if(!BallPipeAuto.Get()){
            // 记录第一次到达数值
            if (!isBallInAuto)
            {
                isBallInAuto = 1;
                pathPos = shootPathAuto.GetSelectedSensorPosition()+500;
            }
        }
        if(isBallInAuto){
            autosubtemp.Close_Intake();
            shootPathAuto.Set(ControlMode::Position,pathPos);
        }
    }

    void setBallInOutAuto(bool ballPos){
        isBallInAuto = ballPos;
    }

    void setInitGyroApril(){
        autoApriltag.GetVisionInfo();
        double aprilTagYaw=0;
        double gyroNow = 0;
        double minYaw = autoApriltag.GetYaw_Vision();
        double maxYaw = autoApriltag.GetYaw_Vision();
        double minGyro = gyro_auto.GetAngle();
        double maxGyro = gyro_auto.GetAngle();
        for(int i=0;i<12;i++){
            autoApriltag.GetVisionInfo();
            if(autoApriltag.GetYaw_Vision()< minYaw){
                minYaw = autoApriltag.GetYaw_Vision();
            }
            if(autoApriltag.GetYaw_Vision()> maxYaw){
                maxYaw = autoApriltag.GetYaw_Vision();
            }
            if(gyro_auto.GetAngle()< minGyro){
                minGyro = gyro_auto.GetAngle();
            }
            if(gyro_auto.GetAngle()> maxGyro){
                maxGyro = gyro_auto.GetAngle();
            }
            aprilTagYaw += autoApriltag.GetYaw_Vision();
            gyroNow += gyro_auto.GetAngle();
            frc::Wait((units::time::second_t)0.01);
        }
        aprilTagYaw = aprilTagYaw - minYaw - maxYaw;
        aprilTagYaw = aprilTagYaw/10;
        gyroNow = gyroNow - maxGyro - minGyro;
        gyroNow = gyroNow/10;

        frc::SmartDashboard::PutNumber("aprilYaw",aprilTagYaw);
        frc::SmartDashboard::PutNumber("initGyroAngle",initGyroAngle);
        frc::SmartDashboard::PutNumber("nowAngle",gyroNow);
        initGyroAngle = initGyroAngle - (aprilTagYaw - (gyroNow-initGyroAngle));
    }

    void waitBallShoot(){
        while(BallPipeAuto.Get()){}
    }

    void SetDrivePID_Slot(int speedMode){
        for (int i = 0; i < 4; i++)
        {
            AutoWheel[i].ChangeDrivePositionPIDSlot(speedMode);
        }
    }

    bool GetIsBallIn(){
        return isBallInAuto;
    }

    void waitBallIn(double timeSec){
        waitBallTimer = frc::Timer::GetFPGATimestamp();
        while (frc::Timer::GetFPGATimestamp() - waitBallTimer < units::second_t(timeSec)){
            getBallOnly();
            if(isBallInAuto){
                break;
            }
        }
    }

    double getFinalAngle(){
        return gyro_auto.GetAngle();
    }
    // void StopDrive(){
    //     for(int i=0;i<4;i++){
    //         path_counter[i]=AutoWheel[i].GetDrivePosition();
    //         AutoWheel[i].SetDriveStop(path_counter[i]);
    //     }
    // }

    // void StopDriveLast(){
    //     for(int i=0;i<4;i++){
    //         path_counter[i]=AutoWheel[i].GetDrivePosition();
    //         AutoWheel[i].SetDriveMotorPosition(path_counter[i]);
    //     }
    // }

};