#pragma once
#include "../../lib/drivers/LazyTalonFX.h"
#include "ctre/Phoenix.h"
#include <frc/DigitalInput.h>


class SubSys
{
private:
    // 子系统执行器电机
    LazyTalonFX *submotor;
    // 执行器电机跟随电机，或比例差动电机。
    LazyTalonFX *submotor_follower;
    
    CANCoder *subcoder;
    double mCoder_offset = 0;

    frc::DigitalInput *subIO;

    short mState = 0;
public:
    /**
     * @brief 无反馈子系统构造函数
     * @param effectorID 执行器电机ID
    */
    SubSys(int effectorID){
        submotor = new LazyTalonFX(effectorID);
    }
    /**
     * @brief 带反馈子系统构造函数
     * @param effectorID 执行器电机ID
     * @param IO_ID NI主控板上的IO_ID,（必须输入值） 
     * 使用该端口时，输入非零ID，0号端口可应用于非子系统
     * 不使用端口时，输入0。
     * @param coderID 非零ID，0与底盘CanCoder冲突。
    */
    SubSys(int effectorID,int IO_ID,int coderID = 0,double __offset = 0){

        submotor = new LazyTalonFX(effectorID);

        // 输入为零时不使用该端口，无效
        if(IO_ID != 0){
            subIO = new frc::DigitalInput(IO_ID);
        }

        // 默认为0，因为底盘存在为0的CanCoder，所以默认情况下为不使能
        if(coderID != 0){
            subcoder = new CANCoder(coderID);
            mCoder_offset = __offset;
        }

    }
    ~SubSys(){
    }

    /**
     * @brief 获取子系统状态
     * @return 子系统状态值（提供-32768～32767取值，外部约定数字含义）
    */
    short GetSubSysState(){
        return mState;
    }
    double GetCoderABSPosition(){
        return subcoder->GetAbsolutePosition() + mCoder_offset;
    }
    double GetCoderVelocity(){
        return subcoder->GetVelocity();
    }

};
