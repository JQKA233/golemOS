#pragma once
# include "ctre/Phoenix.h"
#include <string>


/**
 * 这个类是CANSparkMax的一个简单包装，它通过跳过重复的set命令来减少CAN总线/CPU的开销。
 */
class LazyTalonFX
{
private:
    double mLastSet = NAN;
    ControlMode mLastControlMode = ControlMode::Disabled;
public:
    TalonFX *superTalonFX;

    /**
     * 这个类是CANSparkMax的一个简单包装，它通过跳过重复的set命令来减少CAN总线/CPU的开销。
     */

    LazyTalonFX(int deviceNumber)
    {
        superTalonFX = new TalonFX(deviceNumber);
    }
    LazyTalonFX(int deviceNumber, std::string canbus)
    {
        superTalonFX = new TalonFX(deviceNumber, canbus);
    }

    ~LazyTalonFX()
    {
    }
    double getLastSet()
    {
        return mLastSet;
    }

    double getStatorCurrent()
    {
        return superTalonFX->GetStatorCurrent();
    }
    ControlMode getControlMode()
    {
        return mLastControlMode;
    }
    void set(ControlMode mode, double value) {
        if (value != mLastSet || mode != mLastControlMode) {
            mLastSet = value;
            mLastControlMode = mode;
            superTalonFX->Set(mode, value);
        }
    }

    void ChangePID_Slot(int _slotID){
        superTalonFX->SelectProfileSlot(_slotID,0);
    }
    
};
