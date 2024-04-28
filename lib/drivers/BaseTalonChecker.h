#pragma once
#include <vector>
#include "ctre/Phoenix.h"
#include "MotorChecker.h"


// #include "Subsystem.h"

class BaseTalonChecker : public MotorChecker<BaseTalon> {
private:
    struct StoredBaseTalonConfiguration {
        ControlMode mMode;
        double mSetValue;
    };
    std::vector<StoredBaseTalonConfiguration> mStoredConfigurations;
    void storeConfiguration(){
        // 记录所有 talon 的先前配置
        for (MotorConfig<BaseTalon>& config : mMotorsToCheck) {
            StoredBaseTalonConfiguration configuration;
            configuration.mMode = ControlMode::PercentOutput; //config.mMotor->GetControlMode();
            configuration.mSetValue = 0.0; //config.mMotor->GetLastSet();

            mStoredConfigurations.push_back(configuration);
        }
    }
    void restoreConfiguration(){
        for (size_t i = 0; i < mMotorsToCheck.size(); ++i) {
            mMotorsToCheck[i].mMotor->Set(mStoredConfigurations[i].mMode, mStoredConfigurations[i].mSetValue);
        }
    }
    void setMotorOutput(BaseTalon* motor, double output){
        motor->Set(ControlMode::PercentOutput, output);
    }
    double getMotorCurrent(BaseTalon* motor){
        return motor->GetStatorCurrent();
    }
    bool checkMotors(Subsystem* subsystem, std::vector<MotorConfig<BaseTalon>> motorsToCheck, CheckerConfig checkerConfig){
        BaseTalonChecker checker;
        return checker.checkMotorsImpl(subsystem, motorsToCheck, checkerConfig);
    }

};