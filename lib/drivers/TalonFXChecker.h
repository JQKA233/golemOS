#pragma once
#include <vector>
#include "ctre/Phoenix.h"
#include "MotorChecker.h"
#include "LazyTalonFX.h"


class TalonFXChecker : public MotorChecker<LazyTalonFX> {
private:
    struct StoredTalonFXConfiguration {
        ctre::phoenix::motorcontrol::ControlMode mMode;
        double mSetValue;
    };
    std::vector<StoredTalonFXConfiguration> mStoredConfigurations;

public:
    static bool checkMotors(Subsystem* subsystem, 
                            std::vector<MotorConfig<LazyTalonFX>> motorsToCheck, 
                            CheckerConfig checkerConfig) {
        TalonFXChecker checker;
        return checker.checkMotorsImpl(subsystem, motorsToCheck, checkerConfig);
    }

    // Implementations of abstract methods
    void storeConfiguration() override 
    {
        // Record previous configuration for all TalonFX
        for (MotorConfig<LazyTalonFX> config : mMotorsToCheck) {
            LazyTalonFX* talon = config.mMotor;

            StoredTalonFXConfiguration configuration;
            configuration.mMode = talon->getControlMode();
            configuration.mSetValue = talon->getLastSet();

            mStoredConfigurations.push_back(configuration);
        }
    }

    void restoreConfiguration() override 
    {
        for (size_t i = 0; i < mMotorsToCheck.size(); ++i) {
            mMotorsToCheck[i].mMotor->set(mStoredConfigurations[i].mMode, 
                                          mStoredConfigurations[i].mSetValue);
        }
    }

    void setMotorOutput(LazyTalonFX* motor, double output) override 
    {
        motor->set(ControlMode::PercentOutput, output);
    }

    double getMotorCurrent(LazyTalonFX* motor) override 
    {
        return motor->getStatorCurrent();
    }
};