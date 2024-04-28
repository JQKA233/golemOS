#pragma once
#include <iostream>
#include <vector>
#include <functional>
#include <thread>
#include <numeric>
#include <math.h>

class Subsystem {};

template<class T>
class MotorChecker{
public:
    struct CheckerConfig {
        double mCurrentFloor = 5;
        double mRPMFloor = 2000;

        double mCurrentEpsilon = 5.0;
        double mRPMEpsilon = 500;
        std::function<double()> mRPMSupplier = nullptr;

        double mRunTimeSec = 4.0;
        double mWaitTimeSec = 2.0;
        double mRunOutputPercentage = 0.5;
    };

    template <class U>
    struct MotorConfig {
        std::string mName;
        U* mMotor;
        MotorConfig(std::string name, U* motor) : mName(name), mMotor(motor) {}
    };

    std::vector<MotorConfig<T>> mMotorsToCheck;


    virtual void storeConfiguration() 
    {

    }


    virtual void restoreConfiguration()
    {

    }


    virtual void setMotorOutput(T* motor, double output)
    {

    }


    virtual double getMotorCurrent(T* motor)
    {
        return 0.0;
    }


    bool checkMotorsImpl(Subsystem* subsystem, std::vector<MotorConfig<T>> motorsToCheck, CheckerConfig checkerConfig) {
        bool failure = false;
        std::cout << "////////////////////////////////////////////////" << std::endl;
        std::cout << "Checking subsystem " << typeid(subsystem).name()
            << " for " << motorsToCheck.size() << " motors." << std::endl;

        std::vector<double> currents;
        std::vector<double> rpms;

        mMotorsToCheck = motorsToCheck;
        storeConfiguration();

        for (MotorConfig<T>& config : motorsToCheck) {
            setMotorOutput(config.mMotor, 0.0);
        }

        for (MotorConfig<T>& config : motorsToCheck) {
            std::cout << "Checking: " << config.mName << std::endl;

            setMotorOutput(config.mMotor, checkerConfig.mRunOutputPercentage);
            std::this_thread::sleep_for(std::chrono::duration<double>(checkerConfig.mRunTimeSec));

            // poll the interesting information
            double current = getMotorCurrent(config.mMotor);
            currents.push_back(current);
            std::cout << "Current: " << current;

            double rpm = NAN;
            if (checkerConfig.mRPMSupplier != nullptr) {
                rpm = checkerConfig.mRPMSupplier();
                rpms.push_back(rpm);
                std::cout << " RPM: " << rpm;
            }
            std::cout << std::endl;

            setMotorOutput(config.mMotor, 0.0);

            // perform checks
            if (current < checkerConfig.mCurrentFloor) {
                std::cout << config.mName << " has failed current floor check vs " <<
                    checkerConfig.mCurrentFloor << "!!" << std::endl;
                failure = true;
            }
            if (!std::isnan(rpm) && rpm < checkerConfig.mRPMFloor) {
                std::cout << config.mName << " has failed rpm floor check vs " <<
                    checkerConfig.mRPMFloor << "!!" << std::endl;
                failure = true;
            }

            std::this_thread::sleep_for(std::chrono::duration<double>(checkerConfig.mWaitTimeSec));
        }

        // run aggregate checks

        if (!currents.empty()) {
            double average = std::accumulate(currents.begin(), currents.end(), 0.0) / currents.size();

            if (!std::all_of(currents.begin(), currents.end(), [average, checkerConfig](double val) {
                return std::abs(val - average) < checkerConfig.mCurrentEpsilon;
            })) {
                std::cout << "Currents varied!!!!!!!!!!!" << std::endl;
                failure = true;
            }
        }

        if (!rpms.empty()) {
            double average = std::accumulate(rpms.begin(), rpms.end(), 0.0) / rpms.size();

            if (!std::all_of(rpms.begin(), rpms.end(), [average, checkerConfig](double val) {
                return std::abs(val - average) < checkerConfig.mRPMEpsilon;
            })) {
                std::cout << "RPMs varied!!!!!!!!" << std::endl;
                failure = true;
            }
        }

        // restore talon configurations
        restoreConfiguration();

        return !failure;
    }
};
