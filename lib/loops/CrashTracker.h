#pragma once
#include <iostream>
#include <fstream>
#include <ctime>
#include <cstdlib>

class CrashTracker {
private:
    std::string generateUniqueID() {
        std::string uniqueID;
        // 使用当前时间戳作为一部分标识符
        time_t now = std::time(0);
        uniqueID += std::to_string(now);

        // 使用随机数作为另一部分标识符
        srand(time(0));
        int randomNum = rand() % 10000;
        uniqueID += std::to_string(randomNum);

        return uniqueID;
    }

    void logMarker(std::string mark, std::exception* nullableException = nullptr) {
        std::ofstream outputFile("/home/lvuser/crash_tracking.txt", std::ios_base::app);

        std::string uuidString = generateUniqueID();
        
        time_t currentTime = std::time(0);  // 添加这行来获取当前时间

        outputFile << uuidString << ", " << mark << ", " << std::ctime(&currentTime);

        if (nullableException != nullptr) {
            outputFile << ", " << nullableException->what();
        }

        outputFile << "\n";
        outputFile.close();
    }

public:
    void logRobotStartup() {
        logMarker("robot startup");
    }

    void logRobotConstruction() {
        logMarker("robot construction");
    }

    void logRobotInit() {
        logMarker("robot init");
    }

    void logTeleopInit() {
        logMarker("teleop init");
    }

    void logAutoInit() {
        logMarker("auto init");
    }

    void logDisabledInit() {
        logMarker("disabled init");
    }

    void logThrowableCrash(std::exception& throwable) {
        logMarker("Exception", &throwable);
        throw throwable; // 继续抛出异常
    }
};