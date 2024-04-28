#pragma once
#include <iostream>
#include <fstream>
#include <ctime>
#include <exception>

#include "CrashTracker.h"

class CrashTrackingRunnable {
public:
    virtual void runCrashTracked() = 0;

    void run() {
        try {
            runCrashTracked();
        } catch (std::exception& e) {
            CrashTracker crashTracker;
            crashTracker.logThrowableCrash(e);
            throw e;
        }
    }
};