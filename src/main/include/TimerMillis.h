//timer library to keep track of time in millis

#pragma once

#include "frc/RobotController.h"
#include <iostream>

class TimerMillis
{
private:
    float timeNow;
    float timeElapsed;
    float startTime; 
    float stopTime;
public:
    TimerMillis() {
    }

    ~TimerMillis() {
    }

    void Start() {
        // std::cout << "start Timer" << std::endl;
        startTime = frc::RobotController::GetFPGATime();
        //std::cout << "gggg   start" << std::endl;
    }

    void Stop() {
        stopTime = frc::RobotController::GetFPGATime();
        //std::cout << "gggg             stop  " << std::endl;
    }

    void Reset() {
        timeNow = 0;
        timeElapsed = 0;
        startTime = 0;
        stopTime = 0;
        //std::cout << "gggg                        reset" << std::endl;
    }

    uint64_t Get() {
        timeNow = frc::RobotController::GetFPGATime();
        timeElapsed = timeNow - startTime;
        timeElapsed /= 1000;   // convert from microseconds to milliseconds
        //std::cout << "timeElapsed    " << timeElapsed << std::endl;
        return timeElapsed;
    }
};
