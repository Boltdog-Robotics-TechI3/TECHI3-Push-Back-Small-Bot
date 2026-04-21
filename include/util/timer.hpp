#pragma once
#include "api.h"
#include "pros/rtos.hpp"

typedef void (*callback_function)();

class Timer {
    private:
        callback_function callback;
        int time = 0; // in milliseconds
        int startTime = 0;
        bool initialized = false;
        std::atomic<bool> running = false;
        pros::Task task = pros::Task([this]() {
            while (true) {
                
                // Check if the task restarts upon resuming
                // Ensure the correct task is being suspended/resumed
                // Maybe use pros::Task::current() instead of this->task
                pros::lcd::print(2, "Timer Running %d", pros::millis());
                task.suspend();
                startTime = pros::millis();
                pros::lcd::print(0, "Timer Resumed %d", pros::millis());
                while (running && pros::millis() - startTime < time) { pros::delay(20); }
                if (running && callback) {
                    running = false;
                    callback();
                }
            }
        });
    public:
        Timer(int time, callback_function cb);
        ~Timer();
        void start();
        void stop();
        bool isRunning() { return running; }
        void setTime(int t) { time = t; }
        int getTime() { return time; }
};