#include "api.h"
#include <climits>
#include "util/timer.hpp"

Timer::Timer(int time, callback_function cb) : time(time), callback(cb) {
    pros::delay(100); // Small delay to ensure task is properly initialized
}

Timer::~Timer() {
    // This stops the background thread and frees its memory
    task.remove(); 
}

void Timer::start() {
    if (!running) {
        pros::lcd::print(5, "Timer Start Running %d", pros::millis());
        running = true;
        task.resume();
    }
}

void Timer::stop() {
    running = false;
}
