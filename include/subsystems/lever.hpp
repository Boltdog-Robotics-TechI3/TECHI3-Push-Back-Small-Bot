#pragma once

#include "main.h"

inline int leverSpeed = 127;

enum LeverState {
    IDLE,
    INTAKING,
    OUTTAKING,
    SCORING
};

// Initializes the intake system motors, setting brake mode
void leverInitialize();

// Sets the current lever state
void setLeverState(LeverState state);

// Gets the current lever state
LeverState getLeverState();

// Polls the controller and determines lever motor, lift pistion, and wing piston states
void leverPeriodic();

// Sets the speed of the intake motor to the specified parameter
void setIntakeSpeed(int speed);

// Sets the speed of the lift motor to the specified parameter
void setLeverSpeed(int speed);

// Sets the 
void setLeverPosition(float targetPosition, float maxVel, float maxAccel);

