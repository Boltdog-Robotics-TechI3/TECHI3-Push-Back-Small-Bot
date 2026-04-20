#include <cmath>
#include "lib/tank_chassis.hpp"
#include "pros/rtos.hpp"

/**
 * @brief Move the robot in arcade mode. The left joystick controls the forward/backward movement, and the right joystick controls the rotation.
 * @param leftY The value of the left joystick (forward/backward movement).
 * @param rightX The value of the right joystick (rotation).
 */
void TankChassis::arcade(int leftY, int rightX) {
    if (drivetrain) {
        int scaledLeftY = scaleInput(leftY);
        int scaledRightX = scaleInput(rightX);
        int leftPower = scaledLeftY + scaledRightX;
        int rightPower = scaledLeftY - scaledRightX;
        drivetrain->setMotorSpeeds({leftPower, rightPower});
    }
}

/**
 * @brief Move the robot in tank mode. The left joystick controls the left side motors, and the right joystick controls the right side motors.
 * @param leftY The value of the left joystick (left side motors).
 * @param rightY The value of the right joystick (right side motors).
 */
void TankChassis::tank(int leftY, int rightY) {
    if (drivetrain) {
        int scaledLeftY = scaleInput(leftY);
        int scaledRightY = scaleInput(rightY);
        drivetrain->setMotorSpeeds({scaledLeftY, scaledRightY});
    }
}

/** 
 * @brief Move the robot to a specific position using PID control. This method blocks until the target position is reached.
 * This method will also turn to align itself with the target pose.
 * 
 * @note This method ignores the angle of the target pose and only drives to the x and y coordinates.
 * 
 * @param targetPose The target pose to move to (defaults to the origin).
 * @param timeout The amount of time in milliseconds that the robot will try to reach the pose before giving up (default 3000 milliseconds).
 * @param smallErrorTimeout The amount of time in milliseconds that the robot needs to be within the small error range to finish the movement (default 500 milliseconds).
 * @param largeErrorTimeout The amount of time in milliseconds that the robot needs to be within the large error range to finish the movement (default 1000 milliseconds). 
 * @param maxMoveSpeed The maximum speed the robot can travel, from 0 to 127 (defaults to 50)
 * @param maxMoveAccel The maximum acceleration and decceleration the robot can reach (defaults to 0, aka no limit). 
 * @param smallMoveErrorRange The range the move error needs to be within for the small error timeout in inches (defaults to 1.5 inches)
 * @param largeMoveErrorRange The range the move error needs to be within for the large error timeout in inches (defaults to 2.5 inches)
 * @param maxTurnSpeed The maximum speed the robot can turn, from 0 to 127 (defaults to 30)
 * @param maxTurnAccel The maximum acceleration and decceleration the robot's turns can reach (defaults to 0, aka no limit). 
 * @param smallTurnErrorRange The range the turn error needs to be within for the small error timeout in degrees (defaults to 1 degree)
 * @param largeTurnErrorRange The range the turn error needs to be within for the large error timeout in degrees (defaults to 2 degrees)
 * @param turnStartTime The amount of time elapsed since the start of the movement where, once reached, the robot is allowed to begin turning (defaults to 0 milliseconds, meaning it can start turning instantly).
 * @param minAlignDistance The minimum distance that the robot needs to be from the target pose for the robot to be allowed to turn (default 5 inches).
 */
void TankChassis::moveToPose(moveToPoseParams params) {
    if (!movePID || !alignPID) {
        return;
    }
    if (timeoutTimer == nullptr) {
        timeoutTimer = new Timer(params.timeout, +[]() { Chassis::isAtSetpoint = true; }); 
        smallErrorTimer = new Timer(params.smallErrorTimeout, +[]() { Chassis::isAtSetpoint = true; });
        largeErrorTimer = new Timer(params.largeErrorTimeout, +[]() { Chassis::isAtSetpoint = true; });
        failsafeTimer = new Timer(100, +[]() { Chassis::isAtSetpoint = true; });
        enableTurningTimer = new Timer(params.turnStartTime, +[]() { Chassis::enableTurning = true; });
    }

    isAtSetpoint = false;
    enableTurning = false;

    // Calculate starting angular error to determine if robot should drive forwards or backwards
    float testAngleError = (pose->angleTo(params.targetPose) - M_PI_2) - fmod((pose->getTheta() + 2*M_PI), 2*M_PI);
    if (testAngleError > M_PI) {
        testAngleError = testAngleError - 2*M_PI;
    } 
    else if (testAngleError < -M_PI) {
        testAngleError = 2*M_PI + testAngleError;
    }

    // If error is greater than PI/2, we are probably trying to go backwards (hopefully).
    bool isBackwards = abs(testAngleError) >= M_PI_2;

    float error = 0;
    float angularError = 0;
    int linearOutput = 0;
    int angularOutput = 0;

    timeoutTimer->setTime(params.timeout);
    smallErrorTimer->setTime(params.smallErrorTimeout);
    largeErrorTimer->setTime(params.largeErrorTimeout);
    enableTurningTimer->setTime(params.turnStartTime);

    movePID->reset();
    alignPID->reset();

    movePID->setOutputLimits(-params.maxMoveSpeed, params.maxMoveSpeed);
    movePID->setSmallErrorRange(params.smallMoveErrorRange);
    movePID->setLargeErrorRange(params.largeMoveErrorRange);
    movePID->setSlewRate(params.maxMoveAccel);
    movePID->setIZone(5);

    alignPID->setOutputLimits(-params.maxTurnSpeed, params.maxTurnSpeed);
    alignPID->setSlewRate(params.maxTurnAccel);

    timeoutTimer->start();
    enableTurningTimer->start();
    while (!isAtSetpoint) {
        error = pose->distanceTo(params.targetPose);

        if (fabs(movePID->getPreviousError()) < fabs(error) && error < params.minAlignDistance) {
            failsafeTimer->start();
        }
        else {
            failsafeTimer->stop();
        }
        // std::cout << "error: " << error << "; previous error: " << movePID->getPreviousError() << "; small error: " << movePID->isInSmallErrorRange() << "; large error: " << movePID->isInLargeErrorRange() << std::endl;

        linearOutput = movePID->calculate(0, error) * (isBackwards ? -1 : 1);

        if (error > params.minAlignDistance && enableTurning) {
            // Calculate and normalize the angle error
            angularError = (pose->angleTo(params.targetPose) + (isBackwards ? M_PI : 0) - M_PI_2) - fmod((pose->getTheta() + 2*M_PI), 2*M_PI);
            if (angularError > M_PI) {
                angularError = angularError - 2*M_PI;
            } 
            else if (angularError < -M_PI) {
                angularError = 2*M_PI + angularError;
            }

            angularOutput = alignPID->calculate(0, angularError);

            drivetrain->setMotorSpeeds({linearOutput - angularOutput, linearOutput + angularOutput});
        }
        else {
            drivetrain->setMotorSpeeds({linearOutput, linearOutput});
        }

        if (movePID->isInSmallErrorRange()) {
            smallErrorTimer->start();    
        } else {
            smallErrorTimer->stop();
        }
        if (movePID->isInLargeErrorRange()) {
            largeErrorTimer->start();
        } else {
            largeErrorTimer->stop();
        }  
        
		pros::delay(20);
    }

    smallErrorTimer->stop();
    largeErrorTimer->stop();
    timeoutTimer->stop();
    failsafeTimer->stop();
    enableTurningTimer->stop();
    stop();
}

/**
 * @brief Drive the robot a specified distance. Positive distance drives forwards,
 * negative distance drives backwards.
 * 
 * @param distance the distance to drive in inches.
 */
void TankChassis::moveDistance(double distance, int timeout) {
    if (!movePID || !alignPID || true) {
        return;
    }

    isAtSetpoint = false;

    timeoutTimer->start();
    double initialPosition = odometry ? odometry->getReadings()[0] : (drivetrain->getMotors()[0]->get_position() + drivetrain->getMotors()[1]->get_position()) / 2.0;
    
    movePID->reset();
    alignPID->reset();

    movePID->setOutputLimits(-50, 50);
    movePID->setSmallErrorRange(1);
    movePID->setLargeErrorRange(2);
    movePID->setSlewRate(300);

    alignPID->setOutputLimits(-10, 10);

    while (!isAtSetpoint) {
        double currentPosition = odometry ? odometry->getReadings()[0] : (drivetrain->getMotors()[0]->get_position() + drivetrain->getMotors()[1]->get_position()) / 2.0;

        if (movePID->isInSmallErrorRange()) {
            smallErrorTimer->start();
        }
        else if (movePID->isInLargeErrorRange()) {
            smallErrorTimer->stop();
            largeErrorTimer->start();
        }
        else {
            smallErrorTimer->stop();
            largeErrorTimer->stop();
        }

        int output = movePID->calculate(currentPosition-initialPosition, distance);

        drivetrain->setMotorSpeeds({output, output});

        pros::delay(20);
    }

    timeoutTimer->stop();
    smallErrorTimer->stop();
    largeErrorTimer->stop();
    stop();
}

/**
 * @brief Turn the robot to a specific angle using PID control.
 * 0 Degrees is facing "forward" from the starting orientation.
 * Positive Degrees is counterclockwise, Negative Degrees is clockwise.
 * 
 * @param targetAngle The target angle to turn to (in degrees).
 * @param timeout The amount of time in milliseconds that the robot will try to reach the angle before giving up (default 1500 milliseconds).
 * @param maxTurnSpeed The maximum speed the robot can turn, from 0 to 127 (defaults to 50)
 * @param maxTurnAccel The maximum acceleration and decceleration the robot's turns can reach (defaults to 0, aka no limit). 
 * @param smallErrorRange The range the turn error needs to be within for the small error timeout in degrees (defaults to 1 degree)
 * @param largeErrorRange The range the turn error needs to be within for the large error timeout in degrees (defaults to 2 degrees)
 * @param smallErrorTimeout The amount of time in milliseconds that the robot needs to be within the small error range to finish the movement (default 500 milliseconds).
 * @param largeErrorTimeout The amount of time in milliseconds that the robot needs to be within the large error range to finish the movement (default 1000 milliseconds). 
 */
void TankChassis::turnToAngle(turnToAngleParams params) {
    if (!turnPID) {
        return;
    }
    if (timeoutTimer == nullptr) {
        timeoutTimer = new Timer(params.timeout, +[]() { Chassis::isAtSetpoint = true; }); 
        smallErrorTimer = new Timer(params.smallErrorTimeout, +[]() { Chassis::isAtSetpoint = true; });
        largeErrorTimer = new Timer(params.largeErrorTimeout, +[]() { Chassis::isAtSetpoint = true; });
        failsafeTimer = new Timer(100, +[]() { Chassis::isAtSetpoint = true; });
        enableTurningTimer = new Timer(0, +[]() { Chassis::enableTurning = true; });
    }

    isAtSetpoint = false;

    float targetAngle = Pose::degToRad(params.targetAngle);

    timeoutTimer->setTime(params.timeout);
    smallErrorTimer->setTime(params.smallErrorTimeout);
    largeErrorTimer->setTime(params.largeErrorTimeout);

    turnPID->reset();
    turnPID->setOutputLimits(-params.maxTurnSpeed, params.maxTurnSpeed);
    turnPID->setSmallErrorRange(Pose::degToRad(params.smallErrorRange));
    turnPID->setLargeErrorRange(Pose::degToRad(params.largeErrorRange));
    turnPID->setSlewRate(params.maxTurnAccel);
    turnPID->setIZone(0.1);

    timeoutTimer->start();

    while (!isAtSetpoint) {
        double error = targetAngle - fmod((pose->getTheta() + 2*M_PI), 2*M_PI);
        if (error > M_PI) {
            error = error - 2*M_PI;
        } 
        else if (error < -M_PI) {
            error = 2*M_PI + error;
        }

        int output = turnPID->calculate(0, error);
        drivetrain->setMotorSpeeds({-output, output});

        if (turnPID->isInSmallErrorRange()) {
            smallErrorTimer->start();
        }
        else if (turnPID->isInLargeErrorRange()) {
            smallErrorTimer->stop();
            largeErrorTimer->start();
        }
        else {
            smallErrorTimer->stop();
            largeErrorTimer->stop();
        }

        pros::delay(20);
    }
    smallErrorTimer->stop();
    largeErrorTimer->stop();
    timeoutTimer->stop();
    stop();
}
