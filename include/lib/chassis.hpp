#pragma once
#include <iostream>
#include "drivetrain.hpp"
#include "odom_sensors.hpp"
#include "pid.hpp"
#include "util/pose.hpp"
#include "pros/rtos.hpp"
#include "util/timer.hpp"

class Chassis {
    protected:
        Drivetrain *drivetrain;
        OdomSensors *odometry;

        Pose *pose;
        PIDController *movePID;
        PIDController *turnPID;

        bool tracking = false;

        /**
         * @brief Calculate the robot's current position based on the odometry sensors. Runs constantly in parallel with other tasks.
        */
        void trackPosition();

        /**
         * @brief Starts the tracking task if it is not already running.
         */
        void trackingLoop() {
            pros::Task trackingTask([this]
            {
                while (true) {
                    while (tracking) {
                        trackPosition();
                        pros::delay(20); // avoid tight loop
                    }
                    pros::delay(20);
                }
            });
        }

        /**
         * @brief Scales an input value based on the selected input scaling method.
         * @param input The input value to scale (-127 to 127). 
         * @return The scaled input value.
         */
        double scaleInput(int input);

    public:
        static std::atomic<bool> isAtSetpoint;
        static std::atomic<bool> enableTurning;

        enum InputScale {
            LINEAR,
            CUBIC,
            QUINTIC,
            SIN,
            SINSQUARED,
            TAN,
            XTAN
        };

        InputScale inputScale = LINEAR;

        /**
         * @brief Construct a new Chassis object with full odometry and autonomous capabilities.
         * @param drivetrain Pointer to the drivetrain.
         * @param odometry Pointer to the odometry.
         * @param movePID Pointer to the move PID controller, used for translation movements.
         * @param turnPID Pointer to the turn PID controller, used for rotation.
         */
        Chassis(Drivetrain *drivetrain, OdomSensors *odometry, PIDController *movePID, PIDController *turnPID)
        : drivetrain(drivetrain), odometry(odometry), movePID(movePID), turnPID(turnPID), pose(new Pose()) {}

        /**
         * @brief Construct a new Chassis object with a drivetrain and odometry. 
         * This chassis will have full odometry capabilities, but will not have autonomous features.
         * @param drivetrain Pointer to the drivetrain.
         * @param odometry Pointer to the odometry.
         */
        Chassis(Drivetrain *drivetrain, OdomSensors *odometry)
        : drivetrain(drivetrain), odometry(odometry), movePID(nullptr), turnPID(nullptr), pose(new Pose()) {}

        /**
         * @brief Construct a new Chassis object with a drivetrain and PID controllers. 
         * This chassis will not have odometry capabilities, but will have basic autonomous capabilities.
         * @param drivetrain Pointer to the drivetrain.
         * @param movePID Pointer to the move PID controller, used for translation movements.
         * @param turnPID Pointer to the turn PID controller, used for rotation.
         */
        Chassis(Drivetrain *drivetrain, PIDController *movePID, PIDController *turnPID) 
        : drivetrain(drivetrain), odometry(nullptr), movePID(movePID), turnPID(turnPID) {}

        /**
         * @brief Construct a new Chassis object with only a drivetrain. 
         * This chassis will not have odometry capabilities nor autonomous features.
         * @param drivetrain Pointer to the drivetrain.
         */
        Chassis(Drivetrain *drivetrain) 
        : drivetrain(drivetrain), odometry(nullptr), movePID(nullptr), turnPID(nullptr) {}
 
        /**
         * @brief Sets the input scaling method. The input scaling affects how joystick inputs are translated to motor speeds.
         * 
         * LINEAR: Direct mapping.
         * 
         * CUBIC: Cubic curve for finer control at low speeds.
         * 
         * QUINTIC: Quintic curve for even finer control at low speeds.
         * 
         * SIN: Sine curve for smooth acceleration.
         * 
         * SINSQUARED: Sine squared curve for smooth acceleration and fine control at low speeds.
         * 
         * TAN: Tangent for aggressive acceleration. (may be unstable at high inputs)
         * 
         * XTAN: Exponential tangent curve for fine control at low speeds and aggressive at high speeds. (may be unstable at high inputs)
         * 
         * Link to a graphical representation of these curves: https://www.desmos.com/calculator/xrfbyvksxi
         * 
         * @param scale The input scaling method to set.
         */
        void setInputScale(InputScale scale);

        /**
         * @brief Resets the pose and all of the robot's sensors to their initial state.
         */
        void reset();

        /**
         * @brief Forcefully stop the robot's motors.
         */
        void stop();

        /**
         * @brief Enables the odometry tracking loop of the robot.
         * Call this before using odometry-reliant methods.
         */
        void startTracking() { tracking = true; }

        /**
         * @brief Get the robot's current heading in the world frame.
         * @return The robot's current world frame heading in radians.
         */        
        double getWorldFrameHeading();

        /**
         * @brief Get the robot's current pose (position and orientation).
         * @return The robot's current pose.
         */
        Pose getPose() const;

        /**
         * @brief Set the robot's current pose (position and orientation).
         * @param newPose The new pose to set.
         */
        void setPose(const Pose& newPose);

        /**
         * @brief Set the robot's current pose (position and orientation) using individual values.
         * @param x The new x-coordinate.
         * @param y The new y-coordinate.
         * @param theta The new orientation (in radians).
         */
        void setPose(double x, double y, double theta);

        /**
         * @brief Sets the brake mode for the drivetrain.
         * @param mode The brake mode to set.
         */
        void setBrakeMode(pros::motor_brake_mode_e_t mode);

        struct moveToPoseParams {
            Pose targetPose;
            int timeout = 3000;
            int smallErrorTimeout = 500;
            int largeErrorTimeout = 1000;
            int maxMoveSpeed = 50;
            int maxMoveAccel = 0;
            float smallMoveErrorRange = 1.5;
            float largeMoveErrorRange = 2.5;
            int maxTurnSpeed = 30;
            int maxTurnAccel = 0;
            float smallTurnErrorRange = 1;
            float largeTurnErrorRange = 2;
            int turnStartTime = 0;
            float minAlignDistance = 5.0;
        };

        /**
         * @brief Move the robot to a specific position using PID control. This method blocks until the target position is reached.
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
         */
        void virtual moveToPose(moveToPoseParams params) = 0;

        struct turnToAngleParams {
            int targetAngle = 0;
            int timeout = 1500;
            int maxTurnSpeed = 50;
            int maxTurnAccel = 0;
            float smallErrorRange = 1;
            float largeErrorRange = 2;
            int smallErrorTimeout = 300;
            int largeErrorTimeout = 600;
        };

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
        void virtual turnToAngle(turnToAngleParams params) = 0;
};