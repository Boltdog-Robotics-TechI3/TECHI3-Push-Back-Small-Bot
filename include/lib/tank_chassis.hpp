#pragma once

#include "tank_drivetrain.hpp"
#include "chassis.hpp"
#include "odom_sensors.hpp"
#include "pid.hpp"
#include "util/pose.hpp"
#include "pros/rtos.hpp"

class TankChassis : public Chassis {
    private:

        PIDController *alignPID;
    public:
        /**
         * @brief Construct a new TankChassis object with full odometry and autonomous capabilities.
         * @param drivetrain Pointer to the Tank drivetrain.
         * @param odometry Pointer to the Odom Sensors.
         * @param movePID Pointer to the move PID controller, used for translation movements.
         * @param turnPID Pointer to the turn PID controller, used for rotation.
         * @param alignPID Pointer to the align PID Controller, used in conjuction with movePID to control the heading.
         */
        TankChassis(TankDrivetrain *drivetrain, OdomSensors *odometry, PIDController *movePID, PIDController *turnPID, PIDController *alignPID)
        : alignPID(alignPID), Chassis(drivetrain, odometry, movePID, turnPID) {}

        /**
         * @brief Construct a new TankChassis object with a drivetrain and odometry. 
         * This TankChassis will have full odometry capabilities, but will not have autonomous features.
         * @param drivetrain Pointer to the Tank drivetrain.
         * @param odometry Pointer to the odometry.
         */
        TankChassis(TankDrivetrain *drivetrain, OdomSensors *odometry)
        : alignPID(nullptr), Chassis(drivetrain, odometry) {}

        /**
         * @brief Construct a new TankChassis object with a drivetrain and PID controllers. 
         * This TankChassis will not have odometry capabilities, but will have basic autonomous capabilities.
         * @param drivetrain Pointer to the Tank drivetrain.
         * @param movePID Pointer to the move PID controller, used for translation movements.
         * @param turnPID Pointer to the turn PID controller, used for rotation.
         * @param alignPID Pointer to the align PID Controller, used in conjuction with movePID to control the heading.
         */
        TankChassis(TankDrivetrain *drivetrain, PIDController *movePID, PIDController *turnPID, PIDController *alignPID) 
        : alignPID(alignPID),  Chassis(drivetrain, movePID, turnPID) {}

        /**
         * @brief Construct a new TankChassis object with only a drivetrain. 
         * This TankChassis will not have odometry capabilities nor autonomous features.
         * @param drivetrain Pointer to the Tank drivetrain.
         */
        TankChassis(TankDrivetrain *drivetrain) 
        : alignPID(nullptr), Chassis(drivetrain) {}

         /**
         * @brief Move the robot in arcade mode. The left joystick controls the forward/backward movement, and the right joystick controls the rotation.
         * @param leftY The value of the left joystick (forward/backward movement).
         * @param rightX The value of the right joystick (rotation).
         */
        void arcade(int leftY, int rightX);

        /**
         * @brief Move the robot in tank mode. The left joystick controls the left side motors, and the right joystick controls the right side motors.
         * @param leftY The value of the left joystick (left side motors).
         * @param rightY The value of the right joystick (right side motors).
         */
        void tank(int leftY, int rightY);

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
         * @param maxMoveAccel The maximum acceleration and decceleration the robot can reach (defaults to 127). 
         * @param smallMoveErrorRange The range the move error needs to be within for the small error timeout in inches (defaults to 0.5 inches)
         * @param largeMoveErrorRange The range the move error needs to be within for the large error timeout in inches (defaults to 1.5 inches)
         * @param maxTurnSpeed The maximum speed the robot can turn, from 0 to 127 (defaults to 50)
         * @param maxTurnAccel The maximum acceleration and decceleration the robot's turns can reach (defaults to 127). 
         * @param smallTurnErrorRange The range the turn error needs to be within for the small error timeout in radians (defaults to 0.02 radians)
         * @param largeTurnErrorRange The range the turn error needs to be within for the large error timeout in radians (defaults to 0.04 radians)
         * @param minAlignDistance The minimum distance that the robot needs to be from the target pose for the robot to be allowed to turn (default 5 inches).
         */
        void moveToPose(moveToPoseParams params) override;

        /**
         * @brief Drive the robot a specified distance. Positive distance drives forwards,
         * negative distance drives backwards.
         * 
         * @param distance the distance to drive in inches.
         */
        void moveDistance(double distance, int timeout);

        /**
         * @brief Turn the robot to a specific angle using PID control.
         * 0 Degrees is facing "forward" from the starting orientation.
         * Positive Degrees is counterclockwise, Negative Degrees is clockwise.
         * 
         * @param targetAngle The target angle to turn to (in degrees).
         * @param timeout The amount of time in milliseconds that the robot will try to reach the angle before giving up (default 3000 milliseconds).
         * @param smallErrorTimeout The amount of time in milliseconds that the robot needs to be within the small error range to finish the movement (default 500 milliseconds).
         * @param largeErrorTimeout The amount of time in milliseconds that the robot needs to be within the large error range to finish the movement (default 1000 milliseconds). 
         * @param maxTurnSpeed The maximum speed the robot can turn, from 0 to 127 (defaults to 50)
         * @param maxTurnAccel The maximum acceleration and decceleration the robot's turns can reach (defaults to 127). 
         * @param smallErrorRange The range the turn error needs to be within for the small error timeout in radians (defaults to 0.02 radians)
         * @param largeErrorRange The range the turn error needs to be within for the large error timeout in radians (defaults to 0.04 radians)
         */
        void turnToAngle(turnToAngleParams params) override;
};