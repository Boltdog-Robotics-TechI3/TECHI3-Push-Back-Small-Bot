#pragma once

#include "x_drivetrain.hpp"
#include "chassis.hpp"
#include "odom_sensors.hpp"
#include "pid.hpp"
#include "util/pose.hpp"
#include "pros/rtos.hpp"

class XChassis : public Chassis {
    public:
        /**
         * @brief Construct a new xChassis object with full odometry and autonomous capabilities.
         * @param drivetrain Pointer to the x drivetrain.
         * @param odometry Pointer to the odometry.
         * @param movePID Pointer to the move PID controller, used for translation movements.
         * @param turnPID Pointer to the turn PID controller, used for rotation.
         */
        XChassis(XDrivetrain *drivetrain, OdomSensors *odometry, PIDController *movePID, PIDController *turnPID)
        : Chassis(drivetrain, odometry, movePID, turnPID) {}

        /**
         * @brief Construct a new xChassis object with a drivetrain and odometry. 
         * This xChassis will have full odometry capabilities, but will not have autonomous features.
         * @param drivetrain Pointer to the x drivetrain.
         * @param odometry Pointer to the odometry.
         */
        XChassis(XDrivetrain *drivetrain, OdomSensors *odometry)
        : Chassis(drivetrain, odometry) {}

        /**
         * @brief Construct a new xChassis object with a drivetrain and PID controllers. 
         * This XChassis will not have odometry capabilities, but will have basic autonomous capabilities.
         * @param drivetrain Pointer to the x drivetrain.
         * @param movePID Pointer to the move PID controller, used for translation movements.
         * @param turnPID Pointer to the turn PID controller, used for rotation.
         */
        XChassis(XDrivetrain *drivetrain, PIDController *movePID, PIDController *turnPID) 
        : Chassis(drivetrain, movePID, turnPID) {}

        /**
         * @brief Construct a new xChassis object with only a drivetrain. 
         * This xChassis will not have odometry capabilities nor autonomous features.
         * @param drivetrain Pointer to the x drivetrain.
         */
        XChassis(XDrivetrain *drivetrain) 
        : Chassis(drivetrain) {}

        /**
        * @brief Drive the robot at a specific angle with translational and rotational speeds.
        * @param angle The angle to drive in radians.
        * @param transSpeed The translational speed.
        * @param rotSpeed The rotational speed.
        */
        void driveAngle(double angle, int transSpeed, int rotSpeed);

        /**
         * @brief Move the robot in field-centric mode using joystick inputs.
         * @param leftX The x-value of the left joystick.
         * @param leftY The y-value of the left joystick.
         * @param rightX The x-value of the right joystick.
         */
        void fieldCentricDrive(int leftX, int leftY, int rightX);

        /**
         * @brief Move the robot in robot-centric mode using joystick inputs.
         * @param leftX The x-value of the left joystick.
         * @param leftY The y-value of the left joystick.
         * @param rightX The x-value of the right joystick.
         */
        void robotCentricDrive(int leftX, int leftY, int rightX);

        /**
         * @brief Move the robot in field-centric mode using joystick inputs with direct heading control.
         * The left joystick controls translation, and the right joystick controls the angle the robot is facing.
         * @param leftX The x-value of the left joystick.
         * @param leftY The y-value of the left joystick.
        *  @param rightX The x-value of the right joystick.
         * @param rightY The y-value of the right joystick.
         */
        void fieldCentricHeadingDrive(int leftX, int leftY, int rightX, int rightY);
        
        /**
         * @brief Move the robot to a specific position using PID control. This method blocks until the target position is reached.
         * 
         * @note This method will obey the angle of the target pose while driving to the x and y coordinates.
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
        void moveToPose(moveToPoseParams params) override;

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
        void turnToAngle(turnToAngleParams params) override;
};