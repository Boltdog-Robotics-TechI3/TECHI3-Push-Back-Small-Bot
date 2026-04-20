#include "lib/chassis.hpp"
#include <cmath>

std::atomic<bool> Chassis::isAtSetpoint = false; 
std::atomic<bool> Chassis::enableTurning = false; 

/**
 * @brief Scales an input value based on the selected input scaling method.
 * @param input The input value to scale (-127 to 127). 
 * @return The scaled input value.
 */
double Chassis::scaleInput(int input) {
    double normalizedInput = (double)input / 127.0;
    double scaledInput = 0.0;

    bool isNegative = normalizedInput < 0;

    switch (inputScale) {
        case LINEAR:
            scaledInput = normalizedInput;
            break;
        case CUBIC:
            scaledInput = normalizedInput * normalizedInput * normalizedInput;
            break;
        case QUINTIC:
            scaledInput = normalizedInput * normalizedInput * normalizedInput * normalizedInput * normalizedInput;
            break;
        case SIN:
            scaledInput = sin((normalizedInput * M_PI) / 2);
            break;
        case SINSQUARED:
            scaledInput = sin((normalizedInput * M_PI) / 2);
            scaledInput = scaledInput * scaledInput;
            break;
        case TAN:
            scaledInput = tan(normalizedInput);
            break;
        case XTAN:
            scaledInput = normalizedInput * tan(normalizedInput);
            break;
        default:
            scaledInput = normalizedInput;
    }

    if (isNegative && scaledInput > 0) {
        scaledInput = -scaledInput;
    }

    return scaledInput * 127.0;
}

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
void Chassis::setInputScale(InputScale scale) {
    inputScale = scale;
}

/**
 * @brief Resets the pose and all of the robot's sensors to their initial state.
 */
void Chassis::reset() {
    if (odometry) {
        odometry->reset();
    }
    if (drivetrain) {
        drivetrain->setMotorSpeeds({0, 0, 0, 0, 0});
    }
    if (!tracking) {
        trackingLoop();
    }
}

/**
 * @brief Forcefully stop the robot's motors.
 */
void Chassis::stop() {
    if (drivetrain) {
        drivetrain->setMotorSpeeds({0, 0, 0, 0, 0});
    }
}

double Chassis::getWorldFrameHeading() {
    if (odometry && odometry->imu) {
        return -1 * odometry->getRotationRadians() + M_PI_2;
    } else {
        return 0;
    }
}

/**
 * @brief Get the robot's current pose (position and orientation).
 * @return The robot's current pose.
 */
Pose Chassis::getPose() const { 
    return *pose; 
}

/**
 * @brief Set the robot's current pose (position and orientation).
 * @param newPose The new pose to set.
 */
void Chassis::setPose(const Pose& newPose) { 
    *pose = newPose; 
    odometry->imu->set_rotation((newPose.getTheta() * (180 / M_PI)) * -1);
}

/**
 * @brief Set the robot's current pose (position and orientation) using individual values.
 * @param x The new x-coordinate.
 * @param y The new y-coordinate.
 * @param theta The new orientation (in radians).
 */
void Chassis::setPose(double x, double y, double theta) {
    pose->setX(x);
    pose->setY(y);
    pose->setTheta(theta);
    odometry->imu->set_rotation((theta * (180 / M_PI)) * -1);
}

/**
 * @brief Sets the brake mode for the chassis.
 * @param mode The brake mode to set.
 */
void Chassis::setBrakeMode(pros::motor_brake_mode_e_t mode) {
    if (drivetrain) {
        drivetrain->setBrakeMode(mode);
    }   
}   

//https://thepilons.ca/wp-content/uploads/2018/10/Tracking.pdf
#warning TODO: Make tracking work with different odometry setups
void Chassis::trackPosition() {
    // Get current position
    std::array<double, 3> currentPose = odometry->getReadings();

    double currentVertical = currentPose[0];
    double currentHorizontal = currentPose[1];

    // Calculate changes since last reading
    double previousVertical = odometry->verticalWheel->getLastPosition();
    double previousHorizontal = odometry->horizontalWheel->getLastPosition();

    double verticalChange = currentVertical - previousVertical;
    double horizontalChange = currentHorizontal - previousHorizontal;

    // Update previous positions
    odometry->verticalWheel->setLastPosition(currentVertical);
    //odometry->rightWheel->setLastPosition(currentRight);
    odometry->horizontalWheel->setLastPosition(currentHorizontal);

    Pose formerPosition = getPose();

    // Calculate the change in orientation
    double delTheta = -1 * odometry->getRotationRadians() - formerPosition.getTheta();
    
    // Calculate local displacement vector
    double deltaDl[2]; 
    if(delTheta == 0){
        deltaDl[0] = horizontalChange;
        deltaDl[1] = verticalChange;
    } else {
        deltaDl[0] = (2 * sin(delTheta / 2)) * ((horizontalChange / delTheta) + (odometry->horizontalWheel->getOffset()));
        deltaDl[1] = (2 * sin(delTheta / 2)) * ((verticalChange / delTheta) + (odometry->verticalWheel->getOffset()));
    }
    Pose deltaD = Pose(deltaDl[0], deltaDl[1], delTheta);
    
    // Calculate average orientation
    double thetaM = formerPosition.getTheta() + (delTheta / 2);

    // Rotate local displacement to global frame
    deltaD = deltaD.rotate(-1*thetaM);

    // Update the position
    setPose(formerPosition.getX() + deltaD.getX(), formerPosition.getY() + deltaD.getY(), -1 * odometry->getRotationRadians());
}
