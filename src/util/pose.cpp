#include <cmath>
#include <format>
#include "util/pose.hpp"


/**
 * @brief Sets the pose using another Pose object.
 * @param newPose The new pose to set.
*/
void Pose::setPose(Pose newPose) {
    x = newPose.getX();
    y = newPose.getY();
    theta = newPose.getTheta();
}

/**
 * @brief Calculates the distance to another pose.
 * @param other The other pose to calculate the distance to.
 * @return The distance in inches.
*/
double Pose::distanceTo(const Pose& other) {
    double dx = other.getX() - x;
    double dy = other.getY() - y;
    return std::sqrt(dx * dx + dy * dy);
}

/**
 * @brief Calculates the angle to another pose.
 * @param other The other pose to calculate the angle to.
 * @return The angle in radians.
 */
double Pose::angleTo(const Pose& other) {
    double dx = other.getX() - x;
    double dy = other.getY() - y;
    return std::atan2(dy, dx);
}

/**
 * @brief Offsets the current pose by another pose.
 * @param other The pose to offset by.
 * @return A new Pose object representing the offset pose.
 */
Pose Pose::offset(const Pose& other) {
    double newX = x + other.getX();
    double newY = y + other.getY();
    double newTheta = theta + other.getTheta();
    return Pose(newX, newY, newTheta);
}

/**
 * @brief Offsets the current pose by the opposite of another pose.
 * @param other The pose to offset by.
 * @return A new Pose object representing the offset pose.
 */
Pose Pose::negativeOffset(const Pose& other) {
    double newX = x - other.getX();
    double newY = y - other.getY();
    double newTheta = theta - other.getTheta();
    return Pose(newX, newY, newTheta);
}

Pose Pose::rotate(double angle) {
    double magnitude = sqrt((x*x) + (y*y));
    double theta = (atan2(x, y));

    theta += angle;

    return Pose(magnitude * sin(theta), magnitude * cos(theta), theta);
}

std::string Pose::to_string() {
    return std::format("X:{:.2f} Y:{:.2f} T:{:.2f}", x, y, theta);
}
