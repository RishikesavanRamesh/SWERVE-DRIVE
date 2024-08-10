#include <array>
#include <iostream>
#include <cmath>
#include <vector>

// Forward declarations
class SwerveModuleState;
class Translation2D;
class ChassisSpeeds;

// Translation2D class definition
class Translation2D // used to specify the location of swerve modules from robot frame( which is at the center of the robot ---> front towards + y )
{
public:
    double translation_x;
    double translation_y;

    // Default constructor
    Translation2D() : translation_x(0.0), translation_y(0.0) {}

    Translation2D(double x, double y) : translation_x(x), translation_y(y) {}
};

// ChassisSpeeds class definition (assuming it's defined somewhere)
class ChassisSpeeds
{
public:
    double velocity_x; // this represents the robot moving front which is seen from outside as chasis moving in positive x direction, but the robot frame actually is ( y ; facing front positively , x in right direction positively )
    double velocity_y;
    double omega_z;

    ChassisSpeeds(double vx, double vy, double oz)
        : velocity_x(-vy), velocity_y(vx), omega_z(oz) {} // here the input value of chasis ( +x as the faceof robot  ) is rotated for internal calculations

    // Static method to adjust ChassisSpeeds based on rotation angle
    static ChassisSpeeds FromFieldRelativeSpeeds(const ChassisSpeeds &fieldSpeeds, double robotRotatedAngle)
    {
        // Calculate cosine and sine of angle
        double cosAngle = std::cos(robotRotatedAngle);
        double sinAngle = std::sin(robotRotatedAngle);
        // here the following equations may seem like using left hand coordinate system, but it is done so, to calculate the robot wheels' rotation, when the robot is rotated in the field reference system, the passed argument is the angle robot is turned with reference to the field .
        double rotated_vx = fieldSpeeds.velocity_x * cosAngle + fieldSpeeds.velocity_y * sinAngle;
        double rotated_vy = -fieldSpeeds.velocity_x * sinAngle + fieldSpeeds.velocity_y * cosAngle;
        double rotated_omega = fieldSpeeds.omega_z; // Omega (angular velocity) remains unchanged

        return ChassisSpeeds(rotated_vx, rotated_vy, rotated_omega);
    }
};

// SwerveModuleState class definition
class SwerveModuleState
{
public:
    double velocity; // in mps
    double angle;    // in radians per seconds should only return in positive angle

    // Default constructor
    SwerveModuleState() : velocity(0.0), angle(0.0) {}

    SwerveModuleState(double v, double a) : velocity(v), angle(a) {}

    // Static method to optimize the angle
    // Static method to optimize the angle
    static SwerveModuleState optimize(const SwerveModuleState &state, double currentAngle)
    {

        double angleDifference = std::abs(currentAngle - state.angle);

        // Determine if the angle difference is acute or obtuse
        if (angleDifference > M_PI / 2)
        {
            /// Obtuse angle case

            // for returning only positive angles
            double angle = state.angle - M_PI;

            angle = std::fmod(angle + 2 * M_PI, 2 * M_PI);
            return SwerveModuleState(-state.velocity, std::fmod(angle, 2 * M_PI));
        }
        else
        {
            // Acute angle case
            return SwerveModuleState(state.velocity, std::fmod(state.angle, 2 * M_PI));
        }
    }
};

class SwerveDriveKinematics
{
public:
    SwerveDriveKinematics(const std::vector<Translation2D> &modules) : states(modules.begin(), modules.end())
    {
        // No need for additional initialization
    }

    std::vector<SwerveModuleState> toSwerveModuleStates(const ChassisSpeeds &speeds, const Translation2D &centerOfRotation = Translation2D()) const
    {
        std::vector<SwerveModuleState> result;

        for (size_t i = 0; i < states.size(); ++i)
        {

            // Compute the components of velocity in the rotated frame
            double velocity_x = speeds.velocity_x - (speeds.omega_z * (states[i].translation_y - centerOfRotation.translation_y));
            double velocity_y = speeds.velocity_y + (speeds.omega_z * (states[i].translation_x - centerOfRotation.translation_x));

            // Compute the magnitude of the resultant velocity vector
            double velocity = std::sqrt(velocity_x * velocity_x + velocity_y * velocity_y);

            // Compute the angle of the resultant velocity vector
            double angle = std::atan2(velocity_y, velocity_x);

            // for returning only positive angles
            angle = std::fmod(angle + 2 * M_PI, 2 * M_PI);

            result.push_back(SwerveModuleState(velocity, angle));
        }

        return result;
    }

private:
    std::vector<Translation2D> states;
};
