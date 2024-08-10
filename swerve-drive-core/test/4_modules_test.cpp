#include <gtest/gtest.h>

#include <cmath>
#include "SwerveDriveKinematics.hpp"

const double tolerance = 1e-9;

// creating 4 swerve module
std::vector<Translation2D> modules = {
    Translation2D(0.50, 0.50),
    Translation2D(-0.50, 0.50),
    Translation2D(-0.5, -0.50),
    Translation2D(0.50, -0.50),
};

// Instantiate SwerveDriveKinematics with modules
SwerveDriveKinematics testKinematics(modules);

// 1) one meter forward chasis_speed as input and check the output,
TEST(swerve_drive_core, ModuleStatesForOneMeterPerSecondForwardMovement)
{
    // Creating chasis speed object;
    ChassisSpeeds speeds(01.0, 0.0, 0.0);

    // get moduleStates
    auto moduleStates = testKinematics.toSwerveModuleStates(speeds, Translation2D(0.0, 0.0));

    ASSERT_EQ(M_PI / 2, moduleStates[0].angle);
    ASSERT_EQ(1.0, moduleStates[0].velocity);

    ASSERT_EQ(M_PI / 2, moduleStates[1].angle);
    ASSERT_EQ(1.0, moduleStates[1].velocity);

    ASSERT_EQ(M_PI / 2, moduleStates[2].angle);
    ASSERT_EQ(1.0, moduleStates[2].velocity);

    ASSERT_EQ(M_PI / 2, moduleStates[3].angle);
    ASSERT_EQ(1.0, moduleStates[3].velocity);
}

// 2) oen meter rightwards chasis_speed as input and check the output,
TEST(swerve_drive_core, ModuleStatesForOneMeterPerSecondLeftwardMovement)
{ // Creating chasis speed object;
    ChassisSpeeds speeds(0.0, 1.0, 0.0);

    // get moduleStates
    auto moduleStates = testKinematics.toSwerveModuleStates(speeds, Translation2D(0.0, 0.0));

    ASSERT_EQ(M_PI, moduleStates[0].angle);
    ASSERT_EQ(1.0, moduleStates[0].velocity);

    ASSERT_EQ(M_PI, moduleStates[1].angle);
    ASSERT_EQ(1.0, moduleStates[1].velocity);

    ASSERT_EQ(M_PI, moduleStates[2].angle);
    ASSERT_EQ(1.0, moduleStates[2].velocity);

    ASSERT_EQ(M_PI, moduleStates[3].angle);
    ASSERT_EQ(1.0, moduleStates[3].velocity);
}

TEST(swerve_drive_core, ModuleStatesForOneMeterPerSecondRightwardwardMovement)
{ // Creating chasis speed object;
    ChassisSpeeds speeds(0.0, -1.0, 0.0);

    // get moduleStates
    auto moduleStates = testKinematics.toSwerveModuleStates(speeds, Translation2D(0.0, 0.0));

    ASSERT_EQ(0, moduleStates[0].angle);
    ASSERT_EQ(1.0, moduleStates[0].velocity);

    ASSERT_EQ(0, moduleStates[1].angle);
    ASSERT_EQ(1.0, moduleStates[1].velocity);

    ASSERT_EQ(0, moduleStates[2].angle);
    ASSERT_EQ(1.0, moduleStates[2].velocity);

    ASSERT_EQ(0, moduleStates[3].angle);
    ASSERT_EQ(1.0, moduleStates[3].velocity);
}

TEST(swerve_drive_core, ModuleStatesForOneMeterPerSecondForwardMovementAndOneMeterPerSecondLeftwardMovement)
{ // Creating chasis speed object;
    ChassisSpeeds speeds(01.0, 1.0, 0.0);

    // get moduleStates
    auto moduleStates = testKinematics.toSwerveModuleStates(speeds, Translation2D(0.0, 0.0));

    ASSERT_NEAR(3 * M_PI/4, moduleStates[0].angle, tolerance);
    ASSERT_NEAR(sqrt(2), moduleStates[0].velocity, tolerance);

    ASSERT_NEAR(3 * M_PI/4, moduleStates[1].angle, tolerance);
    ASSERT_NEAR(sqrt(2), moduleStates[1].velocity, tolerance);

    ASSERT_NEAR(3 * M_PI/4, moduleStates[2].angle, tolerance);
    ASSERT_NEAR(sqrt(2), moduleStates[2].velocity, tolerance);

    ASSERT_NEAR(3 * M_PI/4, moduleStates[3].angle, tolerance);
    ASSERT_NEAR(sqrt(2), moduleStates[3].velocity, tolerance);
}

// 3) rotate inplace for 90deg and check if rotating
TEST(swerve_drive_core, ModuleStatesForInPlaceClockwiseRotation)
{ // Creating chasis speed object;
    ChassisSpeeds speeds(0.0, 0.0, 1.0);

    // get moduleStates
    auto moduleStates = testKinematics.toSwerveModuleStates(speeds, Translation2D(0.0, 0.0));

    ASSERT_NEAR(3 * M_PI / 4, moduleStates[0].angle, tolerance);
    ASSERT_NEAR(1 / std::sqrt(2), moduleStates[0].velocity, tolerance);

    ASSERT_NEAR(5 * M_PI / 4, moduleStates[1].angle, tolerance);
    ASSERT_NEAR(1 / std::sqrt(2), moduleStates[1].velocity, tolerance);

    ASSERT_NEAR(7 * M_PI / 4, moduleStates[2].angle, tolerance);
    ASSERT_NEAR(1 / std::sqrt(2), moduleStates[2].velocity, tolerance);

    ASSERT_NEAR(1 * M_PI / 4, moduleStates[3].angle, tolerance);
    ASSERT_NEAR(1 / std::sqrt(2), moduleStates[3].velocity, tolerance);
}

// 4) rotate onemeter to the forward as icr and do 90deg and check if completing
TEST(swerve_drive_core, ModuleStatesForClockwiseRotationWithICRAt1MeterForward)
{
    // Creating chasis speed object;
    ChassisSpeeds speeds(0.0, 0.0, 1.0);

    // get moduleStates
    auto moduleStates = testKinematics.toSwerveModuleStates(speeds, Translation2D(01.0, 0.0));

    ASSERT_NEAR(5 * M_PI / 4, moduleStates[0].angle, tolerance);
    ASSERT_NEAR(1 / std::sqrt(2), moduleStates[0].velocity, tolerance);

    ASSERT_NEAR(251.565051177 * M_PI / 180, moduleStates[1].angle, tolerance);
    ASSERT_NEAR(1.5811388300841898, moduleStates[1].velocity, tolerance);

    ASSERT_NEAR(288.434948823 * M_PI / 180, moduleStates[2].angle, tolerance);
    ASSERT_NEAR(1.5811388300841898, moduleStates[2].velocity, tolerance);

    ASSERT_NEAR(7 * M_PI / 4, moduleStates[3].angle, tolerance);
    ASSERT_NEAR(1 / std::sqrt(2), moduleStates[3].velocity, tolerance);
}

// 5) check if the optimizing function is calculating and minimizing the needed rotation by reversing the direction of rotation when needed,
TEST(swerve_drive_core, OptimizeRotationByReversingDirectionWhenMinimizingRotation)
{
    // Creating chasis speed object;
    ChassisSpeeds speeds(01.0, 01.0, 0.0);

    // get moduleStates
    auto moduleStates = testKinematics.toSwerveModuleStates(speeds, Translation2D(0.0, 0.0));

    // Create a vector or array to store the optimized module states
    std::vector<SwerveModuleState> optimizedModuleStates;

    optimizedModuleStates.push_back(SwerveModuleState::optimize(moduleStates[0], 0));
    optimizedModuleStates.push_back(SwerveModuleState::optimize(moduleStates[1], 6*M_PI/4));
    optimizedModuleStates.push_back(SwerveModuleState::optimize(moduleStates[2], 1*M_PI/4));
    optimizedModuleStates.push_back(SwerveModuleState::optimize(moduleStates[3], 3*M_PI/4));

    ASSERT_NEAR(7 * M_PI/4, optimizedModuleStates[0].angle, tolerance);
    ASSERT_EQ(-std::sqrt(2), optimizedModuleStates[0].velocity);

    ASSERT_NEAR(7 * M_PI/4, optimizedModuleStates[1].angle, tolerance);
    ASSERT_EQ(-std::sqrt(2), optimizedModuleStates[1].velocity);

    ASSERT_NEAR(3 * M_PI/4, optimizedModuleStates[2].angle, tolerance);
    ASSERT_EQ(std::sqrt(2), optimizedModuleStates[2].velocity);

    ASSERT_NEAR(3 * M_PI/4, optimizedModuleStates[3].angle, tolerance);
    ASSERT_EQ(std::sqrt(2), optimizedModuleStates[3].velocity);
}

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}