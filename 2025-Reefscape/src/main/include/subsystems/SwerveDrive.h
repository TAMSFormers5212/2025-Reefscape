#pragma once

#include <frc2/command/CommandPtr.h>

#include <frc/estimator/SwerveDrivePoseEstimator.h>
#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/geometry/Translation2d.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/kinematics/SwerveDriveOdometry.h>
#include <frc/kinematics/SwerveModulePosition.h>
#include <frc/kinematics/SwerveModuleState.h>
#include <pathplanner/lib/auto/AutoBuilder.h>
#include <pathplanner/lib/auto/AutoBuilder.h>
#include <pathplanner/lib/config/RobotConfig.h>
#include <pathplanner/lib/controllers/PPHolonomicDriveController.h>
#include <frc/geometry/Pose2d.h>
#include <frc/kinematics/ChassisSpeeds.h>
#include <frc/DriverStation.h>
#include <frc/controller/PIDController.h>

#include "SwerveModule.h"
#include <studica/AHRS.h>

using namespace SwerveModuleConstants;
using namespace std;


class SwerveDrive : public frc2::SubsystemBase{

public:
    studica::AHRS m_gyro{studica::AHRS::NavXComType::kUSB1};
    SwerveDrive();

    frc::Pose2d AveragePose();
    frc::Pose2d AveragePose(frc::Pose2d visionPose);
    frc::Pose2d OdometryPose();
    frc::Rotation2d getGyroHeading2();
    frc::ChassisSpeeds getRobotRelativeSpeeds();
    frc::ChassisSpeeds getFieldRelativeSpeeds();

    void resetHeading();
    void setHeading(int x);
    void resetOdometry(const frc::Pose2d pose);
    void swerveDrive(double x, double y, double theta, bool fieldCentric);
    void swerveDrive(frc::ChassisSpeeds speed);
    void brake();
    void toggleOffset(bool offset);
    bool getOffsetToggle();
    void toggleOffset();
    
    void tankDrive(double x, double y);
    void moveToAngle(double x, double y);
    void resetAbsoluteEncoders();
    void SyncAbsoluteEncoders();
    void SetAlign(bool a);
    void UpdatePoseEstimate();

    void Periodic() override; //< update pose using gyro, vision, and odometry

private:
    array<SwerveModule, 4> m_modules;

    frc::SwerveDriveKinematics<4> m_driveKinematics;
    frc::SwerveDrivePoseEstimator<4> m_odometry;

    frc::PIDController thetaController; // closed loop control for heading
    // may be something we want to implement if we notice the drive slowly twisting as it drives
    
    bool offsetToggle = true;
    bool align = true;

    frc::Rotation2d heading;

    double lastAngle;
};