#include "subsystems/SwerveDrive.h"

#include <frc/kinematics/ChassisSpeeds.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <pathplanner/lib/auto/AutoBuilder.h>
#include <math.h>

#include <pathplanner/lib/auto/AutoBuilder.h>
#include <pathplanner/lib/config/RobotConfig.h>
#include <pathplanner/lib/controllers/PPHolonomicDriveController.h>


#include <frc/geometry/Pose2d.h>
#include <frc/kinematics/ChassisSpeeds.h>
#include <frc/DriverStation.h>
#include <frc/smartdashboard/Field2d.h>
#include "LimelightHelpers.h"
#include <iostream>
#include <frc/Timer.h>
#include <vector>

using namespace SwerveModuleConstants;
using namespace MathConstants;
using namespace units;
using namespace pathplanner;
// using namespace LimelightHelpers;

SwerveDrive::SwerveDrive()
    : m_modules{{SwerveModule(topleft::driveMotor, topleft::steerMotor,
                              topleft::absencoder, topleft::offset),
                 SwerveModule(topright::driveMotor, topright::steerMotor,
                              topright::absencoder, topright::offset),
                 SwerveModule(bottomleft::driveMotor, bottomleft::steerMotor,
                              bottomleft::absencoder, bottomleft::offset),
                 SwerveModule(bottomright::driveMotor, bottomright::steerMotor,
                              bottomright::absencoder, bottomright::offset)}},
      //+-, --, ++, -+
      m_driveKinematics{{frc::Translation2d{drivebase::WheelBase / 2,
                                            drivebase::TrackWidth / 2},
                         frc::Translation2d{drivebase::WheelBase / 2,
                                            -drivebase::TrackWidth / 2},
                         frc::Translation2d{-drivebase::WheelBase / 2,
                                            drivebase::TrackWidth / 2},
                         frc::Translation2d{-drivebase::WheelBase / 2,
                                            -drivebase::TrackWidth / 2}}},
      m_odometry{m_driveKinematics,
                 frc::Rotation2d(-getGyroHeading()),
                 {m_modules[0].getPosition(), m_modules[1].getPosition(),
                  m_modules[2].getPosition(), m_modules[3].getPosition()},
                 frc::Pose2d()},
      m_poseEstimator{m_driveKinematics,
                      frc::Rotation2d(-getGyroHeading()),
                      {m_modules[0].getPosition(), m_modules[1].getPosition(),
                       m_modules[2].getPosition(), m_modules[3].getPosition()},
                      frc::Pose2d()},
      thetaController(0, 0, 0) {
    m_gyro.Reset();
    
    // m_gyro.ZeroYaw();
    heading = frc::Rotation2d(degree_t{-m_gyro.GetYaw()});
    lastAngle = -m_gyro.GetYaw();
    // resetOdometry(m_poseEstimator.GetEstimatedPosition());

    RobotConfig config = RobotConfig::fromGUISettings();

    AutoBuilder::configure(
        [this](){ return OdometryPose(); }, // Robot pose supplier
        [this](frc::Pose2d pose){ resetOdometry(pose); }, // Method to reset odometry (will be called if your auto has a starting pose)
        [this](){ return getRobotRelativeSpeeds(); }, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
        [this](frc::ChassisSpeeds speeds){ swerveDrive(speeds); }, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
        std::make_shared<PPHolonomicDriveController>( // PPHolonomicController is the built in path following controller for holonomic drive trains
            PIDConstants(0.7, 0.0, 0.0001), // Translation PID constants
            PIDConstants(0.7, 0.0, 0.0) // Rotation PID constants
        ),
        config,
        []() {
            // Boolean supplier that controls when the path will be mirrored for the red alliance
            // This will flip the path being followed to the red side of the field.
            // THE ORIGIN WILL REMAIN ON THE BLUE SIDE
            auto alliance = frc::DriverStation::GetAlliance();
            if (alliance) {
                return alliance.value() == frc::DriverStation::Alliance::kRed;
            }
            return false;
        },
        this // Reference to this subsystem to set requirements
    );
    std::cout << "Swerve subsystem initalized correctly" << std::endl;
}

// frc::Pose2d SwerveDrive::AveragePose() {  // returns the pose estimator position
// // m_poseEstimator.AddVisionMeasurement();
//     return m_poseEstimator.GetEstimatedPosition();
    
// }

// frc::Pose2d SwerveDrive::AveragePose(frc::Pose2d visionPose) {  // returns the pose estimator position with vision correction
//     m_poseEstimator.AddVisionMeasurement(visionPose, frc::Timer::GetFPGATimestamp());
//     return m_poseEstimator.GetEstimatedPosition();
// }

frc::Pose2d SwerveDrive::OdometryPose() {
    //frc::SmartDashboard::PutNumber("odometry pose", val.Degrees().value());
    return m_odometry.GetEstimatedPosition();
    //retun m_odometry.GetPose();
}

frc::Rotation2d SwerveDrive::getGyroHeading() {  // i have no f*cking clue how this works but it returns the gyro heading
    double newAngle = -m_gyro.GetYaw();
    
    double delta = std::fmod(std::fmod((newAngle - lastAngle + 180), 360) + 360, 360) - 180;  // NOLINT
    lastAngle = newAngle;
    heading = heading + frc::Rotation2d(degree_t{delta * 1.02466666667});
    
    return heading;
}

frc::Rotation2d SwerveDrive::getGyroHeading2() {
    return frc::Rotation2d(degree_t(-fmod(m_gyro.GetYaw(), 360) + 0));
}

void SwerveDrive::resetHeading() {  // zeros the gyro
    m_gyro.Reset();
}

void SwerveDrive::setHeading(int x) {  // zeros the gyro to the given position
    m_gyro.SetAngleAdjustment(x);
}

void SwerveDrive::resetOdometry(const frc::Pose2d pose) {
    // resetHeading();
    frc::SmartDashboard::PutNumber("odometry reset x", pose.Translation().X().value());
    frc::SmartDashboard::PutNumber("odometry reset y", pose.Translation().Y().value());
    frc::SmartDashboard::PutNumber("odometry reset rot", pose.Rotation().Degrees().value());
    // resetAbsoluteEncoders();
    resetHeading();
    m_odometry.ResetPosition(getGyroHeading(), {m_modules[0].getPosition(), m_modules[1].getPosition(), m_modules[2].getPosition(), m_modules[3].getPosition()}, pose);
    m_poseEstimator.ResetPosition(getGyroHeading(), {m_modules[0].getPosition(), m_modules[1].getPosition(), m_modules[2].getPosition(), m_modules[3].getPosition()}, pose);
}

void SwerveDrive::swerveDrive(double x, double y, double theta, bool fieldCentric) {  // swerve drive
    frc::ChassisSpeeds speeds = fieldCentric ? frc::ChassisSpeeds::FromFieldRelativeSpeeds(
                                                   x * SwerveModuleConstants::maxSpeed,
                                                   y * SwerveModuleConstants::maxSpeed,
                                                   theta * SwerveModuleConstants::maxRotation,
                                                   units::degree_t{-m_gyro.GetYaw() - 90}) // kinda funny that this doesn't use the getgyro method made earlier
                                             : frc::ChassisSpeeds{
                                                   x * SwerveModuleConstants::maxSpeed,
                                                   y * SwerveModuleConstants::maxSpeed,
                                                   theta * SwerveModuleConstants::maxRotation};
    speeds = speeds.Discretize(speeds.vx, speeds.vy, speeds.omega, units::second_t(0.02));  // second order kinematics?!?! nani

    auto saturatedStates = m_driveKinematics.ToSwerveModuleStates(speeds);

    // // maybe desaturate wheel speeds here
    m_driveKinematics.DesaturateWheelSpeeds(&saturatedStates, maxSpeed);
    // 1. figure out the max speed modules can go
    //  2. figure out the max speed the modules are actually going

    auto states = m_driveKinematics.ToSwerveModuleStates(speeds);

    for (size_t i = 0; i < states.size(); ++i) {
        m_modules[i].setState(states[i]);
    }
}

void SwerveDrive::swerveDrive(frc::ChassisSpeeds speeds) {  // swerve drive
    //speeds = speeds.Discretize(speeds.vx, speeds.vy, speeds.omega, units::second_t(0.02));  // second order kinematics?!?! nani
    speeds = speeds.Discretize(speeds.vx , speeds.vy, speeds.omega, units::second_t(0.02));
    auto saturatedStates = m_driveKinematics.ToSwerveModuleStates(speeds);

    // // maybe desaturate wheel speeds here
    // m_driveKinematics.DesaturateWheelSpeeds(&saturatedStates, maxSpeed);
    // 1. figure out the max speed modules can go
    // 2. figure out the max speed the modules are actually going
    //speeds = speeds.Discretize(speeds.vx , speeds.vy, units::radians_per_second_t{0}, units::second_t(0.02));
    
    
    // saturatedStates = m_driveKinematics.ToSwerveModuleStates(speeds);
    m_driveKinematics.DesaturateWheelSpeeds(&saturatedStates, maxSpeed);
      // second order kinematics?!?!            
    //frc::SmartDashboard::PutNumber("speeds omega", speeds.omega.value());
    auto states = m_driveKinematics.ToSwerveModuleStates(speeds);
    
    for (size_t i = 0; i < states.size(); ++i) {
        m_modules[i].setState(states[i]);
    }
}

void SwerveDrive::brake() {  // sets wheels to o position
    swerveDrive(0, 0, 0.05, false);
}

frc::ChassisSpeeds SwerveDrive::getRobotRelativeSpeeds() {
    return m_driveKinematics.ToChassisSpeeds({m_modules[0].getState(), m_modules[1].getState(), m_modules[2].getState(), m_modules[3].getState()});
}

frc::ChassisSpeeds SwerveDrive::getFieldRelativeSpeeds(){
    // taken from yagsl getfieldvelocity() function
    return frc::ChassisSpeeds::FromFieldRelativeSpeeds(m_driveKinematics.ToChassisSpeeds({m_modules[0].getState(), m_modules[1].getState(), m_modules[2].getState(), m_modules[3].getState()}), getGyroHeading2());
}

void SwerveDrive::moveToAngle(double x, double y) {  // basically crab drive, points all wheels in the same direction ROBOT CENTRIC
    double temp = x;
    x = -y;
    y = temp;
    double r = sqrt(pow(x, 2) + pow(y, 2));
    double angle = 0.0;
    if (x == 0 && y == 0) {
        r = 0;
        angle = 0;
    } else {
        if (x > 0 && y >= 0) {
            angle = atan(y / x) + pi / 2;
        } else if (x <= 0 && y > 0) {
            angle = atan(-x / y) + pi;
        } else if (x < 0 && y <= 0) {
            angle = atan(-y / -x) + 3 * pi / 2;
        } else if (x >= 0 && y < 0) {
            angle = atan(-x / y);
        }
    }
    //testing movetoangle function with these values:
    // frc::SmartDashboard::PutNumber("Magnitude", r);
    // frc::SmartDashboard::PutNumber("angle", angle);
    // frc::SmartDashboard::PutNumber("xm", x);
    // frc::SmartDashboard::PutNumber("ym", y);
    for (auto &module : m_modules) {
        module.setState(frc::SwerveModuleState{meters_per_second_t(r), frc::Rotation2d(radian_t(angle))});
    }
    // frc::SmartDashboard::PutNumber("x", x);
    // frc::SmartDashboard::PutNumber("y", y);
    // frc::SmartDashboard::PutNumber("")
}

void SwerveDrive::tankDrive(double x, double y) {                                                             // untested tank drive configuration implement differential drive later
    m_modules[0].setState(frc::SwerveModuleState{meters_per_second_t(x + y), frc::Rotation2d(radian_t(0))});  // tl
    m_modules[1].setState(frc::SwerveModuleState{meters_per_second_t(x - y), frc::Rotation2d(radian_t(0))});  // tr
    m_modules[2].setState(frc::SwerveModuleState{
        meters_per_second_t(x + y), frc::Rotation2d(radian_t(0))});  // bl
    m_modules[3].setState(frc::SwerveModuleState{
        meters_per_second_t(x - y), frc::Rotation2d(radian_t(0))});  // br
}

void SwerveDrive::SetAlign(bool a) {
    align = a;
}

void SwerveDrive::UpdatePoseEstimate() {
    m_odometry.Update(getGyroHeading2(), {m_modules[0].getPosition(), m_modules[1].getPosition(), m_modules[2].getPosition(), m_modules[3].getPosition()});
    m_poseEstimator.Update(getGyroHeading2(), {m_modules[0].getPosition(), m_modules[1].getPosition(), m_modules[2].getPosition(), m_modules[3].getPosition()});
    if(align == true) { 
        bool doRejectUpdate = false;
        
        LimelightHelpers::SetRobotOrientation("limelight", m_poseEstimator.GetEstimatedPosition().Rotation().Degrees().value(), 0, 0, 0, 0, 0);
        LimelightHelpers::PoseEstimate mt2 = LimelightHelpers::getBotPoseEstimate_wpiBlue_MegaTag2("limelight");
        if(abs(m_gyro.GetRate()) > 720) // if our angular velocity is greater than 720 degrees per second, ignore vision updates
        {
            doRejectUpdate = true;
        }
        if(mt2.tagCount == 0)
        {
            doRejectUpdate = true;
        }
        if(!doRejectUpdate)
        {
            wpi::array<double, 3U> temp = {.7,.7,9999999}; 
            // m_poseEstimator.SetVisionMeasurementStdDevs(VecBuilder.fill(.7,.7,9999999));
             m_poseEstimator.SetVisionMeasurementStdDevs(temp);
            m_poseEstimator.AddVisionMeasurement(
                mt2.pose,
                mt2.timestampSeconds);
        }
    }
}

void SwerveDrive::Periodic() {
    // m_odometry.Update(getGyroHeading2(), {m_modules[0].getPosition(), m_modules[1].getPosition(), m_modules[2].getPosition(), m_modules[3].getPosition()});

    // m_poseEstimator.Update(getGyroHeading2(), {m_modules[0].getPosition(), m_modules[1].getPosition(), m_modules[2].getPosition(), m_modules[3].getPosition()});
    UpdatePoseEstimate();
    // if(sqrt(getRobotRelativeSpeeds().vx.value()*getRobotRelativeSpeeds().vx.value()+getRobotRelativeSpeeds().vy.value()*getRobotRelativeSpeeds().vy.value())<=VisionConstants::stableSpeed){
    //     //if robot is moving slow enough, add vision pose to estimator
    // }
    frc::SmartDashboard::PutNumber("gyro angle", fmod(m_gyro.GetAngle(), 360));
    frc::SmartDashboard::PutNumber("gyro angle2", getGyroHeading2().Degrees().value());

    frc::SmartDashboard::PutNumber("odometry x", m_odometry.GetEstimatedPosition().Translation().X().value());
    frc::SmartDashboard::PutNumber("odometry y", m_odometry.GetEstimatedPosition().Translation().Y().value());
    frc::SmartDashboard::PutNumber("odometry rot", m_odometry.GetEstimatedPosition().Rotation().Degrees().value());
    //frc::SmartDashboard::PutNumber("odometry x", m_odometry.GetPose().Translation().X().value());
    //frc::SmartDashboard::PutNumber("odometry y", m_odometry.GetPose().Translation().Y().value());
    //frc::SmartDashboard::PutNumber("odometry rot", m_odometry.GetPose().Rotation().Degrees().value());

    frc::SmartDashboard::PutNumber("wheel pos", m_modules[0].getDrivePosition()+drivebase::WheelBase.value());
    frc::SmartDashboard::PutNumber("Gyro", m_gyro.GetAngle());
    frc::Field2d m_field;
    frc::SmartDashboard::PutData("robot pos", &m_field);
}   

void SwerveDrive::resetAbsoluteEncoders() {  // resets drive and steer encoders
    for (auto &module : m_modules) {
        module.resetDriveEncoder();
        module.resetSteerEncoder();
    }
}

void SwerveDrive::SyncAbsoluteEncoders() {  // resets steer encoder
    for (auto &module : m_modules) {
        module.resetSteerEncoder();
    }
}

bool SwerveDrive::getOffsetToggle() {  // returns true/false for offsetToggle
    return offsetToggle;
}

void SwerveDrive::toggleOffset() {  // switches the offsetToggle to true or false
    if (offsetToggle) {
        offsetToggle = false;
        for (auto &module : m_modules) {
            module.togglePositionOffset(offsetToggle);
        }
    } else {
        offsetToggle = true;
        for (auto &module : m_modules) {
            module.togglePositionOffset(offsetToggle);
        }
    }
}
