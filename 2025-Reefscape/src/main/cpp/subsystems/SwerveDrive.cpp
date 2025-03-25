#include "subsystems/SwerveDrive.h"

#include <frc/DriverStation.h>
#include <frc/RobotBase.h>
#include <frc/Timer.h>
#include <frc/geometry/Pose2d.h>
#include <frc/kinematics/ChassisSpeeds.h>
#include <frc/smartdashboard/Field2d.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <math.h>
#include <pathplanner/lib/auto/AutoBuilder.h>
#include <pathplanner/lib/config/RobotConfig.h>
#include <pathplanner/lib/controllers/PPHolonomicDriveController.h>

#include <iostream>
#include <vector>

#include "LimelightHelpers.h"

using namespace SwerveModuleConstants;
using namespace MathConstants;
using namespace units;
using namespace pathplanner;
using frc::SmartDashboard;
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
                 frc::Rotation2d(-getGyroHeading2()),
                 {m_modules[0].getPosition(), m_modules[1].getPosition(),
                  m_modules[2].getPosition(), m_modules[3].getPosition()},
                 frc::Pose2d()},
      thetaController(0, 0, 0) {
    m_gyro.Reset();
    resetAbsoluteEncoders();
    // m_gyro.ZeroYaw();
    heading = frc::Rotation2d(degree_t{-m_gyro.GetYaw()});
    lastAngle = -m_gyro.GetYaw();
    // resetOdometry(m_poseEstimator.GetEstimatedPosition());

    RobotConfig config = RobotConfig::fromGUISettings();

    AutoBuilder::configure(
        [this]() { return OdometryPose(); },  // Robot pose supplier
        [this](frc::Pose2d pose) {
            resetOdometry(pose);
        },  // Method to reset odometry (will be called if your auto has a
            // starting pose)
        [this]() {
            return getRobotRelativeSpeeds();
        },  // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
        [this](frc::ChassisSpeeds speeds) {
            swerveDrive(speeds);
        },  // Method that will drive the robot given ROBOT RELATIVE
            // ChassisSpeeds
        std::make_shared<
            PPHolonomicDriveController>(  // PPHolonomicController is the built
                                          // in path following controller for
                                          // holonomic drive trains
            PIDConstants(5.0, 0.0, 0.0),  // Translation PID constants
            PIDConstants(5.0, 0.0, 0.0)   // Rotation PID constants
            ),
        config,
        []() {
            // Boolean supplier that controls when the path will be mirrored for
            // the red alliance This will flip the path being followed to the
            // red side of the field. THE ORIGIN WILL REMAIN ON THE BLUE SIDE
            auto alliance = frc::DriverStation::GetAlliance();
            if (alliance) {
                return alliance.value() == frc::DriverStation::Alliance::kRed;
            }
            return false;
        },
        this  // Reference to this subsystem to set requirements
    );
    std::cout << "Swerve subsystem initalized correctly" << std::endl;
}

// frc::Pose2d SwerveDrive::AveragePose() {  // returns the pose estimator
// position
// // m_poseEstimator.AddVisionMeasurement();
//     return m_poseEstimator.GetEstimatedPosition();

// }

// frc::Pose2d SwerveDrive::AveragePose(frc::Pose2d visionPose) {  // returns
// the pose estimator position with vision correction
//     m_poseEstimator.AddVisionMeasurement(visionPose,
//     frc::Timer::GetFPGATimestamp()); return
//     m_poseEstimator.GetEstimatedPosition();
// }

frc::Pose2d SwerveDrive::OdometryPose() {
    return m_odometry.GetEstimatedPosition();
    // retun m_odometry.GetPose();
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
    // resetAbsoluteEncoders();
    frc::SmartDashboard::PutNumber("odometry reset x",
                                   pose.Translation().X().value());
    frc::SmartDashboard::PutNumber("odometry reset y",
                                   pose.Translation().Y().value());
    frc::SmartDashboard::PutNumber("odometry reset rot",
                                   pose.Rotation().Degrees().value());
    resetHeading();
    m_odometry.ResetPosition(
        getGyroHeading2(),
        {m_modules[0].getPosition(), m_modules[1].getPosition(),
         m_modules[2].getPosition(), m_modules[3].getPosition()},
        pose);
    m_odometry.ResetPosition(
        getGyroHeading2(),
        {m_modules[0].getPosition(), m_modules[1].getPosition(),
         m_modules[2].getPosition(), m_modules[3].getPosition()},
        pose);
}

void SwerveDrive::swerveDrive(double x, double y, double theta,
                              bool fieldCentric) {  // swerve drive
    frc::ChassisSpeeds speeds =
        fieldCentric
            ? frc::ChassisSpeeds::FromFieldRelativeSpeeds(
                  x * SwerveModuleConstants::maxSpeed,
                  y * SwerveModuleConstants::maxSpeed,
                  theta * SwerveModuleConstants::maxRotation,
                  units::degree_t{-m_gyro.GetYaw() -
                                  90})  // kinda funny that this doesn't use the
                                        // getgyro method made earlier
            : frc::ChassisSpeeds{x * SwerveModuleConstants::maxSpeed,
                                 y * SwerveModuleConstants::maxSpeed,
                                 theta * SwerveModuleConstants::maxRotation};
    speeds = speeds.Discretize(
        speeds.vx, speeds.vy, speeds.omega,
        units::second_t(0.02));  // second order kinematics?!?! nani

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
    // speeds = speeds.Discretize(speeds.vx, speeds.vy, speeds.omega,
    // units::second_t(0.02));  // second order kinematics?!?! nani
    speeds = speeds.Discretize(speeds.vx, speeds.vy, speeds.omega,
                               units::second_t(0.02));
    auto saturatedStates = m_driveKinematics.ToSwerveModuleStates(speeds);

    // // maybe desaturate wheel speeds here
    // m_driveKinematics.DesaturateWheelSpeeds(&saturatedStates, maxSpeed);
    // 1. figure out the max speed modules can go
    // 2. figure out the max speed the modules are actually going
    // speeds = speeds.Discretize(speeds.vx , speeds.vy,
    // units::radians_per_second_t{0}, units::second_t(0.02));

    // saturatedStates = m_driveKinematics.ToSwerveModuleStates(speeds);
    m_driveKinematics.DesaturateWheelSpeeds(&saturatedStates, maxSpeed);
    // second order kinematics?!?!
    // frc::SmartDashboard::PutNumber("speeds omega", speeds.omega.value());
    auto states = m_driveKinematics.ToSwerveModuleStates(speeds);

    for (size_t i = 0; i < states.size(); ++i) {
        m_modules[i].setState(states[i]);
    }
}

void SwerveDrive::brake() {  // sets wheels to o position
    swerveDrive(0, 0, 0.05, false);
}

frc::ChassisSpeeds SwerveDrive::getRobotRelativeSpeeds() {
    return m_driveKinematics.ToChassisSpeeds(
        {m_modules[0].getState(), m_modules[1].getState(),
         m_modules[2].getState(), m_modules[3].getState()});
}

frc::ChassisSpeeds SwerveDrive::getFieldRelativeSpeeds() {
    // taken from yagsl getfieldvelocity() function
    return frc::ChassisSpeeds::FromFieldRelativeSpeeds(
        m_driveKinematics.ToChassisSpeeds(
            {m_modules[0].getState(), m_modules[1].getState(),
             m_modules[2].getState(), m_modules[3].getState()}),
        getGyroHeading2());
}

void SwerveDrive::moveToAngle(
    double x, double y) {  // basically crab drive, points all wheels in the
                           // same direction ROBOT CENTRIC
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
    // testing movetoangle function with these values:
    //  frc::SmartDashboard::PutNumber("Magnitude", r);
    //  frc::SmartDashboard::PutNumber("angle", angle);
    //  frc::SmartDashboard::PutNumber("xm", x);
    //  frc::SmartDashboard::PutNumber("ym", y);
    for (auto &module : m_modules) {
        module.setState(frc::SwerveModuleState{
            meters_per_second_t(r), frc::Rotation2d(radian_t(angle))});
    }
    // frc::SmartDashboard::PutNumber("x", x);
    // frc::SmartDashboard::PutNumber("y", y);
    // frc::SmartDashboard::PutNumber("")
}

void SwerveDrive::tankDrive(double x,
                            double y) {  // untested tank drive configuration
                                         // implement differential drive later
    m_modules[0].setState(frc::SwerveModuleState{
        meters_per_second_t(x + y), frc::Rotation2d(radian_t(0))});  // tl
    m_modules[1].setState(frc::SwerveModuleState{
        meters_per_second_t(x - y), frc::Rotation2d(radian_t(0))});  // tr
    m_modules[2].setState(frc::SwerveModuleState{
        meters_per_second_t(x + y), frc::Rotation2d(radian_t(0))});  // bl
    m_modules[3].setState(frc::SwerveModuleState{
        meters_per_second_t(x - y), frc::Rotation2d(radian_t(0))});  // br
}

void SwerveDrive::SetAlign(bool a) { align = a; }

void SwerveDrive::UpdatePoseEstimate() {
    m_odometry.Update(getGyroHeading2(),
                      {m_modules[0].getPosition(), m_modules[1].getPosition(),
                       m_modules[2].getPosition(), m_modules[3].getPosition()});
    if (align) {
        bool doRejectUpdate = false;

        LimelightHelpers::PoseEstimate mt2 =
            LimelightHelpers::getBotPoseEstimate_wpiBlue_MegaTag2("limelight");

        auto pose = mt2.pose;

        frc::SmartDashboard::PutNumber("limelight x", pose.X().value());
        frc::SmartDashboard::PutNumber("limelight y", pose.Y().value());
        frc::SmartDashboard::PutNumber("limelight h",
                                       pose.Rotation().Degrees().value());

        if (abs(m_gyro.GetRate()) >
            720)  // if our angular velocity is greater than 720 degrees per
                  // second, ignore vision updates
        {
            doRejectUpdate = true;
        }
        if (mt2.tagCount == 0) {
            doRejectUpdate = true;
        }
        if (!doRejectUpdate) {
            wpi::array<double, 3U> temp = {.7, .7, 9999999};
            // m_poseEstimator.SetVisionMeasurementStdDevs(VecBuilder.fill(.7,.7,9999999));
            m_odometry.SetVisionMeasurementStdDevs(temp);
            m_odometry.AddVisionMeasurement(mt2.pose, mt2.timestampSeconds);
        }
    }
}

void SwerveDrive::initAuto(void) {
    auto alliance = frc::DriverStation::GetAlliance();
    bool isAuto = frc::DriverStation::IsAutonomous();

    if (alliance.value() == frc::DriverStation::Alliance::kRed && isAuto) {
        LimelightHelpers::SetRobotOrientation(
            "limelight", getGyroHeading2().Degrees().value() + 0, 0, 0, 0, 0,
            0);
    } else if (alliance.value() == frc::DriverStation::Alliance::kRed &&
               !isAuto) {
        LimelightHelpers::SetRobotOrientation(
            "limelight", getGyroHeading2().Degrees().value() + 180, 0, 0, 0, 0,
            0);
    } else if (alliance.value() == frc::DriverStation::Alliance::kBlue &&
               isAuto) {
        LimelightHelpers::SetRobotOrientation(
            "limelight", getGyroHeading2().Degrees().value() + 180, 0, 0, 0, 0,
            0);
    } else {
        LimelightHelpers::SetRobotOrientation(
            "limelight", getGyroHeading2().Degrees().value(), 0, 0, 0, 0, 0);
    }
    UpdatePoseEstimate();
}

void SwerveDrive::Periodic() {
    auto alliance = frc::DriverStation::GetAlliance();
    bool isAuto = frc::DriverStation::IsAutonomous();

    SmartDashboard::PutBoolean("isAuto", isAuto);

    // if(sqrt(getRobotRelativeSpeeds().vx.value()*getRobotRelativeSpeeds().vx.value()+getRobotRelativeSpeeds().vy.value()*getRobotRelativeSpeeds().vy.value())<=VisionConstants::stableSpeed){
    //     //if robot is moving slow enough, add vision pose to estimator
    // }
    frc::SmartDashboard::PutNumber("gyro angle2",
                                   getGyroHeading2().Degrees().value());

    auto pose = m_odometry.GetEstimatedPosition();

    frc::SmartDashboard::PutNumber("odometry x",
                                   pose.Translation().X().value());
    frc::SmartDashboard::PutNumber("odometry y",
                                   pose.Translation().Y().value());
    frc::SmartDashboard::PutNumber("odometry rot",
                                   pose.Rotation().Degrees().value());

    frc::SmartDashboard::PutNumber(
        "wheel pos",
        m_modules[0].getDrivePosition() + drivebase::WheelBase.value());
    frc::SmartDashboard::PutNumber("Gyro", m_gyro.GetAngle());

    frc::Field2d m_field;
    m_field.SetRobotPose(pose);
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

void SwerveDrive::toggleOffset() {  // switches the offsetToggle to true or
                                    // false
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

frc::Rotation2d SwerveDrive::getVelocityHeading() {
    frc::ChassisSpeeds speeds = getFieldRelativeSpeeds();
    return frc::Rotation2d(speeds.vx.value(), speeds.vy.value());
}

frc::Pose2d SwerveDrive::getTargetPose(bool left) {
    std::shared_ptr<nt::NetworkTable> table =
        nt::NetworkTableInstance::GetDefault().GetTable("limelight");
    double id = table->GetNumber("tid", 0.0);
    if (id == 18) {
        if (left) {
            return frc::Pose2d(units::meter_t{3.1}, units::meter_t{4.187},
                               units::degree_t{0});
        } else {
            return frc::Pose2d(units::meter_t{3.081}, units::meter_t{3.86},
                               units::degree_t{0});
        }
    }
    if (id == 19) {
        if (!left) {
            return frc::Pose2d(units::meter_t{3.668}, units::meter_t{5.122},
                               units::degree_t{-60});
        } else {
            return frc::Pose2d(units::meter_t{3.932}, units::meter_t{5.314},
                               units::degree_t{-60});
        }
    }
    if (id == 20) {
        if (!left) {
            return frc::Pose2d(units::meter_t{5.059}, units::meter_t{5.326},
                               units::degree_t{-120});
        } else {
            return frc::Pose2d(units::meter_t{5.323}, units::meter_t{5.158},
                               units::degree_t{-120});
        }
    }
    if (id == 21) {
        if (!left) {
            return frc::Pose2d(units::meter_t{5.862}, units::meter_t{4.187},
                               units::degree_t{180});
        } else {
            return frc::Pose2d(units::meter_t{5.886}, units::meter_t{3.86},
                               units::degree_t{180});
        }
    }
    if (id == 22) {
        if (!left) {
            return frc::Pose2d(units::meter_t{5.323}, units::meter_t{2.904},
                               units::degree_t{120});
        } else {
            return frc::Pose2d(units::meter_t{5.035}, units::meter_t{2.748},
                               units::degree_t{120});
        }
    }
    if (id == 17) {
        if (!left) {
            return frc::Pose2d(units::meter_t{3.944}, units::meter_t{2.748},
                               units::degree_t{60});
        } else {
            return frc::Pose2d(units::meter_t{3.656}, units::meter_t{2.916},
                               units::degree_t{60});
        }
    }
    if (id == 10) {
        if (left) {
            return frc::Pose2d(units::meter_t{11.700}, units::meter_t{4.187},
                               units::degree_t{0});
        } else {
            return frc::Pose2d(units::meter_t{11.676}, units::meter_t{3.86},
                               units::degree_t{0});
        }
    }
    if (id == 9) {
        if (!left) {
            return frc::Pose2d(units::meter_t{12.239}, units::meter_t{5.134},
                               units::degree_t{-60});
        } else {
            return frc::Pose2d(units::meter_t{12.527}, units::meter_t{5.290},
                               units::degree_t{-60});
        }
    }
    if (id == 8) {
        if (left) {
            return frc::Pose2d(units::meter_t{13.618}, units::meter_t{5.302},
                               units::degree_t{-120});
        } else {
            return frc::Pose2d(units::meter_t{13.906}, units::meter_t{5.146},
                               units::degree_t{-120});
        }
    }
    if (id == 7) {
        if (!left) {
            return frc::Pose2d(units::meter_t{14.445}, units::meter_t{4.187},
                               units::degree_t{180});
        } else {
            return frc::Pose2d(units::meter_t{14.409}, units::meter_t{3.86},
                               units::degree_t{180});
        }
    }
    if (id == 6) {
        if (!left) {
            return frc::Pose2d(units::meter_t{13.906}, units::meter_t{2.916},
                               units::degree_t{120});
        } else {
            return frc::Pose2d(units::meter_t{13.606}, units::meter_t{2.748},
                               units::degree_t{120});
        }
    }
    if (id == 11) {
        if (!left) {
            return frc::Pose2d(units::meter_t{12.527}, units::meter_t{2.772},
                               units::degree_t{60});
        } else {
            return frc::Pose2d(units::meter_t{12.239}, units::meter_t{2.928},
                               units::degree_t{60});
        }
    }
    return frc::Pose2d(OdometryPose().Translation().X(),
                       OdometryPose().Translation().Y(),
                       OdometryPose().Rotation().Degrees());
}

frc2::CommandPtr SwerveDrive::driveToTargetPose(frc::Pose2d waypoint,
                                                bool left) {
    frc::ChassisSpeeds speeds = getFieldRelativeSpeeds();

    std::vector<frc::Pose2d> poses{
        frc::Pose2d(OdometryPose().Translation(), getVelocityHeading()),
        waypoint};

    std::vector<Waypoint> waypoints =
        PathPlannerPath::waypointsFromPoses(poses);

    PathConstraints constraints(3.0_mps, 3.0_mps_sq, 360_deg_per_s,
                                720_deg_per_s_sq);
    // auto translation = frc::Translation2d(speeds.vx, speeds.vy).Norm();
    // units::meters_per_second_t thing{
    //     frc::Translation2d(speeds.vx, speeds.vy).Norm().value()};
    auto path = std::make_shared<PathPlannerPath>(
        waypoints, constraints, IdealStartingState(0.0_mps, getGyroHeading2()),
        GoalEndState(0.0_mps,
                     waypoint.Rotation())  // Goal end state. You can set a
                                           // holonomic rotation here. If using
                                           // a differential drivetrain, the
                                           // rotation will have no effect.
    );
    path->preventFlipping = true;

    // return AutoBuilder::followPath(path).AndThen([this] { alignAdjustment();
    // });
    return AutoBuilder::followPath(path);
}

void SwerveDrive::alignAdjustment(bool left) {
    PathPlannerTrajectoryState goalState = PathPlannerTrajectoryState();
    frc::Pose2d goalPose = getTargetPose(left);
    goalState.pose = goalPose;

    PPHolonomicDriveController ctrler = PPHolonomicDriveController(
        PIDConstants(5.0, 0.0, 0.0), PIDConstants(5.0, 0.0, 0.0));

    swerveDrive(ctrler.calculateRobotRelativeSpeeds(OdometryPose(), goalState));
}

frc2::CommandPtr SwerveDrive::generateCommandLeft() {
    std::initializer_list<frc2::Subsystem *> requirements = {this};
    return frc2::cmd::Defer(
        [this] { return driveToTargetPose(getTargetPose(true), true); },
        requirements);
    // return frc2::cmd::Defer(thing, frc2::Requirements(requirements));
    // return driveToTargetPose(getTargetPose());
}

frc2::CommandPtr SwerveDrive::generateCommandRight() {
    std::initializer_list<frc2::Subsystem *> requirements = {this};
    return frc2::cmd::Defer(
        [this] { return driveToTargetPose(getTargetPose(false), false); },
        requirements);
    // return frc2::cmd::Defer(thing, frc2::Requirements(requirements));
    // return driveToTargetPose(getTargetPose());
}
