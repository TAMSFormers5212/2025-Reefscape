#include "commands/AlignToReef.h"

#include <frc2/command/Command.h>
#include <frc2/command/RunCommand.h>
#include <frc2/command/Subsystem.h>
#include <pathplanner/lib/auto/AutoBuilder.h>
#include <pathplanner/lib/auto/NamedCommands.h>
#include <pathplanner/lib/commands/PathPlannerAuto.h>
#include <pathplanner/lib/path/GoalEndState.h>
#include <pathplanner/lib/path/PathConstraints.h>
#include <pathplanner/lib/path/PathPlannerPath.h>
#include <pathplanner/lib/path/Waypoint.h>
#include <iostream>
#include <functional>

#include "subsystems/SwerveDrive.h"
#include <frc2/command/SubsystemBase.h>
#include <frc/smartdashboard/SmartDashboard.h>

using namespace pathplanner;

AlignToReef::AlignToReef(SwerveDrive* swerve) : m_swerve(swerve) {
    AddRequirements(swerve);
}

frc::Rotation2d AlignToReef::getVelocityHeading() {
    frc::ChassisSpeeds speeds = m_swerve->getFieldRelativeSpeeds();
    return frc::Rotation2d(speeds.vx.value(), speeds.vy.value());
}

frc::Pose2d AlignToReef::getTargetPose() {
    return frc::Pose2d(
        m_swerve->OdometryPose().Translation().X() + units::meter_t{1},
        m_swerve->OdometryPose().Translation().Y(),
        m_swerve->OdometryPose().Rotation().Degrees());
}

frc2::CommandPtr AlignToReef::driveToTargetPose(frc::Pose2d waypoint) {
            std::cout <<"Crazy? I was Crazy Once, they locked me in a room, a rubber room full of rats, and rats make me crayz" <<endl;

    frc::ChassisSpeeds speeds = m_swerve->getFieldRelativeSpeeds();
    std::vector<frc::Pose2d> poses{
        frc::Pose2d(m_swerve->OdometryPose().Translation(),
                    getVelocityHeading()),
        waypoint};

    std::vector<Waypoint> waypoints =
        PathPlannerPath::waypointsFromPoses(poses);

    PathConstraints constraints(3.0_mps, 3.0_mps_sq, 360_deg_per_s,
                                720_deg_per_s_sq);
    // auto translation = frc::Translation2d(speeds.vx, speeds.vy).Norm();
    // units::meters_per_second_t thing{
    //     frc::Translation2d(speeds.vx, speeds.vy).Norm().value()};
    auto path = std::make_shared<PathPlannerPath>(
        waypoints, constraints,
        IdealStartingState(0.0_mps, m_swerve->getGyroHeading2()),
        GoalEndState(0.0_mps,
                     waypoint.Rotation())  // Goal end state. You can set a
                                           // holonomic rotation here. If using
                                           // a differential drivetrain, the
                                           // rotation will have no effect.
    );
    path->preventFlipping = true;

    // return AutoBuilder::followPath(path).AndThen([this] { alignAdjustment(); });
    
    return AutoBuilder::followPath(path);
}

void AlignToReef::alignAdjustment() {
    PathPlannerTrajectoryState goalState = PathPlannerTrajectoryState();
    frc::Pose2d goalPose = getTargetPose();
    goalState.pose = goalPose;

    PPHolonomicDriveController ctrler = PPHolonomicDriveController(
        PIDConstants(5.0, 0.0, 0.0), PIDConstants(5.0, 0.0, 0.0));

    m_swerve->swerveDrive(ctrler.calculateRobotRelativeSpeeds(
        m_swerve->OdometryPose(), goalState));
}

frc2::CommandPtr AlignToReef::generateCommand() {
    // std::function<frc2::CommandPtr()> thing = [this] {
    //     return driveToTargetPose(getTargetPose());
    // };
    std::initializer_list<frc2::Subsystem*> requirements = {m_swerve};
   
    return frc2::cmd::Defer([this] {
        return driveToTargetPose(getTargetPose());
    }, requirements).WithInterruptBehavior(Command::InterruptionBehavior::kCancelIncoming);
    // return frc2::cmd::Defer(thing, frc2::Requirements(requirements));
    // return driveToTargetPose(getTargetPose());
}

void AlignToReef::Initialize() {}

void AlignToReef::Execute() {
     frc::SmartDashboard::PutBoolean("Auto Align Ran", true);
}

bool AlignToReef::IsFinished() { return false; }