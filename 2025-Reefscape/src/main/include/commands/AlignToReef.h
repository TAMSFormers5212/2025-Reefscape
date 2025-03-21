#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>

#include "subsystems/SwerveDrive.h"
#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Rotation2d.h>

#include <pathplanner/lib/auto/AutoBuilder.h>
#include <pathplanner/lib/auto/NamedCommands.h>
#include <pathplanner/lib/commands/PathPlannerAuto.h>
#include <pathplanner/lib/path/PathPlannerPath.h>
#include <pathplanner/lib/path/Waypoint.h>
#include <pathplanner/lib/path/PathConstraints.h>
#include <pathplanner/lib/path/GoalEndState.h>


class AlignToReef : public frc2::CommandHelper<frc2::Command, AlignToReef> {
   public:
    explicit AlignToReef(SwerveDrive* swerve);


    void Initialize() override;
    void Execute() override;

    void alignAdjustment();
    frc::Pose2d getTargetPose();
    frc2::CommandPtr driveToTargetPose(frc::Pose2d waypoint);
    frc2::CommandPtr generateCommand();
    frc::Rotation2d getVelocityHeading();

    bool IsFinished() override;
    //SwerveDrive* m_swerve;
   private:
    SwerveDrive* m_swerve;
    
    
};
