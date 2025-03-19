// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/GenericHID.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <frc2/command/Command.h>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/button/CommandXboxController.h>
#include <pathplanner/lib/auto/AutoBuilder.h>
#include <pathplanner/lib/auto/NamedCommands.h>
#include <pathplanner/lib/commands/PathPlannerAuto.h>
#include <pathplanner/lib/path/PathPlannerPath.h>

#include "Constants.h"
#include "subsystems/Superstructure.h"
#include "subsystems/SwerveDrive.h"

using namespace OIConstants;
using namespace pathplanner;

/**
 * This class is where the bulk of the robot should be declared.  Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls).  Instead, the structure of the robot (including subsystems,
 * commands, and trigger mappings) should be declared here.
 */
class RobotContainer {
   public:
    RobotContainer();
    double RotAxis;
    double speedMultiplier = 1.0;
    double XAxis;
    double YAxis;

    SwerveDrive m_drive;
    Superstructure m_superstructure;

    frc2::Command* GetAutonomousCommand();
    void Periodic();

    frc::Pose2d autoStartingPose(void);

   private:
    frc::GenericHID m_driverController{kDriverControllerPort};
    frc::GenericHID m_operatorController{kOperatorControllerPort};

    frc2::CommandPtr m_testAuto = PathPlannerAuto("Test Auto").ToPtr();
    frc2::CommandPtr m_mobility = PathPlannerAuto("Mobility Auton").ToPtr();
    frc2::CommandPtr m_L1Center = PathPlannerAuto("L1 Center").ToPtr();
    frc2::CommandPtr m_L1Left = PathPlannerAuto("L1 Left").ToPtr();
    frc2::CommandPtr m_L1Right = PathPlannerAuto("L1 Right").ToPtr();
    frc2::CommandPtr m_L4Center = PathPlannerAuto("L4 Center").ToPtr();
    frc2::CommandPtr m_L4Left = PathPlannerAuto("L4 Left").ToPtr();
    frc2::CommandPtr m_L4Right = PathPlannerAuto("L4 Right").ToPtr();
    frc2::CommandPtr m_L1CenterL4Left = PathPlannerAuto("L1 Center - Left L4 [UNFINISHED]").ToPtr();
    frc2::CommandPtr m_L1CenterL4Right = PathPlannerAuto("L1 Center - Right L4 [UNFINISHED]").ToPtr();
    frc2::CommandPtr m_L1LeftL4Left = PathPlannerAuto("L1 Left - Left L4 [UNFINISHED]").ToPtr();
    frc2::CommandPtr m_L1RightL4Right = PathPlannerAuto("L1 Right - Right L4 [UNFINISHED]").ToPtr();
    frc::SendableChooser<frc2::Command*> m_chooser;// = ("Mobility Auton"); //m_chooser;

    bool runAlign = false;
    bool override_ = false;
    bool autoIntake = false;

    bool prevDown = false;
    bool prevUp = false;
    bool prevLeft = false;
    bool prevRight = false;

    bool opPrevDown = false;
    bool opPrevUp = false;
    bool opPrevLeft = false;
    bool opPrevRight = false;

    void ConfigureBindings();
};
