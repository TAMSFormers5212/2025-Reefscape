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
    double speedMultiplier=.7;
    double XAxis;
    double YAxis;

    SwerveDrive m_drive;
    Superstructure m_superstructure;

    frc2::CommandPtr m_testAuto1 = PathPlannerAuto("Test Auto 1").ToPtr();

    frc2::Command* GetAutonomousCommand();
    void Periodic();

   private:
    frc::GenericHID m_driverController{kDriverControllerPort};
    frc::GenericHID m_operatorController{kOperatorControllerPort};
    // Replace with CommandPS4Controller or CommandJoystick if needed
    // frc2::CommandXboxController m_driverController{
    //     OperatorConstants::kDriverControllerPort};

    frc2::CommandPtr m_rotationTest = PathPlannerAuto("Rotation Testing").ToPtr();
    frc2::CommandPtr m_mobility =
        PathPlannerAuto("Mobility Auton").ToPtr();
    frc2::CommandPtr m_oneCoral =
        PathPlannerAuto("1 Coral Auton").ToPtr();
    frc::SendableChooser<frc2::Command*> m_chooser;

    bool runAlign = false;

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
