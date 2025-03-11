// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <frc/controller/PIDController.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/Commands.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/ParallelCommandGroup.h>
#include <frc2/command/ParallelRaceGroup.h>
#include <frc2/command/RunCommand.h>
#include <frc2/command/SequentialCommandGroup.h>
#include <frc2/command/StartEndCommand.h>
#include <frc2/command/WaitCommand.h>
#include <frc2/command/button/JoystickButton.h>
#include <frc2/command/button/POVButton.h>
#include <frc2/command/button/Trigger.h>
#include <math.h>
#include <pathplanner/lib/auto/AutoBuilder.h>
#include <pathplanner/lib/auto/NamedCommands.h>
#include <pathplanner/lib/commands/PathPlannerAuto.h>
#include <pathplanner/lib/path/PathPlannerPath.h>

#include <iostream>

#include "commands/Autos.h"
#include "commands/ElevatorL2.h"
#include "commands/ElevatorL3.h"
#include "commands/ElevatorL4.h"
#include "commands/ElevatorSource.h"
#include "commands/ExampleCommand.h"
#include "commands/LiftElevator.h"
#include "commands/OuttakeCmd.h"
#include "commands/StopOuttake.h"
#include "networktables/NetworkTable.h"
#include "networktables/NetworkTableEntry.h"
#include "networktables/NetworkTableInstance.h"
#include "networktables/NetworkTableValue.h"

using namespace pathplanner;
using namespace std;
using namespace frc2;
using namespace OIConstants;

RobotContainer::RobotContainer() {
    NamedCommands::registerCommand(
        "Lift Elevator",
        cmds::LiftElevator(&m_superstructure.m_elevator).ToPtr());
    NamedCommands::registerCommand(
        "Outtake L1", OuttakeCmd(&m_superstructure.m_outtake).ToPtr());
    NamedCommands::registerCommand(
        "Stop Outtake", StopOuttake(&m_superstructure.m_outtake).ToPtr());
    NamedCommands::registerCommand(
        "Elevator L2", ElevatorL2(&m_superstructure.m_elevator).ToPtr());
    NamedCommands::registerCommand(
        "Elevator L3", ElevatorL3(&m_superstructure.m_elevator).ToPtr());
    NamedCommands::registerCommand(
        "Elevator L4", ElevatorL4(&m_superstructure.m_elevator).ToPtr());
    NamedCommands::registerCommand(
        "Elevator Source",
        ElevatorSource(&m_superstructure.m_elevator).ToPtr());

    ConfigureBindings();

    m_rotationTest = PathPlannerAuto("Rotation Testing").ToPtr();
    m_mobility = PathPlannerAuto("Mobility Auton").ToPtr();
    m_oneCoral = PathPlannerAuto("1 Coral Auton").ToPtr();

    m_chooser.SetDefaultOption("1 Coral Auton", m_oneCoral.get());
    // m_chooser.AddOption("1 Coral Auton", m_oneCoral.get());
    // m_chooser.AddOption("Test Auto 1", m_testAu)
    frc::SmartDashboard::PutData(&m_chooser);

    m_drive.SetDefaultCommand(RunCommand(
        [this] {
            if (m_driverController.GetRawButtonPressed(
                    Controller::Y)) {  // zero
                m_drive.resetHeading();
                m_drive.resetAbsoluteEncoders();
            }
            if (m_driverController.GetRawButtonPressed(Controller::X)) {
                m_drive.setHeading(180);
            }

            auto startHeading = m_drive.getGyroHeading2();

            if (m_driverController.GetRawButton(Controller::leftBumper)) {
                speedMultiplier = 0.5;
            } else if (m_driverController.GetRawButton(
                           Controller::rightBumper)) {
                speedMultiplier = 1.0;
            }
            frc::SmartDashboard::PutNumber("speedMultiplier", speedMultiplier);

            XAxis = -m_driverController.GetRawAxis(Controller::leftXAxis) *
                    speedMultiplier;
            YAxis = m_driverController.GetRawAxis(Controller::leftYAxis) *
                    speedMultiplier;
            RotAxis = -m_driverController.GetRawAxis(Controller::rightXAxis) *
                      speedMultiplier * 2;

            if (abs(XAxis) < (Controller::deadband * speedMultiplier)) {
                XAxis = 0;
            }
            if (abs(YAxis) < (Controller::deadband * speedMultiplier)) {
                YAxis = 0;
            }

            frc::SmartDashboard::PutNumber("x", XAxis);
            frc::SmartDashboard::PutNumber("y", YAxis);
            frc::SmartDashboard::PutNumber("rot", RotAxis);

            switch (m_driverController.GetPOV()) {
                case 0:  // up
                    m_drive.swerveDrive(0.0, 0.5, 0.0, false);
                    break;
                case 180:  // down
                    m_drive.swerveDrive(0.0, -0.5, 0.0, false);
                    break;
                case 270:  // left
                    m_drive.swerveDrive(-0.5, 0.0, 0.0, false);
                    break;
                case 90:  // right
                    m_drive.swerveDrive(0.5, 0.0, 0.0, false);
                    break;
                default:
                    m_drive.swerveDrive(XAxis, YAxis, RotAxis, true);
                    break;
            }

            std::shared_ptr<nt::NetworkTable> table =
                nt::NetworkTableInstance::GetDefault().GetTable("limelight");

            frc::SmartDashboard::PutBoolean("runAlign", runAlign);
            if (runAlign) {
                // RotAxis += m_superstructure.m_vision.getOutput() * 0.2;
                // if (m_superstructure.m_vision.isTagPresent()) {
                // if (m_vision.getDistanceError() > 0 &&
                //     m_vision.getDistanceError() < 25) {

                //  YAxis += m_superstructure.m_vision.getDistanceError() *
                //  speedMultiplier;
                //  }
                // }
            }
        },
        {&m_drive}));

    m_superstructure.SetDefaultCommand(RunCommand(
        [this] {
            bool opPovDown = m_operatorController.GetPOV() == 180.0;
            bool opPovUp = m_operatorController.GetPOV() == 0.0;
            bool opPovLeft = m_operatorController.GetPOV() == 270.0;
            bool opPovRight = m_operatorController.GetPOV() == 90.0;
            if (abs(m_operatorController.GetRawAxis(Controller::rightYAxis)) >
                0.05) {
                m_superstructure.m_intake.setPosition(
                    m_superstructure.m_intake.getRelativePosition() +
                    m_operatorController.GetRawAxis(Controller::rightYAxis));
            } else {
                if (opPovUp && !opPrevUp) {
                    m_superstructure.m_elevator.resetEncoders();
                    // m_superstructure.algaeSecond();
                } else if (opPovDown && !opPrevDown) {
                    m_superstructure.algaeFirst();
                } else if (opPovLeft && !opPrevLeft) {
                    m_superstructure.algaeGround();
                } else if (opPovRight && !opPrevRight) {
                    m_superstructure.algaeProcessor();
                }
            }
            opPrevDown = opPovDown;
            opPrevUp = opPovUp;
            opPrevLeft = opPovLeft;
            opPrevRight = opPovRight;
        },
        {&m_superstructure}));

    m_superstructure.m_intake.SetDefaultCommand(RunCommand(
        [this] {
            frc::SmartDashboard::PutNumber(
                "trigger",
                m_operatorController.GetRawAxis(Controller::rightTrigger));
            if (m_operatorController.GetRawAxis(Controller::rightTrigger) >
                0.05) {
                m_superstructure.m_intake.setSpeed(0.8);
            } else if (m_operatorController.GetRawAxis(
                           Controller::leftTrigger) > 0.05) {
                m_superstructure.m_intake.setSpeed(-0.4);
            } else /*(m_operatorController.GetRawAxis(Controller::leftTrigger) <
                     0.05 &&
                 m_operatorController.GetRawAxis(Controller::rightTrigger) <
                     0.05)*/
            {
                m_superstructure.m_intake.stopIntake();
            }
            m_superstructure.m_intake.setSpeed(
                m_operatorController.GetRawAxis(Controller::rightYAxis) / 6);
            // if (abs(m_operatorController.GetRawAxis(Controller::rightYAxis))
            // >
            //     0.05) {

            //     // m_superstructure.m_intake.setPosition(
            //     m_superstructure.m_intake.getRelativePosition() +
            //     m_operatorController.GetRawAxis(Controller::rightYAxis));
            // }
        },
        {&m_superstructure.m_intake}));
    m_superstructure.m_outtake.SetDefaultCommand(RunCommand(
        [this] {
            SmartDashboard::PutNumber("Outtake command", 1);
            if (m_operatorController.GetRawButton(Controller::leftBumper) &&
                m_operatorController.GetRawButton(Controller::rightBumper)) {
                m_superstructure.m_outtake.setLeftSpeed(-0.2);
                m_superstructure.m_outtake.setRightSpeed(-0.2);
            } else if (m_operatorController.GetRawButton(
                           Controller::leftBumper)) {
                // m_superstructure.m_outtake.intakeCoral();
                m_superstructure.m_outtake.setLeftSpeed(-0.2);
            } else if (m_operatorController.GetRawButton(
                           Controller::rightBumper)) {
                m_superstructure.m_outtake.setRightSpeed(-0.2);
            } else if (m_operatorController.GetRawButton(
                           Controller::leftPaddle) &&
                       m_operatorController.GetRawButton(
                           Controller::rightPaddle)) {
                m_superstructure.m_outtake.setLeftSpeed(0.2);
                m_superstructure.m_outtake.setRightSpeed(0.2);
            } else if (m_operatorController.GetRawButton(
                           Controller::leftPaddle)) {
                m_superstructure.m_outtake.setLeftSpeed(0.2);

            } else if (m_operatorController.GetRawButton(
                           Controller::rightPaddle)) {
                m_superstructure.m_outtake.setRightSpeed(0.2);

            } else {
                m_superstructure.m_outtake.setSpeed(0);
            }
        },
        {&m_superstructure.m_outtake}));
    m_superstructure.m_elevator.SetDefaultCommand(RunCommand(
        [this] {
            if (abs(m_operatorController.GetRawAxis(Controller::leftYAxis)) >
                0.05) {
                double elevatorPos = m_superstructure.m_elevator.getPosition();
                // double opInput =
                //     m_operatorController.GetRawAxis(Controller::leftYAxis) /
                //     2 + 0.02;
                // if (elevatorPos < -8500 && opInput < 0.0) {
                // software limit
                // } else {
                // m_superstructure.m_elevator.setSpeed(opInput);
                m_superstructure.m_elevator.setPosition(
                    elevatorPos +
                    m_operatorController.GetRawAxis(Controller::leftYAxis));
            } else {
                if (m_operatorController.GetRawButton(Controller::Y)) {
                    m_superstructure.m_elevator.sourcePos();
                } else if (m_operatorController.GetRawButton(Controller::X)) {
                    m_superstructure.m_elevator.levelTwo();
                } else if (m_operatorController.GetRawButton(Controller::B)) {
                    m_superstructure.m_elevator.levelThree();
                } else if (m_operatorController.GetRawButton(Controller::A)) {
                    m_superstructure.m_elevator.levelFour();
                }
                // presets
            }

            // }

            // if (abs(m_operatorController.GetRawAxis(Controller::leftYAxis)) >
            //     0.05) {

            //     // if (m_superstructure.m_elevator.getPosition() <= 1.65 ||
            //     //     m_operatorController.GetRawAxis(Controller::leftYAxis)
            //     <
            //     //         -0.05) {
            //     //     m_superstructure.m_elevator.setPosition(
            //     //         m_superstructure.m_elevator.getRelativePosition()
            //     +
            //     // m_operatorController.GetRawAxis(Controller::leftYAxis));
            //     // } else {
            //     //     m_superstructure.m_elevator.sourcePos();
            //     // }
            //     //
            //     m_superstructure.m_elevator.setSpeed(m_operatorController.GetRawAxis(Controller::leftYAxis));
            //     // m_superstructure.m_elevator.setPosition(
            //     //     m_superstructure.m_elevator.getRelativePosition() +
            //     // m_operatorController.GetRawAxis(Controller::leftYAxis)/6);

            // } else {
            //     if (m_operatorController.GetRawButton(Controller::Y)) {
            //         m_superstructure.m_elevator.levelOne();
            //     }
            //     else if (m_operatorController.GetRawButton(Controller::X)) {
            //         m_superstructure.m_elevator.levelTwo();
            //     }
            //     else if (m_operatorController.GetRawButton(Controller::B)) {
            //         m_superstructure.m_elevator.levelThree();
            //     }
            //     else if (m_operatorController.GetRawButton(Controller::A)) {
            //         m_superstructure.m_elevator.levelFour();
            //     }
            //     else if (abs(m_operatorController.GetRawAxis(
            //             Controller::rightXAxis)) > 0.1) {
            //         m_superstructure.m_elevator.sourcePos();
            //     }
            // }
        },
        {&m_superstructure.m_elevator}));
    m_superstructure.m_vision.SetDefaultCommand(RunCommand(
        [this] {
            frc::SmartDashboard::PutBoolean("led button pressed", m_driverController.GetRawButtonPressed(Controller::B));

            // if
            // (m_driverController.GetPOV()<Controller::upAngle+5||m_driverController.GetPOV()>355)
            // {
            //     if (m_superstructure.m_vision.getLedOn() == 3) {
            //         // m_superstructure.m_vision.setLedOn(1);
            //     } else if (m_superstructure.m_vision.getLedOn() == 1) {
            //         // m_superstructure.m_vision.setLedOn(3);
            //     }
            //     //
            //     m_superstructure.aim(m_superstructure.m_vision.getDistance(),0,0);
            // }
            // if (m_driverController.GetRawButton(2)) {
            //     m_superstructure.aim(m_superstructure.m_vision.getDistance(),0,0);
            // }
            // frc::SmartDashboard::PutNumber("di",
            // m_superstructure.m_vision.getDistance());
            frc::SmartDashboard::PutNumber("leds", m_superstructure.m_vision.getLedOn());
            frc::SmartDashboard::PutBoolean("toggle offset", m_drive.getOffsetToggle());
        },
        {&m_superstructure.m_vision}));
}

void RobotContainer::ConfigureBindings() {
    // Configure your trigger bindings here

    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    // frc2::Trigger([this] {
    //   return m_subsystem.ExampleCondition();
    // }).OnTrue(ExampleCommand(&m_subsystem).ToPtr());

    // // Schedule `ExampleMethodCommand` when the Xbox controller's B button is
    // // pressed, cancelling on release.
    // m_driverController.B().WhileTrue(m_subsystem.ExampleMethodCommand());
}

frc2::Command* RobotContainer::GetAutonomousCommand() {
    m_drive.resetAbsoluteEncoders();
    // An example command will be run in autonomous
    // return autos::ExampleAuto(&m_subsystem);
    // auto path = PathPlannerPath::fromPathFile("Example Path");
    // return AutoBuilder::followPath(path);
    return (m_chooser.GetSelected());
}

void RobotContainer::Periodic() {}
