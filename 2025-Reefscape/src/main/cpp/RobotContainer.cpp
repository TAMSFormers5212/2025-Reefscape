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
#include "commands/ExampleCommand.h"
#include "commands/LiftElevator.h"
#include "networktables/NetworkTable.h"
#include "networktables/NetworkTableEntry.h"
#include "networktables/NetworkTableInstance.h"
#include "networktables/NetworkTableValue.h"

using namespace pathplanner;
using namespace std;
using namespace frc2;
using namespace OIConstants;

RobotContainer::RobotContainer() {
    NamedCommands::registerCommand("Lift Elevator", cmds::LiftElevator(&m_elevator).ToPtr());

    ConfigureBindings();
    m_drive.SetDefaultCommand(RunCommand(
        [this] {
            auto rot = m_drive.getGyroHeading2();

            if (frc::DriverStation::GetAlliance().value() ==
                frc::DriverStation::Alliance::kRed) {
                // rot = Rotation2d(180_deg).RotateBy(rot);
            }
            speedMultiplier =
                (1 - m_driverController.GetRawAxis(Joystick::ThrottleSlider)) *
                0.5;
            XAxis = -m_driverController.GetRawAxis(Joystick::XAxis) *
                    speedMultiplier;
            YAxis = m_driverController.GetRawAxis(Joystick::YAxis) *
                    speedMultiplier;
            RotAxis = -m_driverController.GetRawAxis(Joystick::RotAxis) *
                      speedMultiplier * 2;
            frc::SmartDashboard::PutNumber(
                "speedToggle",
                m_driverController.GetRawAxis(Joystick::ThrottleSlider));
            frc::SmartDashboard::PutNumber("speed", speedMultiplier * 100);
            double rotDeadband = Joystick::deadband * 2;
            if (abs(XAxis) < (Joystick::deadband * speedMultiplier)) {
                XAxis = 0;
            }
            if (abs(YAxis) < (Joystick::deadband * speedMultiplier)) {
                YAxis = 0;
            }
            if (abs(RotAxis) < (rotDeadband * speedMultiplier)) {
                RotAxis = 0;
            }

            if (m_driverController.GetRawButton(11)) {
                m_drive.moveToAngle(XAxis, YAxis);
            } else if (m_driverController.GetRawButton(12)) {
                m_drive.moveToAngle(0, 0.3);
            }
            std::shared_ptr<nt::NetworkTable> table =
                nt::NetworkTableInstance::GetDefault().GetTable("limelight");

            if (m_driverController.GetRawButton(7)) {
                m_drive.toggleOffset();
            }
            if (m_operatorController.GetRawButton(Controller::down) ||
                m_driverController.GetRawButton(8)) {
                if (m_superstructure.m_vision.isTagPresent()) {
                    // if (m_vision.getDistanceError() > 0 &&
                    //     m_vision.getDistanceError() < 25) {
                    RotAxis += m_superstructure.m_vision.getOutput() * 0.2;
                    //  YAxis += m_superstructure.m_vision.getDistanceError() *
                    //  speedMultiplier;
                    //  }
                }
            }
            if (m_driverController.GetRawButton(6)) {
                m_drive.resetAbsoluteEncoders();
            }
            // if (m_driverController.GetRawButton(6)) {
            //     m_drive.tankDrive(XAxis, YAxis);
            // }

            // if (m_driverController.GetRawButton(1)){
            //     m_superstructure.aim(m_vision.getDistance(),0,0);
            // }
            // if (m_driverController.GetRawButton(2)){
            //     m_superstructure.aim(0.231,600);
            // }
            m_drive.swerveDrive(XAxis, YAxis, RotAxis, true);
        },
        {&m_drive}));
    m_superstructure.m_intake.SetDefaultCommand(RunCommand(
        [this] {
            if (m_operatorController.GetRawAxis(Controller::rightTrigger) >
                0.05) {
                // if (m_superstructure.m_shooter.getSpeed()>400){
                // m_superstructure.m_intake.setSpeed(1.00);
                // }
                // else{
                m_superstructure.m_intake.setSpeed(0.8);
                // }
            }
            if (m_operatorController.GetRawAxis(Controller::leftTrigger) >
                0.05) {
                m_superstructure.m_intake.setSpeed(-0.4);
            }
            if (m_operatorController.GetRawAxis(Controller::leftTrigger) <
                    0.05 &&
                m_operatorController.GetRawButton(Controller::rightBumper) <
                    0.05) {
                m_superstructure.m_intake.stopIntake();
            }
        },
        {&m_superstructure.m_intake}));
    m_superstructure.m_outtake.SetDefaultCommand(RunCommand(
        [this] {
            if (m_operatorController.GetRawButton(Controller::leftBumper)) {
                m_superstructure.m_outtake.intakeCoral();
            }
            if (m_operatorController.GetRawButton(Controller::rightBumper)) {
                m_superstructure.m_outtake.setSpeed(-0.4);
            }
        },
        {&m_superstructure.m_outtake}));
    m_superstructure.m_elevator.SetDefaultCommand(RunCommand(
        [this] {

        },
        {&m_superstructure.m_elevator}));
    m_superstructure.m_vision.SetDefaultCommand(RunCommand(
        [this] {
            if (m_driverController.GetRawButtonPressed(2)) {
                frc::SmartDashboard::PutBoolean(
                    "led button pressed",
                    m_driverController.GetRawButtonPressed(2));
                if (m_superstructure.m_vision.getLedOn() == 3) {
                    m_superstructure.m_vision.setLedOn(1);
                } else if (m_superstructure.m_vision.getLedOn() == 1) {
                    m_superstructure.m_vision.setLedOn(3);
                }
                // m_superstructure.aim(m_superstructure.m_vision.getDistance(),0,0);
            }
            // if (m_driverController.GetRawButton(2)) {
            //     m_superstructure.aim(m_superstructure.m_vision.getDistance(),0,0);
            // }
            // frc::SmartDashboard::PutNumber("di",
            // m_superstructure.m_vision.getDistance());
            frc::SmartDashboard::PutNumber(
                "leds", m_superstructure.m_vision.getLedOn());
            frc::SmartDashboard::PutBoolean("toggle offset",
                                            m_drive.getOffsetToggle());
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

frc2::CommandPtr RobotContainer::GetAutonomousCommand() {
    // An example command will be run in autonomous
    // return autos::ExampleAuto(&m_subsystem);
    auto path = PathPlannerPath::fromPathFile("Example Path");
    return AutoBuilder::followPath(path);
}

void RobotContainer::Periodic() {}
