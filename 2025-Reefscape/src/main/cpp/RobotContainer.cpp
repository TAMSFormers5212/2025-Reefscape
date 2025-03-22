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

#include "commands/AlignToReef.h"
#include "commands/AutoIntake.h"
#include "commands/AutoOuttake.h"
#include "commands/Autos.h"
#include "commands/ElevatorL2.h"
#include "commands/ElevatorL3.h"
#include "commands/ElevatorL4.h"
#include "commands/ElevatorSource.h"
#include "commands/ExampleCommand.h"
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

    NamedCommands::registerCommand(
        "AutoIntake", AutoIntake(&m_superstructure.m_outtake).ToPtr());
    NamedCommands::registerCommand(
        "AutoOuttake", AutoOuttake(&m_superstructure.m_outtake).ToPtr());

    ConfigureBindings();

    m_testAuto = PathPlannerAuto("Test Auto").ToPtr();
    
    m_mobility = PathPlannerAuto("Mobility Auton").ToPtr();
    m_L1Center = PathPlannerAuto("L1 Center").ToPtr();
    m_L1Left = PathPlannerAuto("L1 Left").ToPtr();
    m_L1Right = PathPlannerAuto("L1 Right").ToPtr();
    m_L4Center = PathPlannerAuto("L4 Center").ToPtr();
    m_L4Left = PathPlannerAuto("L4 Left").ToPtr();
    m_L4Right = PathPlannerAuto("L4 Right").ToPtr();
    m_L1CenterL4Left =
        PathPlannerAuto("L1 Center - Left L4 [UNFINISHED]").ToPtr();
    m_L1CenterL4Right =
        PathPlannerAuto("L1 Center - Right L4 [UNFINISHED]").ToPtr();
    m_L1LeftL4Left = PathPlannerAuto("L1 Left - Left L4 [UNFINISHED]").ToPtr();
    m_L1RightL4Right =
        PathPlannerAuto("L1 Right - Right L4 [UNFINISHED]").ToPtr();

    m_chooser.SetDefaultOption("L1 Center", m_L1Center.get());
    m_chooser.AddOption("L1 Left", m_L1Left.get());
    m_chooser.AddOption("L1 Right", m_L1Right.get());

    m_chooser.AddOption("L4 Center", m_L4Center.get());
    m_chooser.AddOption("L4 Left", m_L4Left.get());
    m_chooser.AddOption("L4 Right", m_L4Right.get());

    m_chooser.AddOption("L1 Center L4 Right", m_L1CenterL4Right.get());
    m_chooser.AddOption("L1 Center L4 Left", m_L1CenterL4Left.get());
    m_chooser.AddOption("L1 Left L4 Left", m_L1LeftL4Left.get());
    m_chooser.AddOption("L1 Right L4 Right", m_L1RightL4Right.get());

    m_chooser.AddOption("Mobility Auton", m_mobility.get());
    m_chooser.AddOption("Test Auto", m_testAuto.get());

    frc::SmartDashboard::PutData(&m_chooser);

    m_drive.SetDefaultCommand(RunCommand(
        [this] {
           

            if (m_driverController.GetRawButton(Controller::Y)) {  // zero
                m_drive.resetHeading();
                m_drive.resetAbsoluteEncoders();
            } else if (m_driverController.GetRawButton(Controller::X)) {
                m_drive.setHeading(180);
            }

            auto startHeading = m_drive.getGyroHeading2();

            if (m_driverController.GetRawButton(Controller::leftBumper)) {
                speedMultiplier = 0.5;
            } else if (m_driverController.GetRawButton(
                           Controller::rightBumper)) {
                speedMultiplier = .75;
            } else {
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

            int pov = m_driverController.GetPOV();
            SmartDashboard::PutNumber("drive pov", pov);
            if (pov == 0.0) {  // up
                m_drive.swerveDrive(0.2, 0.0, 0.0, false);
            } else if (pov == 180.0) {  // down
                m_drive.swerveDrive(-0.2, 0.0, 0.0, false);
            } else if (pov == 270.0) {  // left
                m_drive.swerveDrive(-0.0, 0.2, 0.0, false);
            } else if (pov == 90.0) {  // right
                m_drive.swerveDrive(0.0, -0.2, 0.0, false);
            } 
            else if(m_driverController.GetRawButton(
                    Controller::leftPaddle)){
                // m_drive.alignAdjustment();
                m_pathfindAuto = m_drive.generateCommand();
                m_pathfindAuto.Schedule();
            }else {
                m_drive.swerveDrive(XAxis, YAxis, RotAxis, true);
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
            frc::SmartDashboard::PutBoolean("autoIntake", autoIntake);

            bool opPovDown = m_operatorController.GetPOV() == 180.0;
            bool opPovUp = m_operatorController.GetPOV() == 0.0;
            bool opPovLeft = m_operatorController.GetPOV() == 270.0;
            bool opPovRight = m_operatorController.GetPOV() == 90.0;
            frc::SmartDashboard::PutNumber("OperatorControllerRightAxis",m_operatorController.GetRawAxis(Controller::rightYAxis));
            if (abs(m_operatorController.GetRawAxis(Controller::rightYAxis)) >
                0.05) {
                if(m_superstructure.m_intake.getPosition()<=2.52&&m_superstructure.m_intake.getPosition()>=0){
                m_superstructure.m_intake.setPosition(
                    m_superstructure.m_intake.getRelativePosition() +
                    m_operatorController.GetRawAxis(Controller::rightYAxis));
                }
                else{
                    if(m_superstructure.m_intake.getPosition()>=5.9){
                        m_superstructure.m_intake.setPosition(groundPresetAngle);
                    }
                    else if(m_superstructure.m_intake.getPosition()>=2.52&&m_superstructure.m_intake.getPosition()<=5){
                        m_superstructure.m_intake.setPosition(stowPresetAngle);
                    }
                    else{
                        m_superstructure.m_intake.setPosition(m_superstructure.m_intake.getRelativePosition());
                    }
                }
            } else {

                if (opPovUp && !opPrevUp) {
                    m_superstructure.m_intake.stowPreset();
                } else if (opPovDown && !opPrevDown) {
                    
                    m_superstructure.m_intake.groundPreset();
                } else if (opPovLeft && !opPrevLeft) {
                    autoIntake = !autoIntake;
                    if (autoIntake) {
                        m_superstructure.m_outtake.setSpeed(0.12);
                        loopsBackBroken = 0;
                        m_operatorController.SetRumble(
                            frc::GenericHID::RumbleType::kLeftRumble, 0.25);
                    } else {
                        m_superstructure.m_outtake.setSpeed(0);
                        m_operatorController.SetRumble(
                            frc::GenericHID::RumbleType::kLeftRumble, 0.0);
                    }
                } else if (opPovRight && !opPrevRight) {
                    m_superstructure.m_elevator.resetEncoders();
                }
            }

            opPrevDown = opPovDown;
            opPrevUp = opPovUp;
            opPrevLeft = opPovLeft;
            opPrevRight = opPovRight;

            if (autoIntake) {
                if (m_superstructure.m_outtake.getCoralHeld()) {
                    autoIntake = false;
                    m_superstructure.m_outtake.setSpeed(0);
                    m_operatorController.SetRumble(
                        frc::GenericHID::RumbleType::kLeftRumble, 0.0);
                } else if (m_superstructure.m_outtake.getBeamFront()) {
                        m_superstructure.m_outtake.setSpeed(0.03);
                } else if (m_superstructure.m_outtake.getBeamBack()) {
                    loopsBackBroken++;
                    if (loopsBackBroken > 10) {
                        m_superstructure.m_outtake.setSpeed(0.05);
                    }
                }
            }
        },
        {&m_superstructure}));

    m_superstructure.m_outtake.SetDefaultCommand(RunCommand(
        [this] {
            bool leftBumper =
                m_operatorController.GetRawButton(Controller::leftBumper);
            bool rightBumper =
                m_operatorController.GetRawButton(Controller::rightBumper);
            bool leftPaddle =
                m_operatorController.GetRawButton(Controller::leftPaddle);
            bool rightPaddle =
                m_operatorController.GetRawButton(Controller::rightPaddle);

            if (leftBumper || rightBumper) {
                m_superstructure.m_outtake.setLeftSpeed(
                    m_operatorController.GetRawButton(Controller::leftBumper)
                        ? -0.2
                        : 0.0);
                m_superstructure.m_outtake.setRightSpeed(
                    m_operatorController.GetRawButton(Controller::rightBumper)
                        ? -0.2
                        : 0.0);
            } else if (leftPaddle || rightPaddle) {
                m_superstructure.m_outtake.setLeftSpeed(
                    m_operatorController.GetRawButton(Controller::leftPaddle)
                        ? 0.2
                        : 0.0);

                m_superstructure.m_outtake.setRightSpeed(
                    m_operatorController.GetRawButton(Controller::rightPaddle)
                        ? 0.2
                        : 0.0);
            } else if (!autoIntake) {
                m_superstructure.m_outtake.setSpeed(0.0);
            }
        },
        {&m_superstructure.m_outtake}));

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
                m_superstructure.m_intake.setSpeed(0.0);
            }
            m_superstructure.m_intake.setSpeed(
                m_operatorController.GetRawAxis(Controller::rightYAxis) / 6);
            // if (abs(m_operatorController.GetRawAxis(Controller::rightYAxis))>0.05 && m_superstructure.m_intake.getPosition() >= 0 && m_superstructure.m_intake.getPosition() <= 0.38) {

            //     m_superstructure.m_intake.setPosition(
            //     m_superstructure.m_intake.getRelativePosition() +
            //     m_operatorController.GetRawAxis(Controller::rightYAxis));
            // }
            // else if(m_superstructure.m_intake.getPosition() >= 0.7) {
            //     m_superstructure.m_intake.setPosition(
            //     m_superstructure.m_intake.getRelativePosition() +
            //     0.01);
            // }
            // else if(m_superstructure.m_intake.getPosition() > 0.38 && m_superstructure.m_intake.getPosition() < 0.7) {
            //     m_superstructure.m_intake.setPosition(
            //     m_superstructure.m_intake.getRelativePosition() -
            //     0.01);
            // }
        },
        {&m_superstructure.m_intake}));

    m_superstructure.m_elevator.SetDefaultCommand(RunCommand(
        [this] {
            frc::SmartDashboard::PutBoolean("Elevator Override", override_);
            if (m_operatorController.GetRawButtonPressed(Controller::LPress)) {
                override_ = !override_;
            }
            if (abs(m_operatorController.GetRawAxis(Controller::leftYAxis)) >
                0.05) {
                double elevatorPos = m_superstructure.m_elevator.getPosition();
                // double opInput =
                //     m_operatorController.GetRawAxis(Controller::leftYAxis) /
                //     2 + 0.02;
                // m_superstructure.m_elevator.setSpeed(opInput);
                m_superstructure.m_elevator.setPosition(
                    elevatorPos +
                    m_operatorController.GetRawAxis(Controller::leftYAxis));
            } else {
                if (m_superstructure.m_outtake.getBeamBack() && !override_) {
                    // noop
                } else {
                    if (m_operatorController.GetRawButton(Controller::Y)) {
                        m_superstructure.m_elevator.sourcePos();
                    } else if (m_operatorController.GetRawButton(
                                   Controller::X)) {
                        m_superstructure.m_elevator.levelTwo();
                    } else if (m_operatorController.GetRawButton(
                                   Controller::B)) {
                        m_superstructure.m_elevator.levelThree();
                    } else if (m_operatorController.GetRawButton(
                                   Controller::A)) {
                        m_superstructure.m_elevator.levelFour();
                    }
                }
            }
        },
        {&m_superstructure.m_elevator}));
    m_superstructure.m_vision.SetDefaultCommand(RunCommand(
        [this] {
            frc::SmartDashboard::PutBoolean(
                "led button pressed",
                m_driverController.GetRawButtonPressed(Controller::B));

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
            frc::SmartDashboard::PutNumber(
                "leds", m_superstructure.m_vision.getLedOn());
            frc::SmartDashboard::PutBoolean("toggle offset",
                                            m_drive.getOffsetToggle());
        },
        {&m_superstructure.m_vision}));
}

void RobotContainer::ConfigureBindings() {
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

    // return (*m_chooser.GetSelected()).ToPtr();
    PathPlannerAuto selectedAuton(m_chooser.GetSelected()->GetName());
    m_drive.resetOdometry(selectedAuton.getStartingPose());
    return (m_chooser.GetSelected());
}

void RobotContainer::Periodic() {}

// frc::Pose2d RobotContainer::autoStartingPose(void) {
//     // return m_mobility.getStartingPose();
// }
