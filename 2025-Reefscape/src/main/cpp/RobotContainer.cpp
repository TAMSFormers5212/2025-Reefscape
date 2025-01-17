// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <frc2/command/button/Trigger.h>

#include "commands/Autos.h"
#include "commands/ExampleCommand.h"
#include <pathplanner/lib/path/PathPlannerPath.h>
#include <pathplanner/lib/commands/PathPlannerAuto.h>
#include <pathplanner/lib/auto/AutoBuilder.h>
#include <frc2/command/CommandPtr.h>
#include <frc/controller/PIDController.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/Commands.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/RunCommand.h>
#include <frc2/command/SequentialCommandGroup.h>
#include <frc2/command/StartEndCommand.h>
#include <frc2/command/button/JoystickButton.h>
#include <frc2/command/button/POVButton.h>
#include <frc2/command/button/Trigger.h>
#include <math.h>
#include <frc2/command/CommandPtr.h>


#include "networktables/NetworkTable.h"
#include "networktables/NetworkTableEntry.h"
#include "networktables/NetworkTableInstance.h"
#include "networktables/NetworkTableValue.h"
#include <pathplanner/lib/commands/PathPlannerAuto.h>
#include <pathplanner/lib/auto/AutoBuilder.h>
#include <pathplanner/lib/path/PathPlannerPath.h>

#include <pathplanner/lib/auto/NamedCommands.h>

#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/Commands.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <iostream>
#include <frc2/command/Command.h>
#include <frc2/command/ParallelRaceGroup.h>
#include <frc2/command/ParallelCommandGroup.h>
#include <pathplanner/lib/auto/NamedCommands.h>
#include <frc/geometry/Rotation2d.h>
#include <frc2/command/WaitCommand.h>
using namespace pathplanner;
using namespace std;
using namespace frc2;
using namespace OIConstants;
RobotContainer::RobotContainer() {
  // Initialize all of your commands and subsystems here

  // Configure the button bindings
  ConfigureBindings();
  m_drive.SetDefaultCommand(RunCommand(
        [this] {
            auto rot = m_drive.getGyroHeading2();
    
    if (frc::DriverStation::GetAlliance().value() == frc::DriverStation::Alliance::kRed){
        
        //rot = Rotation2d(180_deg).RotateBy(rot);
    }
    //frc::SmartDashboard::PutNumber("roa t", rot.Degrees().value());
    //frc::SmartDashboard::PutNumber("gyro offset", m_drive.getGyroHeading2().Degrees().value());
            speedMultiplier = (1 - m_driverController.GetRawAxis(Joystick::ThrottleSlider)) * 0.5;
            XAxis = -m_driverController.GetRawAxis(Joystick::XAxis) * speedMultiplier;
            YAxis = m_driverController.GetRawAxis(Joystick::YAxis) * speedMultiplier;
            RotAxis = -m_driverController.GetRawAxis(Joystick::RotAxis) * speedMultiplier*2;
            frc::SmartDashboard::PutNumber("speedToggle", m_driverController.GetRawAxis(Joystick::ThrottleSlider));
            frc::SmartDashboard::PutNumber("speed", speedMultiplier * 100);
            double rotDeadband = Joystick::deadband*2;
            if (abs(XAxis) < (Joystick::deadband*speedMultiplier)) {    
                XAxis = 0;
            }
            if (abs(YAxis) < (Joystick::deadband*speedMultiplier)) {
                YAxis = 0;
            }
            if (abs(RotAxis) < (rotDeadband*speedMultiplier)) {
                RotAxis = 0;
            }

            //frc::SmartDashboard::PutNumber("x", XAxis);
            //frc::SmartDashboard::PutNumber("y", YAxis);
            //frc::SmartDashboard::PutNumber("rot", RotAxis);

            if (m_driverController.GetRawButton(11)) {
                m_drive.moveToAngle(XAxis, YAxis);
            } else if (m_driverController.GetRawButton(12)) {
                m_drive.moveToAngle(0, 0.3);
            }
            std::shared_ptr<nt::NetworkTable> table = nt::NetworkTableInstance::GetDefault().GetTable("limelight");

            if (m_driverController.GetRawButton(7)) {
               m_drive.toggleOffset();
            }
            // if (m_operatorController.GetRawAxis(Controller::leftTrigger)>0.05||m_driverController.GetRawButton(8)) {
            //     if (m_superstructure.m_vision.isTagPresent()){
            //     // if (m_vision.getDistanceError() > 0 &&
            //     //     m_vision.getDistanceError() < 25) {
            //          RotAxis += m_superstructure.m_vision.getOutput()* 0.2;
            //         //  YAxis += m_superstructure.m_vision.getDistanceError() * speedMultiplier;  
            //         //  }
            //     }\
                
            // }
            if (m_driverController.GetRawButton(6)){
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
        {&m_drive}
    ));
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
