// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/Superstructure.h"

Superstructure::Superstructure(): 
m_intake(IntakeConstants::motor), m_outtake(OuttakeConstants::leftMotor), 
m_elevator(ElevatorConstants::leftMotor, ElevatorConstants::rightMotor,ElevatorConstants::encoder, ElevatorConstants::encoderOffset ) {
  resetSuperstructure();
  // Implementation of subsystem constructor goes here.
}

void Superstructure::resetSuperstructure(){
    // m_arm.resetMotors();
    m_intake.resetMotor();
    m_outtake.resetMotor();
    m_elevator.resetMotors();
    // m_shooter.resetMotors();
    // m_leftWinch.resetMotor();
    // m_rightWinch.resetMotor();
}
// frc2::CommandPtr ExampleSubsystem::ExampleMethodCommand() {
//   // Inline construction of command goes here.
//   // Subsystem::RunOnce implicitly requires `this` subsystem.
//   return RunOnce([/* this */] { /* one-time action goes here */ });
// }

// bool ExampleSubsystem::ExampleCondition() {
//   // Query some boolean state, such as a digital sensor.
//   return false;
// }

void Superstructure::Periodic() {
  // Implementation of subsystem periodic method goes here.
}

void Superstructure::SimulationPeriodic() {
  // Implementation of subsystem simulation periodic method goes here.
}
