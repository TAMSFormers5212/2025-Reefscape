// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/Superstructure.h"
Superstructure::Superstructure(): 
m_intake(IntakeConstants::intakeMotor, IntakeConstants::pivotMotor, IntakeConstants::pivotEncoder, IntakeConstants::pivotOffset), m_outtake(OuttakeConstants::leftMotor, OuttakeConstants::rightMotor, OuttakeConstants::beamBreakIO), 
m_elevator(ElevatorConstants::leftMotor, ElevatorConstants::rightMotor,ElevatorConstants::encoder, ElevatorConstants::encoderOffset ),
m_vision() {
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

void Superstructure::algaeGround(){
  m_elevator.groundAlgae();
  m_intake.setPosition(m_intake.groundPreset());
}

void Superstructure::algaeProcessor(){
  m_elevator.processor();
  m_intake.setPosition(m_intake.processorPreset());
}

void Superstructure::algaeFirst(){
  m_elevator.firstAlgae();
  m_intake.setPosition(m_intake.firstAlgaePreset());
}

void Superstructure::algaeSecond(){
  m_elevator.secondAlgae();
  m_intake.setPosition(m_intake.secondAlgaePreset());
}

void Superstructure::Periodic() {
  // Implementation of subsystem periodic method goes here.
}

void Superstructure::SimulationPeriodic() {
  // Implementation of subsystem simulation periodic method goes here.
}
