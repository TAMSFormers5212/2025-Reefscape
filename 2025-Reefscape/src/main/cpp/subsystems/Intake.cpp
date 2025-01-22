// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/Intake.h"
#include "Constants.h"
#include <frc/smartdashboard/Mechanism2d.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <cmath>
#include <iostream>
#include <frc/DigitalInput.h>

using namespace IntakeConstants;
using namespace rev;
using namespace std;
using namespace MathConstants;
Intake::Intake(int intakeMotor, int pivotMotor, int encoder, double encoderOffset): m_intakeMotor(intakeMotor, rev::spark::SparkMax::MotorType::kBrushless),
  m_pivotMotor(pivotMotor, rev::spark::SparkMax::MotorType::kBrushless),
    m_intakeConfig(), m_pivotConfig(), m_pivotFF(kiS, kiA, kiV) {
        resetMotor();
    m_intakeMotor.Configure(m_intakeConfig, SparkMax::ResetMode::kResetSafeParameters, SparkMax::PersistMode::kPersistParameters);
    m_pivotMotor.Configure(m_pivotConfig, SparkMax::ResetMode::kResetSafeParameters, SparkMax::PersistMode::kPersistParameters);  
  // Implementation of subsystem constructor goes here.
}
void Intake::resetMotor() {

    m_intakeConfig
       .SetIdleMode(rev::spark::SparkBaseConfig::IdleMode::kBrake)
       .VoltageCompensation(12.0)
       .SmartCurrentLimit(20, 25)
       .Inverted(true);

    m_intakeConfig.encoder
        .PositionConversionFactor(1.0 / intakeRatio);

    m_pivotConfig
       .SetIdleMode(rev::spark::SparkBaseConfig::IdleMode::kBrake)
       .VoltageCompensation(12.0)
       .SmartCurrentLimit(20, 25)
       .Inverted(true);

    m_pivotConfig.encoder
        .PositionConversionFactor(1.0 / pivotRatio);
    
    m_pivotConfig.closedLoop
      .Pidf(kiP, kiI, kiD, kiFF)
      .IZone(kiIz)
      .OutputRange(kiMinOutput, kiMaxOutput);

    // m_intakeController.SetP(kiP);
    // m_intakeController.SetI(kaI);
    // m_intakeController.SetD(kaD);
    // m_intakeController.SetFF(kaFF);
    // m_intakeController.SetIZone(kaIz);
    // m_intakeController.SetOutputRange(kMinOutput, kMaxOutput);

    // m_intakeController.SetSmartMotionMaxAccel(maxAccel);
    // m_leftController.SetSmartMotionMaxVelocity(maxVelo);
    // m_leftController.SetSmartMotionMinOutputVelocity(0);
    // m_leftController.SetSmartMotionAllowedClosedLoopError(allowedError);

    // m_intakeMotor.SetIdleMode(CANSparkBase::IdleMode::kBrake);
    // m_intakeMotor.EnableVoltageCompensation(12.0);
    // m_intakeMotor.SetSmartCurrentLimit(20, 25);
    // m_intakeMotor.SetInverted(true);
    m_pivotEncoder.SetPosition(getPosition());
    resetEncoder();
    m_intakeMotor.Configure(m_intakeConfig, SparkMax::ResetMode::kResetSafeParameters, SparkMax::PersistMode::kPersistParameters);
     m_pivotMotor.Configure(m_pivotConfig, SparkMax::ResetMode::kResetSafeParameters, SparkMax::PersistMode::kPersistParameters);
    // m_encoder.SetPositionConversionFactor(1.0 / intakeRatio);
}
void Intake::resetEncoder(){
  m_pivotEncoder.SetPosition(getPosition());
  initalPosition = getPosition();
}
void Intake::setPosition(double pivotPose){
  position=pivotPose;
}
double Intake::getPosition(){
   return abs(m_absoluteEncoder.Get()-pivotOffset)*pi2;
}
void Intake::set(double value){
  m_pivotMotor.Set(value);
}
double Intake::groundPreset(){
  //idk
  return 0;
}
double Intake::processorPreset(){
  //idk
  return 0;
}
void Intake::setSpeed(double speed) { m_intakeMotor.Set(speed); }
void Intake::stopIntake() {  // in case of 2 notes and need to eject 
    m_intakeMotor.Set(0);
    
}
double Intake::getSpeed(){
  return m_encoder.GetVelocity();
}
double Intake::getOutputCurrent(){ 
  return m_intakeMotor.GetOutputCurrent(); 
}

int Intake::getState() { return state; }

void Intake::setState(int state) { this->state = state; }

void Intake::Periodic() {
    units::radian_t ffP{position+getPosition()-m_pivotEncoder.GetPosition()};
    units::radians_per_second_t ffV{0};
    units::radians_per_second_squared_t ffA(0);
  // Implementation of subsystem periodic method goes here.
}

// void ExampleSubsystem::SimulationPeriodic() {
//   // Implementation of subsystem simulation periodic method goes here.
// }
