// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/Outtake.h"
#include "Constants.h"
#include <frc/smartdashboard/Mechanism2d.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <cmath>
#include <iostream>
#include <frc/DigitalInput.h>

using namespace OuttakeConstants;
using namespace rev;
using namespace std;
using namespace MathConstants;
Outtake::Outtake(int motor, int sensor): m_outtakeMotor(motor, rev::spark::SparkMax::MotorType::kBrushless),
    m_outtakeConfig() {
        resetMotor();
    m_outtakeMotor.Configure(m_outtakeConfig, SparkMax::ResetMode::kResetSafeParameters, SparkMax::PersistMode::kPersistParameters);
  // Implementation of subsystem constructor goes here.
}
void Outtake::resetMotor() {

    m_outtakeConfig
       .SetIdleMode(rev::spark::SparkBaseConfig::IdleMode::kBrake)
       .VoltageCompensation(12.0)
       .SmartCurrentLimit(20, 25)
       .Inverted(true);

    m_outtakeConfig.encoder
    //replace intakeRatio
        .PositionConversionFactor(1.0 / pulleyRatio);
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

    m_outtakeMotor.Configure(m_outtakeConfig, SparkMax::ResetMode::kResetSafeParameters, SparkMax::PersistMode::kPersistParameters);
    // m_encoder.SetPositionConversionFactor(1.0 / intakeRatio);
}
void Outtake::stopOuttake() {  // in case of 2 notes and need to eject 
    m_outtakeMotor.Set(0);
}
void Outtake::intakeCoral(){
  while(!coralHeld){
   setSpeed(0.4);
  }
  stopOuttake();
}
void Outtake::setSpeed(double speed){
  m_outtakeMotor.Set(speed);
}
double Outtake::getSpeed(){
  return m_encoder.GetVelocity();
  }

bool Outtake::getCoral(){
  return coralHeld;
}
void Outtake::Periodic() {
  // Implementation of subsystem periodic method goes here.
  if (m_beamBreak.Get()==0){
        coralHeld = true;
        //m_LEDs.setColor(0.65);
    }
    else{
        coralHeld= false;
        //m_LEDs.setColor(0.77);
    }
    frc::SmartDashboard::PutBoolean("holding coral", coralHeld);
}

// void ExampleSubsystem::SimulationPeriodic() {
//   // Implementation of subsystem simulation periodic method goes here.
// }
