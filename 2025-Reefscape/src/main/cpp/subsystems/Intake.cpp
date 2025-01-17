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
Intake::Intake(int motor): m_intakeMotor(motor, rev::spark::SparkMax::MotorType::kBrushless),
    m_intakeConfig() {
        resetMotor();
    m_intakeMotor.Configure(m_intakeConfig, SparkMax::ResetMode::kResetSafeParameters, SparkMax::PersistMode::kPersistParameters);
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

    m_intakeMotor.Configure(m_intakeConfig, SparkMax::ResetMode::kResetSafeParameters, SparkMax::PersistMode::kPersistParameters);
    // m_encoder.SetPositionConversionFactor(1.0 / intakeRatio);
}


void Intake::Periodic() {
  // Implementation of subsystem periodic method goes here.
}

// void ExampleSubsystem::SimulationPeriodic() {
//   // Implementation of subsystem simulation periodic method goes here.
// }
