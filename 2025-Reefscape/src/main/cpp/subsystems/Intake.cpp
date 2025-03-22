// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/Intake.h"

#include <frc/DigitalInput.h>
#include <frc/smartdashboard/Mechanism2d.h>
#include <frc/smartdashboard/SmartDashboard.h>

#include <cmath>
#include <iostream>

#include "Constants.h"

using namespace IntakeConstants;
using namespace rev;
using namespace std;
using namespace MathConstants;

Intake::Intake(int intakeMotor, int pivotMotor, int encoder,
               double encoderOffset)
    : m_intakeMotor(intakeMotor, rev::spark::SparkMax::MotorType::kBrushless),
      m_pivotMotor(pivotMotor, rev::spark::SparkMax::MotorType::kBrushless),
      m_intakeConfig(),
      m_pivotConfig(),
      m_pivotFF(kiS, kiG, kiV) {
    resetMotor();
    position=getPosition();
    m_intakeMotor.Configure(m_intakeConfig,
                            SparkMax::ResetMode::kResetSafeParameters,
                            SparkMax::PersistMode::kPersistParameters);
    m_pivotMotor.Configure(m_pivotConfig,
                           SparkMax::ResetMode::kResetSafeParameters,
                           SparkMax::PersistMode::kPersistParameters);
}

void Intake::resetMotor() {
    m_intakeConfig.SetIdleMode(rev::spark::SparkBaseConfig::IdleMode::kBrake)
        .VoltageCompensation(12.0)
        .SmartCurrentLimit(20, 25)
        .Inverted(true);

    m_intakeConfig.encoder.PositionConversionFactor(pi2 / intakeRatio);

    m_pivotConfig.SetIdleMode(rev::spark::SparkBaseConfig::IdleMode::kBrake)
        .VoltageCompensation(12.0)
        .SmartCurrentLimit(20, 25)
        .Inverted(true);

    m_pivotConfig.encoder.PositionConversionFactor(pi2 / pivotRatio);

    m_pivotConfig.closedLoop.Pidf(kiP, kiI, kiD, kiFF)
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
    m_intakeMotor.Configure(m_intakeConfig,
                            SparkMax::ResetMode::kResetSafeParameters,
                            SparkMax::PersistMode::kPersistParameters);
    m_pivotMotor.Configure(m_pivotConfig,
                           SparkMax::ResetMode::kResetSafeParameters,
                           SparkMax::PersistMode::kPersistParameters);
    // m_encoder.SetPositionConversionFactor(1.0 / intakeRatio);
}

void Intake::resetEncoder() {
    m_pivotEncoder.SetPosition(getPosition());
}

void Intake::setPosition(double pivotPose) {
    position = pivotPose;
    intakeCommandGiven = true;
}

double Intake::getRelativePosition() { return m_pivotEncoder.GetPosition(); }

double Intake::getPosition() {
    return abs(m_absoluteEncoder.Get() - pivotOffset) * pi2;
}

void Intake::stowPreset() { setPosition(getRelativePosition()+(stowPresetAngle)-getPosition()); }
void Intake::groundPreset() { setPosition(getRelativePosition()+(stowPresetAngle)-getPosition()); }
void Intake::processorPreset() { setPosition(processorPresetAngle); }
void Intake::reefPreset() { setPosition(reefPresetAngle); }

void Intake::setSpeed(double speed) { m_intakeMotor.Set(speed); }
void Intake::setPivotSpeed(double speed) { m_pivotMotor.Set(speed); }

double Intake::getSpeed() { return m_encoder.GetVelocity(); }

double Intake::getOutputCurrent() { return m_intakeMotor.GetOutputCurrent(); }

void Intake::Periodic() {
    units::radian_t ffP{position}; //+ getPosition() - getRelativePosition()};
    units::radians_per_second_t ffV{0};
    units::radians_per_second_squared_t ffA(0);
    if (intakeCommandGiven) {
        m_pivotController.SetReference(
            position, rev::spark::SparkLowLevel::ControlType::kPosition,
            rev::spark::kSlot0, m_pivotFF.Calculate(ffP, ffV, ffA).value());
    } else {
        m_pivotController.SetReference(
            m_pivotFF.Calculate(ffP, ffV, ffA).value(),
            rev::spark::SparkLowLevel::ControlType::kVoltage);
    }
    frc::SmartDashboard::PutNumber("intake target pos", position);
    frc::SmartDashboard::PutNumber("intake abs pos", m_absoluteEncoder.Get());
    frc::SmartDashboard::PutNumber("intake abs pos offset", m_absoluteEncoder.Get()-pivotOffset);
    frc::SmartDashboard::PutNumber("intake abs pos radians", getPosition());
    frc::SmartDashboard::PutNumber("intake frequency",
                                   m_absoluteEncoder.GetFrequency());
    frc::SmartDashboard::PutBoolean("intake abs connected",
                                    m_absoluteEncoder.IsConnected());

    frc::SmartDashboard::PutNumber("Intake neo pos",
                                   m_pivotEncoder.GetPosition());
}
