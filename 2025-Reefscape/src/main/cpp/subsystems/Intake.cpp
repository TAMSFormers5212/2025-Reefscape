// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/Intake.h"
#include "Constants.h"

#include <frc/DigitalInput.h>
#include <frc/smartdashboard/Mechanism2d.h>
#include <frc/smartdashboard/SmartDashboard.h>

#include <cmath>
#include <iostream>


using namespace IntakeConstants;
using namespace rev;
using namespace std;
using namespace MathConstants;

using frc::SmartDashboard;

Intake::Intake(int intakeMotor, int pivotMotor, int encoder,
               double encoderOffset)
    : m_intakeMotor(intakeMotor, rev::spark::SparkMax::MotorType::kBrushless),
      m_pivotMotor(pivotMotor, rev::spark::SparkMax::MotorType::kBrushless),
      m_intakeConfig(),
      m_pivotConfig(),
      m_pivotFF(IntakeConstants::kiS, IntakeConstants::kiG,
                IntakeConstants::kiV) {
    resetMotor();
    position = getPosition();
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
        .Inverted(false);

    m_pivotConfig.encoder.PositionConversionFactor(pi2 / pivotRatio);

    m_pivotConfig.closedLoop.Pidf(kiP, kiI, kiD, kiFF)
        .IZone(kiIz)
        .OutputRange(kiMinOutput, kiMaxOutput);

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

void Intake::resetEncoder() { m_pivotEncoder.SetPosition(getPosition()); }

void Intake::setTargetPosition(double pivotPose) {
    if (pivotPose > 90) pivotPose = 90;
    if (pivotPose < -30) pivotPose = -30;

    position = pivotPose;
    intakeCommandGiven = true;
}

double Intake::getTargetPosition(void) { return position; }

double Intake::getRelativePosition() { return m_pivotEncoder.GetPosition(); }

double Intake::getPosition() {
    double i360 = (m_absoluteEncoder.Get() - pivotOffset) * 360;
    if (i360 > 180) return i360 - 360;
    if (i360 < -180) return i360 + 360;
    return i360;
}

void Intake::stowPreset() { setTargetPosition(stowPresetAngle); }

void Intake::groundPreset() { setTargetPosition(groundPresetAngle); }

void Intake::processorPreset() { setTargetPosition(processorPresetAngle); }
void Intake::reefPreset() { setTargetPosition(reefPresetAngle); }

void Intake::setSpeed(double speed) { m_intakeMotor.Set(speed); }
void Intake::setPivotSpeed(double speed) { m_pivotMotor.Set(speed); }

double Intake::getSpeed() { return m_encoder.GetVelocity(); }

double Intake::getOutputCurrent() { return m_intakeMotor.GetOutputCurrent(); }

void Intake::Periodic() {
    const double kP = 0.002;
    const double kCos = 0.0001;
    double currentPos;

    if (m_absoluteEncoder.IsConnected()) {
        currentPos = getPosition();
    } else {
        currentPos = getRelativePosition();
    }

    double targetPos = position;
    double error = targetPos - currentPos;

    double pid = error * kP;
    double ff = cos(currentPos) * kCos;

    double power = ff;  // pid + ff;
    setPivotSpeed(power);

    // units::radian_t ffP{position*pi2/180};
    // units::radians_per_second_t ffV{0};
    // units::radians_per_second_squared_t ffA{0};
    // setPivotSpeed(m_pivotFF.Calculate(ffP,ffV,ffA).value());

    SmartDashboard::PutNumber("intake abs pos offset",
                              m_absoluteEncoder.Get() - pivotOffset);
    SmartDashboard::PutNumber("intake abs pos", m_absoluteEncoder.Get());
    SmartDashboard::PutNumber("Intake neo pos", m_pivotEncoder.GetPosition());

    SmartDashboard::PutNumber("pivot target pos", targetPos);
    SmartDashboard::PutNumber("pivot current pos", currentPos);
    SmartDashboard::PutNumber("pivot error", error);
    SmartDashboard::PutNumber("pivot power", power);
    SmartDashboard::PutNumber("pivot pid", pid);
    SmartDashboard::PutNumber("pivot ff", ff);

    SmartDashboard::PutNumber("intake frequency",
                              m_absoluteEncoder.GetFrequency());
    SmartDashboard::PutBoolean("intake abs connected",
                               m_absoluteEncoder.IsConnected());
}
