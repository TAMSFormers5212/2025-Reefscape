// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/Elevator.h"

#include <frc/smartdashboard/Mechanism2d.h>
#include <frc/smartdashboard/SmartDashboard.h>

#include <cmath>
#include <iostream>

using namespace ElevatorConstants;
using namespace rev;
using namespace std;
using namespace MathConstants;

using frc::SmartDashboard;

Elevator::Elevator(int leftMotor, int rightMotor, int encoderOne,
                   int encoderTwo, double encoderOffset)
    : m_leftMotor(leftMotor, rev::spark::SparkMax::MotorType::kBrushless),
      m_rightMotor(rightMotor, rev::spark::SparkMax::MotorType::kBrushless),
      m_leftConfig(),
      m_rightConfig(),
      m_elevatorFF(ElevatorConstants::kaS, ElevatorConstants::kaG,
                   ElevatorConstants::kaV) {
    resetMotors();
}

void Elevator::resetMotors() {
    m_leftConfig.closedLoop.Pidf(kaP, kaI, kaD, kaFF)
        .IZone(kaIz)
        .OutputRange(kMinOutput, kMaxOutput);
    m_rightConfig.SetIdleMode(rev::spark::SparkBaseConfig::IdleMode::kBrake)
        .VoltageCompensation(12.0)
        .SmartCurrentLimit(20)
        .Inverted(true);
    m_rightConfig.encoder.PositionConversionFactor(pi2 / elevatorRatio);
    m_rightMotor.Configure(m_rightConfig,
                           SparkMax::ResetMode::kResetSafeParameters,
                           SparkMax::PersistMode::kPersistParameters);

    m_leftConfig.SetIdleMode(rev::spark::SparkBaseConfig::IdleMode::kBrake)
        .VoltageCompensation(12.0)
        .SmartCurrentLimit(20)
        .Follow(m_rightMotor, false);

    m_leftConfig.encoder.PositionConversionFactor(pi2 / elevatorRatio);
    m_rightConfig.closedLoop.Pidf(kaP, kaI, kaD, kaFF)
        .IZone(kaIz)
        .OutputRange(kMinOutput, kMaxOutput);
    m_leftMotor.Configure(m_leftConfig,
                          SparkMax::ResetMode::kResetSafeParameters,
                          SparkMax::PersistMode::kPersistParameters);

    resetEncoders();
}

void Elevator::resetEncoders() {
    m_rightEncoder.SetPosition(0);
    m_leftEncoder.SetPosition(0);
}

double Elevator::getPosition() { return (m_rightEncoder.GetPosition()); }

double Elevator::getTargetPosition() {
    return position;
}

void Elevator::setPosition(double pose) {
    commandGiven = true;
    position = pose;
}

double Elevator::getPresetOffset() {
    return presetOffset;
}

void Elevator::changePresetOffset(double offsetChange) {
    if (presetOffset < -2 || presetOffset > 5) return;

    presetOffset += offsetChange;
}

void Elevator::levelTwo() {
    commandGiven = true;
    position = levelTwoHeight + presetOffset;
}

void Elevator::levelThree() {
    commandGiven = true;
    position = levelThreeHeight + presetOffset;
}

void Elevator::levelFour() {
    commandGiven = true;
    position = levelFourthHeight + presetOffset;
}

void Elevator::sourcePos() {
    commandGiven = true;
    position = sourceIntakeHeight + presetOffset;
}

void Elevator::groundAlgae() {
    commandGiven = true;
    position = groundAlgaeHeight + presetOffset;
}

void Elevator::firstAlgae() {
    commandGiven = true;
    position = firstAlgaeHeight + presetOffset;
}

void Elevator::secondAlgae() {
    commandGiven = true;
    position = secondAlgaeHeight + presetOffset;
}

void Elevator::processor() {
    commandGiven = true;
    position = processorHeight + presetOffset;
}

bool Elevator::closeEnough(void) {
    return fabs(getPosition() - position) /
               ElevatorConstants::levelFourthHeight <
           0.1;
}

void Elevator::Periodic() {
    bool curLimitSwitch = m_limitSwitch.Get();
    if (curLimitSwitch && !prevLimitSwitch) {
        // resetEncoders();
    }

    double elevatorSensorDistance =
        m_distanceSensor.GetAverageVoltage() / 3.3 * 4000 - 5;
    double converted =
        elevatorSensorDistance * pi2 * kFactor * elevatorRatio / 25.4;
    if (elevatorSensorDistance <= 100 && lastDistance > 100) {
        // m_rightEncoder.SetPosition(converted - 0.35);
        // reset = true;
    }
    frc::SmartDashboard::PutBoolean("sensor reset", reset);
    lastDistance = elevatorSensorDistance;

    // SmartDashboard::PutBoolean("useDistance", useDistance);

    units::meter_t ffP{position};
    units::meters_per_second_t ffV{0};
    units::meters_per_second_squared_t ffA(0);
    if (commandGiven) {
        m_rightController.SetReference(
            position, rev::spark::SparkLowLevel::ControlType::kPosition,
            rev::spark::kSlot0, m_elevatorFF.Calculate(ffV, ffA).value());
    } else {
        m_rightController.SetReference(
            m_elevatorFF.Calculate(ffV, ffA).value(),
            rev::spark::SparkLowLevel::ControlType::kVoltage);
    }

    SmartDashboard::PutNumber("elevator preset offset", presetOffset);
    SmartDashboard::PutNumber("converted distance", converted);
    SmartDashboard::PutBoolean("limit switch pressed", m_limitSwitch.Get());
    SmartDashboard::PutNumber("elevator target pos", position);
    SmartDashboard::PutNumber("Elevator Speed", m_leftMotor.Get());
    SmartDashboard::PutNumber("elevator neo pos", m_rightEncoder.GetPosition());

    SmartDashboard::PutNumber("elevator current",
                              m_rightMotor.GetOutputCurrent());
    SmartDashboard::PutNumber("elevator distance", elevatorSensorDistance);
    SmartDashboard::PutNumber("elevator sensor",
                              m_distanceSensor.GetAverageVoltage());
    SmartDashboard::PutNumber("sensor Value", m_distanceSensor.GetValue());
}
void Elevator::SimulationPeriodic() {
    // Implementation of subsystem simulation periodic method goes here.
}
