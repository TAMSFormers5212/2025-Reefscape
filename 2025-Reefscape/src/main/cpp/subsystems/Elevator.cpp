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

Elevator::Elevator(int leftMotor, int rightMotor, int encoderOne, int encoderTwo,
    double encoderOffset)
    : m_leftMotor(leftMotor, rev::spark::SparkMax::MotorType::kBrushless),
    m_rightMotor(rightMotor, rev::spark::SparkMax::MotorType::kBrushless),
    m_leftConfig(),
    m_rightConfig(),
    m_elevatorFF(ElevatorConstants::kaS, ElevatorConstants::kaG,
        ElevatorConstants::kaV) {
    resetMotors();
    initialPosition = getPosition();
    frc::SmartDashboard::PutNumber("elevator init pos", initialPosition);
    // m_leftMotor.Configure(m_leftConfig,
    //                       SparkBase::ResetMode::kResetSafeParameters,
    //                       SparkBase::PersistMode::kPersistParameters);
    // m_rightMotor.Configure(m_rightConfig,
    //                        SparkBase::ResetMode::kResetSafeParameters,
    //                        SparkBase::PersistMode::kPersistParameters);
    // Implementation of subsystem constructor goes here.
}

void Elevator::resetMotors() {
    //  m_leftConfig.closedLoop.Pidf(kaP, kaI, kaD, kaFF)
    //     .IZone(kaIz)
    //     .OutputRange(kMinOutput, kMaxOutput);
    m_rightConfig.SetIdleMode(rev::spark::SparkBaseConfig::IdleMode::kBrake)
        .VoltageCompensation(12.0)
        .SmartCurrentLimit(40);
    m_rightConfig.encoder.PositionConversionFactor(pi2 / elevatorRatio);
    m_rightMotor.Configure(m_rightConfig,
        SparkMax::ResetMode::kResetSafeParameters,
        SparkMax::PersistMode::kPersistParameters);

    m_leftConfig.SetIdleMode(rev::spark::SparkBaseConfig::IdleMode::kBrake)
        .VoltageCompensation(12.0)
        .SmartCurrentLimit(40)
        .Follow(m_rightMotor, false);

    m_leftConfig.encoder.PositionConversionFactor(pi2 / elevatorRatio);
    // m_rightConfig.closedLoop.Pidf(kaP, kaI, kaD, kaFF)
        // .IZone(kaIz)
        // .OutputRange(kMinOutput, kMaxOutput);
    m_leftMotor.Configure(m_leftConfig,
        SparkMax::ResetMode::kResetSafeParameters,
        SparkMax::PersistMode::kPersistParameters);
        
    resetEncoders();
    // Implementation of subsystem periodic method goes here.
}

void Elevator::resetEncoders() {
    m_rightEncoder.SetPosition(0);
    m_leftEncoder.SetPosition(0);
    initialPosition = getPosition();
}

void Elevator::setSpeed(double speed) {
    m_leftMotor.Set(speed);
    m_rightMotor.Set(speed);
}

double Elevator::getPosition() {  // returns the absolute encoder position with offset
    // return abs(m_absoluteEncoder.GetAbsolutePosition()-0.75)*pi2;
    // return abs(m_absoluteEncoder.Get() - elevatorOffset) * pi2;
    return m_rightEncoder.GetPosition();
}
double Elevator::getRelativePosition() { return m_rightEncoder.GetPosition(); }

void Elevator::levelOne() { position = levelOneHeight; }
void Elevator::levelTwo() { position = levelTwoHeight; }
void Elevator::levelThree() { position = levelThreeHeight; }
void Elevator::levelFour() { position = levelFourthHeight; }
void Elevator::sourcePos() { position = sourceIntakeHeight; }
void Elevator::groundAlgae() { position = groundAlgaeHeight; }
void Elevator::firstAlgae() { position = firstAlgaeHeight; }
void Elevator::secondAlgae() { position = secondAlgaeHeight; }
void Elevator::processor() { position = processorHeight; }

void Elevator::setPosition(
    double pose) {  // sets the goal pose to given parameter
    position = pose;
    // smart motion implementation
    //  m_rightController.SetReference(pose,
    //                                 CANSparkLowLevel::ControlType::kSmartMotion);

    // double ff = -sin((getPosition()-0.5)*MathConstants::pi2)*0.1;
    // m_leftController.SetFF(ff);
    // m_leftController.SetReference(pose,
    // CANSparkLowLevel::ControlType::kPosition);
}
void Elevator::Periodic() {
    units::meter_t ffP{ position };
    units::meters_per_second_t ffV{ 0 };
    units::meters_per_second_squared_t ffA(0);
    // m_rightController.SetReference(
    //     position, rev::spark::SparkLowLevel::ControlType::kPosition,
    //     rev::spark::kSlot0, m_elevatorFF.Calculate(ffV, ffA).value());
    frc::SmartDashboard::PutNumber("elevator abs pos", m_absoluteEncoder.Get());
    frc::SmartDashboard::PutNumber("elevator relative pos", m_rightEncoder.GetPosition());
    frc::SmartDashboard::PutNumber("Elevator Speed", m_leftMotor.Get());
}
void Elevator::SimulationPeriodic() {
    // Implementation of subsystem simulation periodic method goes here.
}
