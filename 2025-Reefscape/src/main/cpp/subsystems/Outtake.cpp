#include "subsystems/Outtake.h"

#include <frc/DigitalInput.h>
#include <frc/smartdashboard/Mechanism2d.h>
#include <frc/smartdashboard/SmartDashboard.h>

#include <cmath>
#include <iostream>

#include "Constants.h"
#include "commands/AutoIntake.h"

using namespace OuttakeConstants;
using namespace rev;
using namespace std;
using namespace MathConstants;
Outtake::Outtake(int leftMotor, int rightMotor, int beamFront, int beamBack)
    : m_leftOuttakeMotor(leftMotor,
                         rev::spark::SparkMax::MotorType::kBrushless),
      m_rightOuttakeMotor(rightMotor,
                          rev::spark::SparkMax::MotorType::kBrushless),
      m_leftOuttakeConfig(),
      m_rightOuttakeConfig() {
    resetMotor();
    m_leftOuttakeMotor.Configure(m_leftOuttakeConfig,
                                 SparkMax::ResetMode::kResetSafeParameters,
                                 SparkMax::PersistMode::kPersistParameters);
    m_rightOuttakeMotor.Configure(m_rightOuttakeConfig,
                                  SparkMax::ResetMode::kResetSafeParameters,
                                  SparkMax::PersistMode::kPersistParameters);
}

void Outtake::resetMotor() {
    m_leftOuttakeConfig
        .SetIdleMode(rev::spark::SparkBaseConfig::IdleMode::kBrake)
        .VoltageCompensation(12.0)
        .SmartCurrentLimit(40)
        .Inverted(false);
    m_rightOuttakeConfig
        .SetIdleMode(rev::spark::SparkBaseConfig::IdleMode::kBrake)
        .VoltageCompensation(12.0)
        .SmartCurrentLimit(40)
        .Inverted(true);

    m_leftOuttakeConfig
        .encoder
        // replace intakeRatio
        .PositionConversionFactor(1.0 / pulleyRatio);
    m_rightOuttakeConfig
        .encoder
        // replace intakeRatio
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

    m_leftOuttakeMotor.Configure(m_leftOuttakeConfig,
                                 SparkMax::ResetMode::kResetSafeParameters,
                                 SparkMax::PersistMode::kPersistParameters);
    m_rightOuttakeMotor.Configure(m_rightOuttakeConfig,
                                  SparkMax::ResetMode::kResetSafeParameters,
                                  SparkMax::PersistMode::kPersistParameters);
    // m_encoder.SetPositionConversionFactor(1.0 / intakeRatio);
}

void Outtake::setSpeed(double speed) {
    m_leftOuttakeMotor.Set(speed);
    m_rightOuttakeMotor.Set(speed);
}

void Outtake::setLeftSpeed(double speed) { m_leftOuttakeMotor.Set(speed); }
void Outtake::setRightSpeed(double speed) { m_rightOuttakeMotor.Set(speed); }

double Outtake::getSpeed() { return m_leftEncoder.GetVelocity(); }

bool Outtake::getBeamFront() { return beamFront.Get(); }
bool Outtake::getBeamBack() { return beamBack.Get(); }

bool Outtake::getCoralHeld() { return coralHeld; }

void Outtake::Periodic() {
    bool bFront = beamFront.Get();
    bool bBack = beamBack.Get();

    if (bFront && !bBack) {
        coralHeld = true;
        // m_LEDs.setColor(0.65);
    } else {
        coralHeld = false;
        // m_LEDs.setColor(0.77);
    }

    frc::SmartDashboard::PutBoolean("beamFront", bFront);
    frc::SmartDashboard::PutBoolean("beamBack", bBack);
    frc::SmartDashboard::PutBoolean("holding coral", coralHeld);

    frc::SmartDashboard::PutNumber("Left Speed", m_leftOuttakeMotor.Get());
    frc::SmartDashboard::PutNumber("Right Speed", m_rightOuttakeMotor.Get());
}
