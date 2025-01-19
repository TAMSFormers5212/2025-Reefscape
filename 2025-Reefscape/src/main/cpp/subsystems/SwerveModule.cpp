#include "subsystems/SwerveModule.h"

#include <frc/smartdashboard/Mechanism2d.h>
#include <frc/smartdashboard/SmartDashboard.h>

#include <cmath>
#include <iostream>

using namespace SwerveModuleConstants;
using namespace rev;
using namespace std;
using namespace MathConstants;

SwerveModule::SwerveModule(int driveMotor, int steerMotor, int absEncoder, double offset)
    : encoderOffset(offset),
      m_driveMotor(driveMotor, rev::spark::SparkMax::MotorType::kBrushless),
      m_steerMotor(steerMotor, rev::spark::SparkMax::MotorType::kBrushless),
      m_driveConfig(),
      m_steerConfig(),
      m_absoluteEncoder(absEncoder),
      m_moduleName(getName(driveMotor)) {

    // m_absoluteEncoder.SetPositionOffset(encoderOffset);
    resetModule();
    
    m_driveMotor.Configure(m_driveConfig, SparkBase::ResetMode::kResetSafeParameters, SparkBase::PersistMode::kPersistParameters);
    m_steerMotor.Configure(m_steerConfig, SparkBase::ResetMode::kResetSafeParameters, SparkBase::PersistMode::kPersistParameters);
    // tony: im not quite sure of the behavior of this function, but it sounds
    // like it just offsets the value returned by get absolute position
    cout << "Swerve Module " << getName(driveMotor) << " initialized correctly" << endl;
}

frc::SwerveModuleState SwerveModule::getState() {  // return current module state
    
    return {units::meters_per_second_t{m_driveEncoder.GetVelocity()}, units::radian_t{getAbsolutePosition()}};
}

frc::SwerveModulePosition SwerveModule::getPosition() {  // get encoder positions
    return {units::meter_t{m_driveEncoder.GetPosition()}, units::radian_t{getAbsolutePosition()}};
}

void SwerveModule::resetModule() {  // resets the drive and steer motors
    resetDriveMotor();
    resetSteerMotor();
}

void SwerveModule::resetDriveMotor() {  // sets pid, current limit, and encoder
            
                                // conversion factor to set values
    m_driveConfig.closedLoop
        .Pidf(kdP, kdI, kdD, kdFF);
    // m_driveMotor.RestoreFactoryDefaults();

    // m_driveController.SetP(kdP);
    // m_driveController.SetI(kdI);
    // m_driveController.SetD(kdD);
    // m_driveController.SetFF(kdFF);
    m_driveConfig
        .SetIdleMode(SparkBaseConfig::IdleMode::kBrake)
        .VoltageCompensation(12.0)
        .SmartCurrentLimit(40, 60);

    m_driveConfig.encoder
        .PositionConversionFactor((SwerveModuleConstants::wheelCircumfrenceMeters / 1.0376 / (SwerveModuleConstants::driveRatio)).value())
        .VelocityConversionFactor((SwerveModuleConstants::wheelCircumfrenceMeters / 1.0376 / SwerveModuleConstants::driveRatio / 60_s).value());
    //currently in inches per second
    

    resetDriveEncoder();
    m_driveMotor.Configure(m_driveConfig, SparkBase::ResetMode::kResetSafeParameters, SparkBase::PersistMode::kPersistParameters);
    m_steerMotor.Configure(m_steerConfig, SparkBase::ResetMode::kResetSafeParameters, SparkBase::PersistMode::kPersistParameters);
}

void SwerveModule::resetSteerMotor() {  // sets pid, current limit, encoder position, and encoder conversion factor to set values
    // m_steerMotor.RestoreFactoryDefaults();
    m_steerConfig.closedLoop
        .Pidf(ktP, kdI, kdD, kdFF)
        
        .PositionWrappingEnabled(true)
        .PositionWrappingMaxInput(pi)
        .PositionWrappingMinInput(-pi);
    
    // m_steerController.SetP(ktP);
    // m_steerController.SetI(ktI);
    // m_steerController.SetD(ktD);
    // m_steerController.SetFF(ktFF);
    // m_steerController.SetPositionPIDWrappingEnabled(true);
    // m_steerController.SetPositionPIDWrappingMaxInput(pi2);
    // m_steerController.SetPositionPIDWrappingMinInput(0);
    m_steerConfig
        .SetIdleMode(SparkBaseConfig::IdleMode::kBrake)
        .VoltageCompensation(12.0)
        .SmartCurrentLimit(20, 30);
    // m_steerMotor.SetIdleMode(CANSparkBase::IdleMode::kBrake);
    // m_steerMotor.EnableVoltageCompensation(12.0);
    // m_steerMotor.SetSmartCurrentLimit(20, 30);

    //m_steerMotor.SetInverted(true);
    m_steerEncoder.SetPosition(m_steerEncoder.GetPosition());
    m_steerConfig.encoder
        
        .PositionConversionFactor(pi2 / (SwerveModuleConstants::steerRatio));
        
        
    
    // m_steerEncoder.SetPositionConversionFactor(pi2 / SwerveModuleConstants::steerRatio);

    resetSteerEncoder();
    m_driveMotor.Configure(m_driveConfig, SparkBase::ResetMode::kResetSafeParameters, SparkBase::PersistMode::kPersistParameters);
    m_steerMotor.Configure(m_steerConfig, SparkBase::ResetMode::kResetSafeParameters, SparkBase::PersistMode::kPersistParameters);
}

void SwerveModule::resetDriveEncoder() {  // set drive encoder to 0.0
    m_driveEncoder.SetPosition(0.0);
}

void SwerveModule::resetSteerEncoder() {  // sets relative steer encoder to absolute encoder position
    
    m_steerEncoder.SetPosition(getAbsolutePosition());
    // m_steerController.
}

double SwerveModule::getAbsolutePosition() {                                                                // returns the absolute encoder position
    // return (m_absoluteEncoder.GetAbsolutePosition() - m_absoluteEncoder.GetPositionOffset()) * pi2;
    double x = (m_absoluteEncoder.Get() - encoderOffset);
    if(x < 0) {
        x += 1;
    }

    return (x * pi2);  // shouldn't need to add an offset value because position offset was set in constructor
}

double SwerveModule::getDrivePosition() {  // returns the drive encoder position
    return m_driveEncoder.GetPosition();
}

double SwerveModule::getSteerPosition() {  // returns the relative steer encoder position
    return m_steerEncoder.GetPosition() + 0.25;
    
}

double SwerveModule::getDriveVelocity() {  // returns the drive encoder velocity
    return m_driveEncoder.GetVelocity();
}

std::string SwerveModule::getName(
    int driveMotorID) {  // returns the module name (top left, topright, bottomleft, bottom right)
    // return this->m_moduleName;
    if (driveMotorID == topleft::driveMotor) {
        return "top left";
    } else if (driveMotorID == topright::driveMotor) {
        return "top right";
    } else if (driveMotorID == bottomleft::driveMotor) {
        return "bottom left";
    } else if (driveMotorID == bottomright::driveMotor) {
        return "bottom right";
    } else {
        return "" + driveMotorID;
    }
}

void SwerveModule::setState(const frc::SwerveModuleState state) {  // sets the module to given state
    // frc::SwerveModuleState optimizedState = frc::SwerveModuleState::Optimize(state, units::radian_t(getSteerPosition() + 0.25));
    // frc::SwerveModuleState stateNew = state;
    // if(state.angle.Degrees().value() < 0) {
        
    //     // state.angle.Degrees() = state.angle.Degrees() + units::degree_t(180);
    //      stateNew = frc::SwerveModuleState{units::velocity::meters_per_second_t{state.speed.value()}, frc::Rotation2d(state.angle.Degrees() + units::degree_t(360))};
    // }
    // else {
    //     stateNew = frc::SwerveModuleState{units::velocity::meters_per_second_t{state.speed.value()}, frc::Rotation2d(state.angle.Degrees())};
    // }
    
    frc::SwerveModuleState optimizedState = state;
    optimizedState.Optimize(units::radian_t(getAbsolutePosition()));
    optimizedState.CosineScale(units::radian_t(getAbsolutePosition()));
    frc::Rotation2d curAngle = units::radian_t(getAbsolutePosition());
    frc::SmartDashboard::PutNumber("STATE SPEED " + getName(m_driveMotor.GetDeviceId()), optimizedState.speed.value());
    frc::SmartDashboard::PutNumber("current " + getName(m_driveMotor.GetDeviceId( )), curAngle.Degrees().value());
    frc::SmartDashboard::PutNumber("absolute " + getName(m_driveMotor.GetDeviceId()), getAbsolutePosition());
   frc::SmartDashboard::PutNumber("ABS ENCODER " + getName(m_driveMotor.GetDeviceId()), m_absoluteEncoder.Get() - encoderOffset);
    // frc::SmartDashboard::PutNumber("STEER ENCODER " + getName(m_driveMotor.GetDeviceId()), getSteerPosition());
    m_steerController.SetReference((optimizedState.angle.Radians().value() * 7/9), rev::spark::SparkBase::ControlType::kPosition);
    // frc::SmartDashboard::PutNumber("encoder " + getName(m_driveMotor.GetDeviceId()), adjustedAngle);
    m_driveController.SetReference(optimizedState.speed.value(), rev::spark::SparkBase::ControlType::kVelocity);
    // idk i took this from 2363. heres what they said:
    //  Since we use relative encoder of steer motor, it is a field (doesn't
    //  wrap from 2pi to 0 for example). We need to calculate delta to avoid
    //  taking a longer route This is analagous to the EnableContinuousInput() 
    //  function of WPILib's PIDController classes

    // curAngle = frc::Rotation2d(curAngle.Degrees() + units::angle::degree_t{90});

    // frc::SmartDashboard::PutNumber("CUR ANGLE " + getName(m_driveMotor.GetDeviceId()), curAngle.Degrees().value());
    // double delta = std::fmod(std::fmod((optimizedState.angle.Radians().value() - curAngle.Radians().value() + pi), pi2) + pi2, pi2) - (pi);  // NOLINT
    // double temp = optimizedState.angle.Radians().value();
    // if(temp < 0) {
    //     temp += pi;
    // }
    frc::SmartDashboard::PutNumber("OBS ANGLE " + getName(m_driveMotor.GetDeviceId()), optimizedState.angle.Degrees().value());
    // double adjustedAngle = temp + curAngle.Radians().value();
    //
    
    // However, I used setPositionPIDWrappingEnabled(), so I don't think this is needed

    // double adjustedAngle = optimizedState.angle.Radians().value();
    // adjustedAngle = optimizedState.angle.Degrees().value()/360;
    // double adjustedPosition = optimizedState.angle.Degrees().value()/360; // turns it into a circle fraction

    // angle we want to go to
    // frc::SmartDashboard::PutNumber("O " + getName(m_driveMotor.GetDeviceId()), delta);
    
    if (getName(m_driveMotor.GetDeviceId()) == "top left"){ // put the motor encoder velocity           
        //std::cout << getName(m_driveMotor.GetDeviceId()) << " " <<optimizedState.speed.value() << " " << m_driveMotor.GetAppliedOutput() << " " << m_driveMotor.GetOutputCurrent() << " " << m_driveEncoder.GetVelocity() << endl;
    }
}

void SwerveModule::Periodic() {
    // frc::SmartDashboard::PutNumber("velocity " + getName(m_driveMotor.GetDeviceId()), abs(getDriveVelocity() / 12));
    // current angle based on the neo encoder
    frc::SmartDashboard::PutNumber("steer num" + getName(m_driveMotor.GetDeviceId()), units::radian_t{getSteerPosition()}.value());
    frc::SmartDashboard::PutNumber("angle " + getName(m_driveMotor.GetDeviceId()), getSteerPosition());
    // print the absolute encoder reading
    frc::SmartDashboard::PutNumber(getName(m_driveMotor.GetDeviceId()) + " degrees", getAbsolutePosition() / 3.14 * 180);
    // this is the absolute encoder reading minus the position offset
    // frc::SmartDashboard::PutNumber(getName(m_driveMotor.GetDeviceId()) + " o abs", getAbsolutePosition()/pi2);    
    frc::SmartDashboard::PutNumber("ABS ENCODER " + getName(m_driveMotor.GetDeviceId()), m_absoluteEncoder.Get() - encoderOffset);
    frc::SmartDashboard::PutNumber("STEER ENCODER " + getName(m_driveMotor.GetDeviceId()), getSteerPosition());
    // frc::SmartDashboard::PutNumber(getName(m_driveMotor.GetDeviceId()) + "off", m_absoluteEncoder.GetPositionOffset());
}

void SwerveModule::togglePositionOffset(bool toggleOffset) {  // turns on or off the absolute encoder position offset (TESTING ONLY, DO NOT USE IN COMPETITION)
    if (toggleOffset) {
        // m_absoluteEncoder.SetPositionOffset(this->encoderOffset);
    } else {
        // m_absoluteEncoder.SetPositionOffset(0);
        
    }
}