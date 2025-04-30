#pragma once

#include <frc2/command/SubsystemBase.h>

#include <rev/SparkMax.h>
#include <rev/config/SparkMaxConfig.h>
#include <rev/SparkRelativeEncoder.h>
#include <rev/SparkClosedLoopController.h>

#include <frc/kinematics/SwerveModuleState.h>
#include <frc/kinematics/SwerveModulePosition.h>
#include <frc/AnalogEncoder.h>


#include <Constants.h>
//Goodbye code, we had a good run-sameer

using namespace rev::spark;

class SwerveModule : public frc2::SubsystemBase{

public:
    SwerveModule(int driveMotor, int steerMotor, int absEncoder, double offset);

    void resetModule();
    void resetDriveMotor();
    void resetSteerMotor();
    void resetDriveEncoder();
    void resetSteerEncoder();
    double getDrivePosition();
    double getSteerPosition();
    double getDriveVelocity();
    double getAbsolutePosition();
    std::string getName(int driveMotor);
    frc::SwerveModuleState getState();
    frc::SwerveModulePosition getPosition();
    void setState(const frc::SwerveModuleState state);
    void togglePositionOffset(bool toggleOffset);


    void Periodic() override;

private:

    double encoderOffset;

    SparkMax m_driveMotor;
    SparkMax m_steerMotor;

    SparkMaxConfig m_driveConfig;
    SparkMaxConfig m_steerConfig;

    SparkRelativeEncoder m_driveEncoder = m_driveMotor.GetEncoder();
    SparkRelativeEncoder m_steerEncoder = m_steerMotor.GetEncoder();

    SparkClosedLoopController m_driveController = m_driveMotor.GetClosedLoopController();
    SparkClosedLoopController m_steerController = m_steerMotor.GetClosedLoopController();

    frc::AnalogEncoder m_absoluteEncoder;

    std::string m_moduleName;
};
