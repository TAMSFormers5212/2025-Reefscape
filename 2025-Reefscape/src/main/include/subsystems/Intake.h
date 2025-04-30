#pragma once

#include <frc/AnalogEncoder.h>
#include <frc/DigitalInput.h>
#include <frc/DutyCycleEncoder.h>
#include <frc/controller/ArmFeedForward.h>
#include <frc/controller/PIDController.h>
#include <frc/controller/ProfiledPIDController.h>
#include <frc/controller/SimpleMotorFeedforward.h>
#include <frc/trajectory/TrapezoidProfile.h>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>
#include <rev/SparkClosedLoopController.h>
#include <rev/SparkMax.h>
#include <rev/SparkRelativeEncoder.h>
#include <rev/config/SparkMaxConfig.h>

#include "Constants.h"
//Goodbye code, we had a good run-sameer

using namespace std;
using namespace rev::spark;
using namespace IntakeConstants;

class Intake : public frc2::SubsystemBase {
   public:
    Intake(int intakeMotor, int pivotMotor, int encoder, double encoderOffset);
    void resetMotor();
    void resetEncoder();

    void setSpeed(double speed);
    double getSpeed();
    void setPivotSpeed(double speed);
    void setTargetPosition(double pivotPose);
    double getTargetPosition(void);

    double getRelativePosition();
    double getPosition();

    void stowPreset();
    void groundPreset();
    void processorPreset();
    void reefPreset();

    double getOutputCurrent();
    void Periodic() override;
   private:
    SparkMax m_intakeMotor; 
    SparkMaxConfig m_intakeConfig;
    SparkRelativeEncoder m_encoder = m_intakeMotor.GetEncoder();
    SparkClosedLoopController m_intakeController =
        m_intakeMotor.GetClosedLoopController();

    SparkMax m_pivotMotor; 
    SparkMaxConfig m_pivotConfig;
    SparkRelativeEncoder m_pivotEncoder = m_pivotMotor.GetEncoder();
    SparkClosedLoopController m_pivotController =
        m_pivotMotor.GetClosedLoopController();

    frc::DutyCycleEncoder m_absoluteEncoder{pivotEncoder};

    frc::ArmFeedforward m_pivotFF;
    double position = 0.0;

    bool intakeCommandGiven = false;
};
