
#pragma once

#include <Constants.h>
#include <frc/AnalogEncoder.h>
#include <frc/DigitalInput.h>
#include <frc/controller/PIDController.h>
#include <frc/controller/ProfiledPIDController.h>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>
#include <rev/SparkClosedLoopController.h>
#include <rev/SparkMax.h>
#include <rev/SparkRelativeEncoder.h>
#include <rev/config/SparkMaxConfig.h>

using namespace std;
using namespace rev::spark;
//Goodbye code, we had a good run-sameer

class Outtake : public frc2::SubsystemBase {
   public:
    Outtake(int leftMotor, int rightMotor, int beamFront, int beamBack);
    void resetMotor();
    double getSpeed();
    void setSpeed(double speed);
    void setLeftSpeed(double speed);
    void setRightSpeed(double speed);

    void Periodic() override;

    bool getCoralHeld(void);
    bool getBeamFront(void);
    bool getBeamBack(void);

   private:
    bool coralHeld = false;

    SparkMax m_leftOuttakeMotor;
    SparkMaxConfig m_leftOuttakeConfig;
    SparkRelativeEncoder m_leftEncoder = m_leftOuttakeMotor.GetEncoder();
    SparkClosedLoopController m_leftOuttakeController =
        m_leftOuttakeMotor.GetClosedLoopController();

    SparkMax m_rightOuttakeMotor;
    SparkMaxConfig m_rightOuttakeConfig;
    SparkRelativeEncoder m_rightEncoder = m_rightOuttakeMotor.GetEncoder();
    SparkClosedLoopController m_rightOuttakeController =
        m_rightOuttakeMotor.GetClosedLoopController();

    frc::DigitalInput beamFront{OuttakeConstants::beamFront};
    frc::DigitalInput beamBack{OuttakeConstants::beamBack};
};
