// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>
#include <rev/SparkMax.h>

#include <frc/controller/PIDController.h>
#include <frc/controller/ProfiledPIDController.h>

#include <frc/AnalogEncoder.h>
#include <rev/config/SparkMaxConfig.h>
#include <rev/SparkRelativeEncoder.h>
#include <rev/SparkClosedLoopController.h>  
#include <Constants.h>
#include <frc/DigitalInput.h>
using namespace std;
using namespace rev::spark;
class Outtake: public frc2::SubsystemBase {
 public:
  Outtake(int leftMotor, int rightMotor, int beamFront, int beamBack);
void resetMotor();
double getSpeed();
void intakeCoral();
void stopOuttake();
void autoIntake();
void setSpeed(double speed);
void setLeftSpeed(double speed);
void setRightSpeed(double speed);
void Periodic() override;

bool getCoralHeld();
bool getBeamFront(void);
bool getBeamBack(void);
  /**
   * Example command factory method.
   */

  /**
   * An example method querying a boolean state of the subsystem (for example, a
   * digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */

  /**
   * Will be called periodically whenever the CommandScheduler runs during
   * simulation.
   */
    //void SimulationPeriodic() override;
 private:
    bool coralHeld = false;

    SparkMax m_leftOuttakeMotor; // may need to switch to 775 if neo550 is not fixed in time

    SparkMaxConfig m_leftOuttakeConfig;

    SparkRelativeEncoder m_leftEncoder = m_leftOuttakeMotor.GetEncoder();

    SparkClosedLoopController m_leftOuttakeController = m_leftOuttakeMotor.GetClosedLoopController();
    SparkMax m_rightOuttakeMotor; // may need to switch to 775 if neo550 is not fixed in time

    SparkMaxConfig m_rightOuttakeConfig;

    SparkRelativeEncoder m_rightEncoder = m_rightOuttakeMotor.GetEncoder();

    SparkClosedLoopController m_rightOuttakeController = m_rightOuttakeMotor.GetClosedLoopController();
    frc::DigitalInput beamFront{OuttakeConstants::beamFront};
    frc::DigitalInput beamBack{OuttakeConstants::beamBack};
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
};
