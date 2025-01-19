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
class Intake: public frc2::SubsystemBase {
 public:
  Intake(int intakeMotor, int pivotMotor);
void resetMotor();
void setSpeed(double speed);
double getSpeed();
void stopIntake();
double getOutputCurrent();
 int getState();
    void setState(int state);
void Periodic() override;
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
    SparkMax m_intakeMotor; // may need to switch to 775 if neo550 is not fixed in time

    SparkMaxConfig m_intakeConfig;
   
    SparkRelativeEncoder m_encoder = m_intakeMotor.GetEncoder();

    SparkClosedLoopController m_intakeController = m_intakeMotor.GetClosedLoopController();
     SparkMax m_pivotMotor; // may need to switch to 775 if neo550 is not fixed in time

    SparkMaxConfig m_pivotConfig;
    SparkRelativeEncoder m_pivotEncoder = m_pivotMotor.GetEncoder();

    SparkClosedLoopController m_pivotController = m_pivotMotor.GetClosedLoopController();
    int state = IntakeConstants::empty;
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
};
