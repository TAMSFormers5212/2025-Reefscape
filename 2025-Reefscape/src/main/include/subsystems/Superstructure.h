// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>

#include <frc/controller/PIDController.h>
#include <frc/controller/ProfiledPIDController.h>

#include <frc/AnalogEncoder.h>

#include <rev/SparkMax.h>
#include <rev/SparkRelativeEncoder.h>
#include <rev/SparkClosedLoopController.h>
#include <vector>
#include <Constants.h>
#include "Intake.h"

using namespace std;
using namespace PoseConstants;
class Superstructure : public frc2::SubsystemBase {
 public:
  Superstructure();

    void resetSuperstructure();
  /**
   * Example command factory method.
   */
//   frc2::CommandPtr ExampleMethodCommand();

  /**
   * An example method querying a boolean state of the subsystem (for example, a
   * digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

  /**
   * Will be called periodically whenever the CommandScheduler runs during
   * simulation.
   */
  void SimulationPeriodic() override;
    Intake m_intake;
 private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
};
