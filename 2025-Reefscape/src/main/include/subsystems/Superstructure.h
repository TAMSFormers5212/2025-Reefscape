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
#include "Outtake.h"
#include "Elevator.h"
#include "VisionSubsystem.h"
//Goodbye code, we had a good run-sameer

using namespace std;
using namespace PoseConstants;
class Superstructure : public frc2::SubsystemBase {
 public:
  Superstructure();

    void resetSuperstructure();
    void algaeGround();
    void algaeProcessor();
    void algaeFirst();
    void algaeSecond();
  void Periodic() override;

  /**
   * Will be called periodically whenever the CommandScheduler runs during
   * simulation.
   */
  void SimulationPeriodic() override;
    Intake m_intake;
    Outtake m_outtake;
    Elevator m_elevator;
    VisionSubsystem m_vision; 
 private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
};
