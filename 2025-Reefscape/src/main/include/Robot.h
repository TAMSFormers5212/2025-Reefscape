// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
//Goodbye code, we had a good run-sameer

#pragma once

#include <frc/TimedRobot.h>
#include <frc2/command/CommandPtr.h>

#include <optional>

#include "RobotContainer.h"

class Robot : public frc::TimedRobot {
   public:
    void RobotInit() override;
    void RobotPeriodic() override;
    void DisabledInit() override;
    void DisabledPeriodic() override;
    void AutonomousInit() override;
    void AutonomousPeriodic() override;
    void TeleopInit() override;
    void TeleopPeriodic() override;
    void TestPeriodic() override;
    void SimulationInit() override;
    void SimulationPeriodic() override;

   private:
    cs::UsbCamera usbCam;
    // Have it empty by default so that if testing teleop it
    // doesn't have undefined behavior and potentially crash.
    std::optional<frc2::Command*> m_autonomousCommand;

    RobotContainer m_container;
};
