// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>
#include <frc/DutyCycleEncoder.h>
#include <frc/controller/PIDController.h>
#include <frc/controller/ProfiledPIDController.h>
#include <frc/controller/SimpleMotorFeedforward.h>
#include <frc/controller/ElevatorFeedforward.h>
#include <frc/trajectory/TrapezoidProfile.h>
#include <units/voltage.h>

#include <rev/SparkMax.h>
#include <rev/config/SparkMaxConfig.h>
#include <rev/SparkRelativeEncoder.h>
#include <rev/SparkClosedLoopController.h>
#include <frc/AnalogEncoder.h>


#include <Constants.h>
using namespace rev::spark;
using namespace frc;
using namespace ElevatorConstants;

class Elevator : public frc2::SubsystemBase {
 public:
  Elevator(int leftMotor, int rightMotor, int encoder, double encoderOffset);
    void setPosition(double elevatorPose);

    double getPosition();
    double getRelativePosition();
    void setSpeed(int speed);
    void resetMotors();
    void resetEncoders();
    void levelOne();
    void levelTwo();
    void levelThree();
    void levelFour();
    void sourcePos();
    void groundAlgae();
    void firstAlgae();
    void secondAlgae();
    void processor();
    

  void Periodic() override;

  /**
   * Will be called periodically whenever the CommandScheduler runs during
   * simulation.
   */
  void SimulationPeriodic() override;
 double initialPosition = 0.0;
 private:
    SparkMax m_leftMotor;
    SparkMax m_rightMotor;
    SparkMaxConfig m_leftConfig;
    SparkMaxConfig m_rightConfig;

    SparkRelativeEncoder m_leftEncoder = m_leftMotor.GetEncoder();
    SparkRelativeEncoder m_rightEncoder = m_rightMotor.GetEncoder();

     SparkClosedLoopController m_leftController = m_leftMotor.GetClosedLoopController(); // leader
    SparkClosedLoopController m_rightController = m_rightMotor.GetClosedLoopController(); // follower

    DutyCycleEncoder m_absoluteEncoder{encoder};
    double position = 0.0;
    ElevatorFeedforward m_elevatorFF;
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
};
