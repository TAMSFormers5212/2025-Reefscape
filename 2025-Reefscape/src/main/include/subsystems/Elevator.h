// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <Constants.h>
#include <frc/AnalogEncoder.h>
#include <frc/Encoder.h>
#include <frc/controller/ElevatorFeedforward.h>
#include <frc/controller/PIDController.h>
#include <frc/controller/ProfiledPIDController.h>
#include <frc/controller/SimpleMotorFeedforward.h>
#include <frc/trajectory/TrapezoidProfile.h>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>
#include <rev/SparkClosedLoopController.h>
#include <rev/SparkMax.h>
#include <frc/DigitalInput.h>
#include <rev/SparkRelativeEncoder.h>
#include <rev/config/SparkMaxConfig.h>
#include <units/voltage.h>
using namespace rev::spark;
using namespace frc;
using namespace ElevatorConstants;

class Elevator : public frc2::SubsystemBase {
   public:
    Elevator(int leftMotor, int rightMotor, int encoderOne, int encoderTwo, double encoderOffset);
    void setPosition(double elevatorPose);

    double getPosition();
    double getRelativePosition();
    void setSpeed(double speed);
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

    bool closeEnough(void);

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

    SparkClosedLoopController m_leftController =
        m_leftMotor.GetClosedLoopController();  // leader
    SparkClosedLoopController m_rightController =
        m_rightMotor.GetClosedLoopController();  // follower

    Encoder m_absoluteEncoder{encoderOne, encoderTwo};
    double position = 0.0;
    ElevatorFeedforward m_elevatorFF;
    bool commandGiven = false;
    frc::DigitalInput m_limitSwitch{limitSwitch};
    // Components (e.g. motor controllers and sensors) should generally be
    // declared private and exposed only through public methods.
};
