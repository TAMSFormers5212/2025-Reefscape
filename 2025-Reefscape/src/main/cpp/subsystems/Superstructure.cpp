#include "subsystems/Superstructure.h"
//Goodbye code, we had a good run-sameer

Superstructure::Superstructure()
    : m_intake(IntakeConstants::intakeMotor, IntakeConstants::pivotMotor,
               IntakeConstants::pivotEncoder, IntakeConstants::pivotOffset),
      m_outtake(OuttakeConstants::leftMotor, OuttakeConstants::rightMotor,
                OuttakeConstants::beamFront, OuttakeConstants::beamBack),
      m_elevator(ElevatorConstants::leftMotor, ElevatorConstants::rightMotor,
                 ElevatorConstants::encoderOne, ElevatorConstants::encoderTwo,
                 ElevatorConstants::encoderOffset),
      m_vision() {
    resetSuperstructure();
}

void Superstructure::resetSuperstructure() {
    m_intake.resetMotor();
    m_outtake.resetMotor();
    m_elevator.resetMotors();
}

void Superstructure::algaeGround() {
    m_elevator.groundAlgae();
    m_intake.groundPreset();
}

void Superstructure::algaeProcessor() {
    m_elevator.processor();
    m_intake.processorPreset();
}

void Superstructure::algaeFirst() {
    m_elevator.firstAlgae();
    m_intake.reefPreset();
}

void Superstructure::algaeSecond() {
    m_elevator.secondAlgae();
    m_intake.reefPreset();
}

void Superstructure::Periodic() {}

void Superstructure::SimulationPeriodic() {}
