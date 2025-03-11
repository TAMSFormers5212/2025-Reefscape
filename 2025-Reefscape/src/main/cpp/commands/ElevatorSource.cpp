#include "commands/ElevatorSource.h"

ElevatorSource::ElevatorSource(Elevator* elevator) : m_elevator(elevator) {
    AddRequirements(elevator);
}

void ElevatorSource::Initialize() {
    // m_outtake->setRightSpeed(0.2);
    m_elevator->sourcePos();
}


void ElevatorSource::End(bool interrupted) {}