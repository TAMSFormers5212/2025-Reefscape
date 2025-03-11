#include "commands/ElevatorL2.h"

ElevatorL2::ElevatorL2(Elevator* elevator) : m_elevator(elevator) {
    AddRequirements(elevator);
}

void ElevatorL2::Initialize() {
    // m_outtake->setRightSpeed(0.2);
    m_elevator->levelTwo();
}


void ElevatorL2::End(bool interrupted) {}