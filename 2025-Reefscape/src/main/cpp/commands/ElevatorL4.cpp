#include "commands/ElevatorL4.h"

ElevatorL4::ElevatorL4(Elevator* elevator) : m_elevator(elevator) {
    AddRequirements(elevator);
}

void ElevatorL4::Initialize() {
    // m_outtake->setRightSpeed(0.2);
    m_elevator->levelFour();
}


void ElevatorL4::End(bool interrupted) {}