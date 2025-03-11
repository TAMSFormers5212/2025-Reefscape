#include "commands/ElevatorL3.h"

ElevatorL3::ElevatorL3(Elevator* elevator) : m_elevator(elevator) {
    AddRequirements(elevator);
}

void ElevatorL3::Initialize() {
    // m_outtake->setRightSpeed(0.2);
    m_elevator->levelThree();
}


void ElevatorL3::End(bool interrupted) {}