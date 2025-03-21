#include "commands/ElevatorL4.h"

ElevatorL4::ElevatorL4(Elevator* elevator) : m_elevator(elevator) {
    AddRequirements(elevator);
}

void ElevatorL4::Initialize() {
    // m_outtake->setRightSpeed(0.2);
    m_elevator->levelFour();
}


bool ElevatorL4::IsFinished(void) {
    return m_elevator->closeEnough();
}