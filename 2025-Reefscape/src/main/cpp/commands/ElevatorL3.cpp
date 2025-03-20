#include "commands/ElevatorL3.h"

ElevatorL3::ElevatorL3(Elevator* elevator) : m_elevator(elevator) {
    AddRequirements(elevator);
}

void ElevatorL3::Initialize() {
    m_elevator->levelThree();
}

bool ElevatorL3::IsFinished(void) {
    return m_elevator->closeEnough();
}
