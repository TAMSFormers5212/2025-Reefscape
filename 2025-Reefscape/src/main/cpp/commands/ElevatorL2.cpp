#include "commands/ElevatorL2.h"

#include "Constants.h"

ElevatorL2::ElevatorL2(Elevator* elevator) : m_elevator(elevator) {
    AddRequirements(elevator);
}

void ElevatorL2::Initialize() {
    m_elevator->levelTwo();
}

bool ElevatorL2::IsFinished() {
    return m_elevator->closeEnough();
}
