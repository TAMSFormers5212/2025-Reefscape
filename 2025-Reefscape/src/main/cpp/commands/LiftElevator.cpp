#include "commands/LiftElevator.h"

using namespace cmds;

LiftElevator::LiftElevator(Elevator* elevator) : m_elevator{elevator} {
    // Register that this command requires the subsystem.
    AddRequirements(m_elevator);
}

void LiftElevator::Initialize() {}

void LiftElevator::Execute() {}

void LiftElevator::End(bool interrupted) {}