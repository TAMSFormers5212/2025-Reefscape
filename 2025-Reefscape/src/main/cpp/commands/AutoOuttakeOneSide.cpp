#include "commands/AutoOuttakeOneSide.h"


AutoOuttakeOneSide::AutoOuttakeOneSide(Outtake* outtake) : outtake(outtake) {
    AddRequirements(outtake);
}

void AutoOuttakeOneSide::Initialize() {
    outtake->setLeftSpeed(0.2);
}

void AutoOuttakeOneSide::Execute() {
}

bool AutoOuttakeOneSide::IsFinished() {
    return !outtake->getBeamFront();
}

void AutoOuttakeOneSide::End(bool interrupted) {
    outtake->setLeftSpeed(0);
}