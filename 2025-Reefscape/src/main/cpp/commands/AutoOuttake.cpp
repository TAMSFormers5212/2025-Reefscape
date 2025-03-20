#include "commands/AutoOuttake.h"


AutoOuttake::AutoOuttake(Outtake* outtake) : outtake(outtake) {
    AddRequirements(outtake);
}

void AutoOuttake::Initialize() {
    outtake->setSpeed(0.2);
}

void AutoOuttake::Execute() {
    if (!outtake->getBeamFront()) {
        isFinished = true;
        outtake->setSpeed(0.0);
    }
}

bool AutoOuttake::IsFinished() {
    return isFinished;
}