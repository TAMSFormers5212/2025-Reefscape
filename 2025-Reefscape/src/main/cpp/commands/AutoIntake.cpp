#include "commands/AutoIntake.h"


AutoIntake::AutoIntake(Outtake* outtake) : outtake(outtake) {
    AddRequirements(outtake);
}

void AutoIntake::Initialize() {
    outtake->setSpeed(0.3);
}

void AutoIntake::Execute() {
    if (outtake->getBeamFront() && !outtake->getBeamBack()) {
        isFinished = true;
        outtake->setSpeed(0.0);
    }  else if (outtake->getBeamBack() && !isFinished) {
        outtake->setSpeed(0.04);
    }
}

bool AutoIntake::IsFinished() {
    return isFinished;
}