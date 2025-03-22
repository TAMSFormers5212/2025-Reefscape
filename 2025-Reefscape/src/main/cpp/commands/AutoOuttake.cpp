#include "commands/AutoOuttake.h"


AutoOuttake::AutoOuttake(Outtake* outtake) : outtake(outtake) {
    AddRequirements(outtake);
}

void AutoOuttake::Initialize() {
    outtake->setSpeed(0.2);
}

void AutoOuttake::Execute() {
}

bool AutoOuttake::IsFinished() {
    return !outtake->getBeamFront();
}

void AutoOuttake::End(bool interrupted) {
    outtake->setSpeed(0);
}