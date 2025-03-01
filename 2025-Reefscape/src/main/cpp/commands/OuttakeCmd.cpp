#include "commands/OuttakeCmd.h"

OuttakeCmd::OuttakeCmd(Outtake* outtake) : m_outtake(outtake) {
    AddRequirements(outtake);
}

void OuttakeCmd::Initialize() {
    // m_outtake->setRightSpeed(0.2);
    m_outtake->setLeftSpeed(0.2);
}


void OuttakeCmd::End(bool interrupted) {}