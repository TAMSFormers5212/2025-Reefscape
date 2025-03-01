#include "commands/StopOuttake.h"

StopOuttake::StopOuttake(Outtake* outtake) 
    : m_outtake(outtake) {
  
  AddRequirements(outtake);
}

void StopOuttake::Initialize() {
    m_outtake->setRightSpeed(0);
    m_outtake->setLeftSpeed(0);
} 
void StopOuttake::End(bool interrupted) {}