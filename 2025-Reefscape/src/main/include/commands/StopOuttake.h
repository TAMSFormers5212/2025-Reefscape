#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>

#include "subsystems/SwerveDrive.h"
#include <subsystems/Outtake.h>

#include <subsystems/Intake.h>
class StopOuttake : public frc2::CommandHelper<frc2::Command, StopOuttake> {
public:
    explicit StopOuttake(Outtake* outtake);

    void Initialize() override;

    void End(bool interrupted) override;

private:
    
    Outtake* m_outtake;
};