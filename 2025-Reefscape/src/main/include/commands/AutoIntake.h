#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include <subsystems/Outtake.h>

#include "subsystems/SwerveDrive.h"

class AutoIntake : public frc2::CommandHelper<frc2::Command, AutoIntake> {
   public:
    explicit AutoIntake(Outtake* outtake);

    void Initialize() override;
    void Execute() override;

    bool IsFinished() override;

   private:
    Outtake* outtake;

    bool isFinished = false;
};
