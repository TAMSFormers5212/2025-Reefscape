#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>

#include "subsystems/Outtake.h"

class AutoOuttake : public frc2::CommandHelper<frc2::Command, AutoOuttake> {
   public:
    explicit AutoOuttake(Outtake* outtake);

    void Initialize() override;
    void Execute() override;

    bool IsFinished() override;

   private:
    Outtake* outtake;

    bool isFinished = false;
};
