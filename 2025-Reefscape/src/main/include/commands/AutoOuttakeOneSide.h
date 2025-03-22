#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>

#include "subsystems/Outtake.h"

class AutoOuttakeOneSide : public frc2::CommandHelper<frc2::Command, AutoOuttakeOneSide> {
   public:
    explicit AutoOuttakeOneSide(Outtake* outtake);

    void Initialize() override;
    void Execute() override;
    void End(bool interrupted) override;

    bool IsFinished() override;

   private:
    Outtake* outtake;
};
