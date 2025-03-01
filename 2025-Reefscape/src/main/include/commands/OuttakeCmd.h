#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include <subsystems/Intake.h>
#include <subsystems/Outtake.h>

#include "subsystems/SwerveDrive.h"

class OuttakeCmd : public frc2::CommandHelper<frc2::Command, OuttakeCmd> {
   public:
    explicit OuttakeCmd(Outtake* outtake);

    void Initialize() override;

    void End(bool interrupted) override;

   private:
    Outtake* m_outtake;
};