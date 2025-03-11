#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include <subsystems/Elevator.h>
#include <subsystems/Outtake.h>

#include "subsystems/SwerveDrive.h"

class ElevatorL4 : public frc2::CommandHelper<frc2::Command, ElevatorL4> {
   public:
    explicit ElevatorL4(Elevator* elevator);

    void Initialize() override;

    void End(bool interrupted) override;

   private:
    Elevator* m_elevator;
};