#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include <subsystems/Elevator.h>
#include <subsystems/Outtake.h>

#include "subsystems/SwerveDrive.h"

class ElevatorL2 : public frc2::CommandHelper<frc2::Command, ElevatorL2> {
   public:
    explicit ElevatorL2(Elevator* elevator);

    void Initialize() override;
    bool IsFinished(void) override;

   private:
    Elevator* m_elevator;
};