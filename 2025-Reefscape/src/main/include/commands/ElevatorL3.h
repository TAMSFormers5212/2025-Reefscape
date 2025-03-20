#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include <subsystems/Elevator.h>
#include <subsystems/Outtake.h>

#include "subsystems/SwerveDrive.h"

class ElevatorL3 : public frc2::CommandHelper<frc2::Command, ElevatorL3> {
   public:
    explicit ElevatorL3(Elevator* elevator);

    void Initialize() override;
    bool IsFinished(void) override;

   private:
    Elevator* m_elevator;
};