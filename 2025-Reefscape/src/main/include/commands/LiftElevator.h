#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>

#include "subsystems/Elevator.h"

namespace cmds {

class LiftElevator : public frc2::CommandHelper<frc2::Command, LiftElevator> {
   public:
    void Initialize() override;

    void End(bool interrupted) override;

    explicit LiftElevator(Elevator* elevator);

   private:
    Elevator* m_elevator;
};

}  // namespace cmds