#pragma once

#include <frc2/command/SubsystemBase.h>
#include <frc/DutyCycleEncoder.h>
#include <frc/controller/PIDController.h>
#include <frc/controller/ProfiledPIDController.h>
#include <frc/controller/SimpleMotorFeedforward.h>
#include <frc/controller/ArmFeedForward.h>
#include <frc/trajectory/TrapezoidProfile.h>
#include <units/voltage.h>




#include <frc/AnalogEncoder.h>


#include <Constants.h>
#include <frc/motorcontrol/Spark.h>
//Goodbye code, we had a good run-sameer

// using namespace rev;
using namespace frc;


class LEDController : public frc2::SubsystemBase{
    private:
        double m_color = 0.0;
        Spark m_ledController;
    public:
      LEDController();
      void Periodic() override;
      void setColor(double d);


};