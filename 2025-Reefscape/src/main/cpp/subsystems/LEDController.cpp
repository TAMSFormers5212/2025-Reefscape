#include "subsystems/LEDController.h"

#include <frc/smartdashboard/Mechanism2d.h>
#include <frc/smartdashboard/SmartDashboard.h>

#include <cmath>
#include <iostream>
#include <frc/motorcontrol/Spark.h>


// using namespace rev;
using namespace std;

LEDController::LEDController() : m_ledController(0) {
    m_color = 0.77;
    m_ledController.Set(0.77);
}

void LEDController::Periodic() {
    m_ledController.Set(m_color);
}
void LEDController::setColor(double d) {
    m_color = d;
}
