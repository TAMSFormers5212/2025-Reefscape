#pragma once

#include <frc/controller/PIDController.h>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>

#include <span>

#include "Constants.h"
#include "frc/smartdashboard/Smartdashboard.h"
#include "networktables/NetworkTable.h"
#include "networktables/NetworkTableEntry.h"
#include "networktables/NetworkTableInstance.h"
#include "networktables/NetworkTableValue.h"
#include <cameraserver/CameraServer.h>

using namespace SwerveModuleConstants;

class VisionSubsystem : public frc2::SubsystemBase {
   public:
    VisionSubsystem();

    frc2::CommandPtr VisionMethodCommand();

    bool VisionCondition();

    void Periodic() override;
    void setOutput(double op);
    double getOutput();

    double getX(); // horizontal axis position
    double getY(); // vertical axis position
    double getZ(); // distance axis position
    
    //implement these eventually
    int getID();
    // frc::Pose2d getVisionPose();

    void setDistanceError(double dist_error);
    double getDistanceError();
    double getDistance();
    void setLedOn(int ledsOn);
    int getLedOn();
    void SimulationPeriodic() override;
    bool isTagPresent();
    

   private:
    // cs::UsbCamera usbCam = frc::CameraServer::StartAutomaticCapture(); //usb back camera
    // cs::CvSink m_cvSink = frc::CameraServer::GetVideo();
    // cs::CvSource m_outputStream = frc::CameraServer::PutVideo("front", 640, 480);

    std::shared_ptr<nt::NetworkTable> table;
    std::vector<double, std::allocator<double>> table2;
    frc::PIDController pid; // alignment pid
    double output;
    double distError;
    double distance;
    int ledOn;
    int tagID;
    bool tagPresent;
};