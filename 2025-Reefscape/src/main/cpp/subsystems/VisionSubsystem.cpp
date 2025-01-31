#include "subsystems/VisionSubsystem.h"

#include <span>

#include "frc/smartdashboard/Smartdashboard.h"
#include "networktables/NetworkTable.h"
#include "networktables/NetworkTableEntry.h"
#include "networktables/NetworkTableInstance.h"
#include "networktables/NetworkTableValue.h"

using namespace VisionConstants;
using namespace MathConstants;

VisionSubsystem::VisionSubsystem() 
: output(0), distError(0), pid(kvP, kvI, kvD), ledOn(1)
 {
  //usbCam.SetResolution(640, 480);
  
}

frc2::CommandPtr VisionSubsystem::VisionMethodCommand() {
    return RunOnce([/* this */] { /* one-time action goes here */ });
}

bool VisionSubsystem::VisionCondition() {
    return false;
}

void VisionSubsystem::Periodic() {
    table = nt::NetworkTableInstance::GetDefault().GetTable("limelight");
    table2 = nt::NetworkTableInstance::GetDefault().GetTable("limelight")->GetNumberArray("tid", std::vector<double>(6));
    nt::NetworkTableInstance::GetDefault().GetTable("limelight")->PutNumber("ledMode", ledOn);

    // frc::PIDController pid(kvP, kvI, kvD); dont need to recreate pid every
    // periodic cycle
    // double targetOffsetAngle_Vertical = table->GetNumber("ty", 0.0);
    // if (targetOffsetAngle_Vertical ==0){
    //     tagPresent=false;
    // }
    // else{
    //   tagPresent=true;
    // }
    // // double id = table2.at(0);
    double id = table->GetNumber("tid", 0.0);
    // tagID = id;
    // //changee to match specific conditions
    // double goalHeightInches = 0.0;
    // //// 2024 code
    // //double goalHeightInches = 12.0;
    // // if (id == 4.0 || id == 7.0){
    // //   goalHeightInches = VisionConstants::speakerTagHeight;
    // // }
    // // else if (id == 10.0 || id == 9.0 || id == 1.0 || id == 2.0){
    // //   goalHeightInches = VisionConstants::sourceTagHeight;
    // // }
    // // else if (id == 5.0 || id == 6.0){
    // //   goalHeightInches = VisionConstants::ampTagHeight;
    // // }
    // // else if (id == 11.0 ||id == 12.0 ||id == 13.0 ||id == 14.0 ||id == 15.0 ||id == 16.0){
    // //   goalHeightInches = VisionConstants::stageTagHeight;
    // // }
    
    
    // double angleToGoalDegrees = VisionConstants::limelightAngleAboveHorizontal + targetOffsetAngle_Vertical;
    // double angleToGoalRadians = angleToGoalDegrees * (pi / 180.0);

    // // calculate distance
    // double distanceFromLimelightToGoalInches = (VisionConstants::speakerTagHeight - VisionConstants::limelightHeight.value()) / tan(angleToGoalRadians);
    // distance = distanceFromLimelightToGoalInches;
    // // double distanceFromCenterToGoalInches=sqrt(pow(distanceFromLimelightToGoalInches,2)-pow(VisionConstants::limelightHorizontalOffset.value(),2));
    // float KpDistance = -0.1f;  // Proportional control constant for distance 
    // float desired_distance = 50;
    // float distance_error = abs(desired_distance-distanceFromLimelightToGoalInches) * KpDistance; 
    // // reason the swerve drive doesn't stop moving when trying to align to a certain distance from tag. taking the absolute value of it ensures that it will always be positive meaning the robot wont ever think to change directions
    
    // if (targetOffsetAngle_Vertical == 0){
    //   setDistanceError(0);
    // }
    // else{
    //   setDistanceError(distance_error);
    // }
    

    //frc::SmartDashboard::PutNumber("up angle", targetOffsetAngle_Vertical);
    // frc::SmartDashboard::PutNumber("distance", distanceFromLimelightToGoalInches);
    // frc::SmartDashboard::PutNumber("actualdistance", distanceFromCenterToGoalInches);
    frc::SmartDashboard::PutNumber("id", id);
    double targetOffsetAngle_Horizontal = table->GetNumber("tx", 0.0);
    double heading_error = targetOffsetAngle_Horizontal;//+VisionConstants::subWooferAngleOffset;//asin(VisionConstants::limelightHorizontalOffset.value()/distanceFromLimelightToGoalInches);
    // pid.SetSetpoint(0);
    //frc::SmartDashboard::PutNumber("heading", heading_error);
    //frc::SmartDashboard::PutNumber("tx", targetOffsetAngle_Horizontal);
    double output = pid.Calculate(heading_error, 0);
    //frc::SmartDashboard::PutNumber("pid", output);
    if (targetOffsetAngle_Horizontal != 0){
      setOutput(output);
    }


    // pid.Calculate();
}

void VisionSubsystem::setOutput(double op) { output = op; }

double VisionSubsystem::getOutput() { return output; }

void VisionSubsystem::setLedOn(int ledsOn) {ledOn = ledsOn;}
int VisionSubsystem::getLedOn(){return ledOn;}

bool VisionSubsystem::isTagPresent(){
  return tagPresent;
}
void VisionSubsystem::setDistanceError(double dist_error) {
    distError = dist_error;
}
int VisionSubsystem::getID(){
    return tagID;
}
double VisionSubsystem::getDistanceError() { return distError; }
double VisionSubsystem::getDistance(){
  return distance;
}
void VisionSubsystem::SimulationPeriodic() {
    // Implementation of subsystem simulation periodic method goes here.
}

double VisionSubsystem::getX(){
    return table->GetNumber("tx", 0.0);
}

double VisionSubsystem::getY() {
    return table->GetNumber("ty", 0.0);
}

double VisionSubsystem::getZ() {
    return table->GetNumber("tz", 0.0);
}