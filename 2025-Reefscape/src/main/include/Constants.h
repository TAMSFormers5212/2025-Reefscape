#pragma once

#include <frc/geometry/Rotation3d.h>
#include <frc/geometry/Transform3d.h>
#include <frc/geometry/Translation2d.h>

#include <units/length.h>
#include <units/angle.h>
#include <units/angular_velocity.h>
#include <units/angular_acceleration.h>
#include <units/acceleration.h>
#include <units/math.h>
#include <units/velocity.h>
#include <units/length.h>
#include <units/time.h>
#include <units/voltage.h>
// #include <units/base.h>
#include <numbers>
#include <string>
#include <vector>
#include <array>

namespace PortConstants {


    //LEDS?
    //vision or other accessories

    constexpr int kMXP = 2; // mxp port

    //digital io 


    //analog in - the only ones we're using rn are the swerve encoders and those are in SwerveConstants

} 

namespace MathConstants {
    constexpr double pi = 3.1415926535;
    constexpr double pi2 = 6.283185307;

    constexpr double gravityMeters = 9.81;
    constexpr double gravityFeet = 32.1741;

    constexpr double InchToCM = 2.54;
}


namespace OIConstants {//Controller buttons 

    constexpr int kDriverControllerPort = 0;
    constexpr int kOperatorControllerPort = 1;

// need to check these constants, previous code used xbox controller, now its hid controller
    //flight joystick controller
    namespace Joystick{
        //Axis
        constexpr int XAxis = 0; 
        constexpr int YAxis = 1; 
        constexpr int RotAxis = 2; 
        constexpr int ThrottleSlider = 3;
        constexpr int miniXAxis = 5; // mini axis only returns -1, 0, or 1
        constexpr int miniYAxis = 6;
        //Buttons
        constexpr int Trigger = 1; 
        constexpr int ButtonTwo = 2; // side thumb button
        constexpr int ButtonThree = 3; // 3-6 on top
        constexpr int ButtonFour = 4;
        constexpr int ButtonFive = 5;
        constexpr int ButtonSix = 6;
        constexpr int ButtonSeven = 7; // 7-12 on bottom
        constexpr int ButtonEight = 8;
        constexpr int ButtonNine = 9;
        constexpr int ButtonTen = 10;
        constexpr int ButtonEleven = 11;
        constexpr int ButtonTwelve = 12;

        constexpr double deadband = 0.15; 
    }

    namespace Controller{//console controller
        //axis
        constexpr int leftXAxis = 0;
        constexpr int leftYAxis = 1;
        constexpr int rightXAxis = 4;
        constexpr int rightYAxis = 5;
        constexpr int leftTrigger = 2;
        constexpr int rightTrigger = 3;
        //buttons
        constexpr int A = 1;
        constexpr int B = 2;
        constexpr int X = 3;
        constexpr int Y = 4;
        constexpr int leftBumper = 5;
        constexpr int rightBumper = 6;
        constexpr int View = 7;
        constexpr int Menu = 8;
        constexpr int LPress = 9;
        constexpr int RPress = 10;
        //POV buttons (the d-pad)
        constexpr int leftAngle = 270;
        constexpr int downAngle = 180;
        constexpr int rightAngle = 90;
        constexpr int upAngle = 0;
        constexpr int left = 0; // not actual numbers, just filler rn
        constexpr int down = 1;
        constexpr int right = 2;
        constexpr int up = 3;
    }



}

namespace SwerveModuleConstants {//per swerve module
    constexpr double ktP = 0.8; // Turning PID
    constexpr double ktI = 0.0;
    constexpr double ktD = 0.0;
    constexpr double ktFF = 0.0;//0.00001;
    
    constexpr double kdP = 0.1; // Driving Speed PID
    constexpr double kdI = 0.0;
    constexpr double kdD = 0.0;
    constexpr double kdFF = 0.225;

    constexpr auto maxSpeed = 1.00_mps; // max free speed of SDS mk4 L1 withs neos
    constexpr auto maxRotation = 2.0_rad_per_s; // figure this out later
    constexpr double driveRatio = 8.14; //SDS Mk4 L1
    constexpr double steerRatio = 12.8; //SDS Mk4 L1

    // written this way to adjust wheel radius/diameter more easily
    constexpr units::inch_t wheelDiameter = 4_in; // likely different based on tread wear
    constexpr units::inch_t wheelCircumfrence = units::inch_t{wheelDiameter.value()*MathConstants::pi}; // recalculate based on wheel diameter 
    constexpr units::inch_t centerDistance = 10.5_in;
    constexpr units::meter_t wheelCircumfrenceMeters = units::meter_t{wheelCircumfrence};//0.31918581_m;

    constexpr double khP = 0.0; // Heading PID
    constexpr double khI = 0.0;
    constexpr double khD = 0.0;
    constexpr double khFF = 0.00000;

    constexpr double kxP = 0.0; // X PID
    constexpr double kxI = 0.0;
    constexpr double kxD = 0.0;
    constexpr double kxFF = 0.00000;

    constexpr double kyP = 0.0; // Y PID
    constexpr double kyI = 0.0;
    constexpr double kyD = 0.0;
    constexpr double kyFF = 0.00000;

    //possible heaving, x, and y PID for auto/pathplanning

    constexpr double kRampTimeSeconds = 0.1; // slew rate limiter (delay on acceleration)

    namespace drivebase{
        constexpr units::meter_t WheelBase = 0.5451_m; // for kinematics
        constexpr units::meter_t TrackWidth = 0.5451_m; // aka 21.5 inches
        // 27x27 inch chassis from 2023, will update once 2024 chassis is assembed, but its just +1 inch 
        // now updated to 28x28 2024 chassis
    }
    //2, 6, 3, 8
    //3 0 2 1
    
    namespace topleft{
        constexpr int driveMotor = 9; //CAN ID
        constexpr int steerMotor = 2; //CAN ID
        constexpr int absencoder = 3; //analogin Port

        constexpr double offset = 0.508-0.5; 
        //0.508-0.25

    }
    namespace topright{
        constexpr int driveMotor = 5; //CAN ID
        constexpr int steerMotor = 6; //CAN ID
        constexpr int absencoder = 0; //analogin Port
 
        constexpr double offset = 0.170+0.5;
        //0.170-0.25+1
    }
    namespace bottomleft{
        constexpr int driveMotor = 4; //CAN ID
        constexpr int steerMotor = 3; //CAN ID
        constexpr int absencoder = 2; //analogin Port

        constexpr double offset = 0.322 + 0.5;
        //0.322 - 0.25
    }
    namespace bottomright{          
        constexpr int driveMotor = 7; //CAN ID
        constexpr int steerMotor = 8; //CAN ID
        constexpr int absencoder = 1; //analogin Port

        constexpr double offset =0.834;
        //0.834 + 0.25 - 1
    }

}

namespace ArmConstants{

    constexpr int leftMotor = 12;
    constexpr int rightMotor = 11;

    constexpr int encoder = 2; // depends on what encoder. planning for a rev through bore 
    // constexpr int limitSwitch = 1;

    constexpr double encoderOffset = 0;

    constexpr double sprocketRatio = 5.5;
    constexpr int planetaryRatio = 20;
    constexpr int armRatio = 110; // 20:1 maxplanetary * 66:12 sprocket
    
    constexpr double kaP = 0.50;
    constexpr double kaI = 0.0000;
    constexpr double kaD = 0.00;
    constexpr double kaFF = 0.0;
    constexpr double kaIz = 0.0;
    constexpr units::second_t kaT = 20_ms;

    constexpr int kMaxOutput = 1;
    constexpr int kMinOutput = -1;
    constexpr units::radians_per_second_squared_t maxAccel{1};
    constexpr units::radians_per_second_t maxVelo{1};
    constexpr double allowedError = 0.0; 
    constexpr double stableDistance = 0;
    
    
    constexpr double minPosition = 0; // soft limits
    constexpr double maxPosition = 1;
    
    // currently just the estimates from recalc
    constexpr units::volt_t kaS{0.0};
    constexpr units::volt_t kaG{0.50};
    constexpr units::volt_t vkaV{0.014};
    constexpr units::radians_per_second_t akaV{1};
    constexpr auto kaV = vkaV / akaV;
    constexpr units::volt_t vkaA{0.05};
    constexpr units::radians_per_second_squared_t akaA{1};
    constexpr auto kaA = vkaA / akaV ; // not nessecary
}

namespace ShooterConstants{
    constexpr int leftMotor = 15;
    constexpr int rightMotor = 14;
    constexpr double pulleyRatio = 30.0/18.0;
    constexpr units::inch_t wheelDiameter = 4_in; // may change based on rpm
    
    //velocity pid
    constexpr double ksS = 0.1;
    constexpr double ksV = 0.1;
    constexpr double ksA = 0.0;
    
    //idk abt these values so making new ones
    constexpr double ksP = 0.001; // shooter uses a velocity PID 0.001
    constexpr double ksI = 0.0; // since the shooter motors are completely separate, it may be useful to tune them separately
    constexpr double ksD = 0.0005; // 0.0005
    constexpr double ksFF = 0.0;

    constexpr units::revolutions_per_minute_t maxNeoRpm = 5700_rpm;
    constexpr units::revolutions_per_minute_t maxWheelRpm = 9000_rpm;//replace this value with whatever max andymark says
    constexpr units::feet_per_second_t maxExitVelocity = 90_fps;

    constexpr units::volt_t KlsS{0.16}; // friction term`
    constexpr units::volt_t vKlsV{0.019}; // velocity term
    constexpr units::meter_t aKlsV{1};
    constexpr units::second_t sKlsV{1}; 
    constexpr auto KlsV = vKlsV *sKlsV / aKlsV;
    

    constexpr units::volt_t KrsS{0.195}; // friction term
    constexpr units::volt_t vKrsV{0.019}; // velocity term
    constexpr units::meter_t aKrsV{1};
    constexpr auto KrsV = vKrsV *sKlsV/ aKrsV;

}

namespace IntakeConstants{
    constexpr int motor = 13;

    constexpr int beamBreakIO = 0;  // digital io pins

    constexpr double intakeRatio = 2.0;

    constexpr double loadedCurrent = 10.0; // current when note is held
    constexpr double freeCurrent = 5.0; // current when no note is held
    //random guess numbers, experiement in order to prevent burning out the neo550
    
    constexpr int empty = 0;
    constexpr int held = 1;
    constexpr int indexed = 2;
}

namespace WinchConstants{
    constexpr int leftWinchMotor = 1;
    constexpr int rightWinchMotor = 10;


    constexpr double winchRatio = 60;
    constexpr units::inch_t winchDiameter = 0.5_in;
    constexpr units::inch_t heightToTravel = 27_in; // distance from chain to winch, aka max winchable distance

    constexpr double kwP = 1;
    constexpr double kwI = 0.0;
    constexpr double kwD = 0.0;
    constexpr double kwFF = 0.0;
    constexpr double kwIz = 0.0;
}

namespace PoseConstants{
    //normal setpoints

    //Amp
    constexpr double armAmp = 0;
    constexpr double ampAngle = 0;
    constexpr double ampSpeed = 0;

    //Climb
    constexpr double armClimb = 0;

}

namespace VisionConstants{

    //vision pid constants for aiming
    constexpr double kvP = 0.05;
    constexpr double kvI = 0.00;
    constexpr double kvD = 0.01;
    constexpr double kvFF = 0.0;

    constexpr double stableSpeed = 1.0; // random value for now

    //vision pid constants for translating

    constexpr units::inch_t limelightHeight = 16.5_in; // get a more precise number

    constexpr double limelightAngleback = 65.7;//degrees tilted back
    constexpr double limelightAngleAboveHorizontal= 23.9; // why this number?
    //constexpr double limelightToCenter = 9.75_in;
    constexpr units::inch_t limelightToFrame = 1.5_in;
    // constexpr double subWooferAngleOffset = 14.8;
 
    constexpr double TagHeight = 8;// height of the tags
    constexpr double TagCenterHeight = 4; // center of the tag from the middle

    // tags are measured from the carpet to the bottom of the tag
    constexpr double speakerTagHeight = 4*12+3+7.0/8; // height in inches
    constexpr double sourceTagHeight = 4*12+1.0/8;
    constexpr double ampTagHeight = 4 * 12 + 1.0 / 8;
    constexpr double stageTagHeight = 4*12-0.5;

    // 0. speaker center, 1. speaker side, 2. amp, 3. stage amp, 4. stage source, 5. stage center, 6. source close, 7. source far
    // constexpr std::array<int, 8> redTags = {4, 3, 5, 12, 11, 13, 10, 9};
    // constexpr std::array<int, 8> blueTags = {7, 8, 6, 15, 16, 14, 1, 2};
    constexpr int redSpeakerCenter = 4;
    constexpr int redSpeakerSide = 3;
    constexpr int redAmp = 5;
    constexpr int redStageAmp = 12;
    constexpr int redStageSource = 11;
    constexpr int redStageCenter = 13;
    constexpr int redSourceClose = 10;
    constexpr int redSourceFar = 9;
    constexpr int blueSpeakerCenter = 7;
    constexpr int blueSpeakerSide = 8;
    constexpr int blueAmp = 6;
    constexpr int blueStageAmp = 15;
    constexpr int blueStageSource = 16;
    constexpr int blueStageCenter = 14;
    constexpr int blueSourceClose = 1;
    constexpr int blueSourceFar = 2;
} 