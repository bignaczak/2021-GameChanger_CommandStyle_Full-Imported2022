// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
public final class Constants {

    //Imperial vs Metric  
    public static final boolean useMetric = true;

    //DriveTrain
    public static final class DriveConstants {   
    
        public static final int kLeftMotor1CANID = 1;
        public static final int kLeftMotor2CANID = 2;
        public static final int kRightMotor1CANID = 3;
        public static final int kRightMotor2CANID = 4;
        public static final int kIMUCANID = 40;

        public static final int kLeftMotorEncoderPort1 = 2;
        public static final int kLeftMotorEncoderPort2 = 3;
        public static final int kRightMotorEncoderPort1 = 4;
        public static final int kRightMotorEncoderPort2 = 5;
        public static final boolean kRightMotorEncoderReversed = true;


        public static final boolean kUseFieldOreint = false;
        public static final boolean kUseDriveRamping = true;

        public static final double yAxisRampingDeadband = 0.08;
        public static final double xAxisRampingDeadband = 0.08;
        public static final double yAxisDeadband = 0.1;
        public static final double xAxisDeadband = 0.1;

        public static final boolean kLeftMotorReversed = false;
        public static final boolean kRightMotorReversed = true;
        public static final int kDrivetrain_encResolution = 1024;
        public static final int kDrivetrain_WheelDiamInches = 6;
        public static final double kDrivetrain_GearReduction = 12.58;  //not needed unless we are using buit in Neo hall effect sensors
        public static final double kDrivetrain_WheelDiameter = useMetric?inchesToMeters(kDrivetrain_WheelDiamInches):kDrivetrain_WheelDiamInches; 
        
        //I am mesuring 2048 Counts per wheel revolution
        public static final double kCountsPerDist = 2048/(Math.PI*kDrivetrain_WheelDiameter);
        public static final double kDrivetrain_encDistPerPulse = 1/kCountsPerDist;

        public static final double kRobotLenInches = 22.875;
        public static final double kRobotWidthInches = 19.25;
        public static final double kRobotTrackWidthInches = 19.25;
        public static final double kRobotLen = useMetric?inchesToMeters(kRobotLenInches):kRobotLenInches;  
        public static final double kRobotWidth = useMetric?inchesToMeters(kRobotWidthInches):kRobotWidthInches;  // Verify
        public static final DifferentialDriveKinematics kDriveKinematics =
        new DifferentialDriveKinematics(useMetric?inchesToMeters(kRobotTrackWidthInches):kRobotTrackWidthInches);

        // These are example values only - DO NOT USE THESE FOR YOUR OWN ROBOT!
        // These characterization values MUST be determined either experimentally or theoretically
        // for *your* robot's drive.
        // The Robot Characterization Toolsuite provides a convenient tool for obtaining these
        // values for your robot.
        public static final double ksVolts = 0.22;
        public static final double kvVoltSecondsPerMeter = 1.98;
        public static final double kaVoltSecondsSquaredPerMeter = 0.2;

        // Example value only - as above, this must be tuned for your drive!
        public static final double kPDriveVel = 8.5;
    }

    public static final class OIConstants {
        public static final int kDriverControllerPort = 0;
        public static final int kSecondaryControllerPort = 1;
    }
    
    public static final class AutoConstants {
        public static final double kMaxSpeedMetersPerSecond = 3;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;
    
        // Reasonable baseline values for a RAMSETE follower in units of meters and seconds
        public static final double kRamseteB = 2;
        public static final double kRamseteZeta = 0.7;
    }

    public static final class ShooterConstants{
        public static final int kShooterMotorCANID = 12;
        public static double kAutonRPM = 4500;
        public static final double kBlueZoneRPM = 3700;
        public static final double kGreenZoneRPM = 1100;
        public static final double kYellowZoneRPM = 3100;
        public static final double kRedZoneRPM = 3100; //2788

        //Accuracy challenge p values
        public static final double pG = .0015;
        public static final double pB = .0023;
        public static final double pY = .0021;
        public static final double pR = .0020;
    }

    public static final class VisionConstants {
        public static final double kBlueOffest = 0;
        public static final double kYellowOffset = -6.7;
        public static final double kRedOffset = -11;
        public static final double kAngleThreshold= 0.2; //Set to 0.2 degress
        public static final double kVerticalThreshold = 0; //Set to 0.  must be exact
        public static final double kpAim = 0.01852; // Max Horiz Error is +/- 27 deg.
        public static final double kpVertical = 0.05;  //Current zone offset max (absolute) is red zone at 11
        public static final double kMinRotateSpeed = 0.15;
        public static final double kMinLinearSpeed = 0.1;

    }
    
    public static final class ConveyorConstants {   
        public static final int kConveyorUpperCANID = 5;
        public static final int kConveyorBottomCANID = 6;
        public static final double kConveyorSpitSpeed = 0.5; 
        public static final double kConveyorBottomFeedSpeed = 1.0;
        public static final double kConveyorFeedDeadband = 0.07;  
    }

    public static final class IntakeConstants {   
        public static final int kArmSweeperCANID = 7;
        public static final int kArmLiftCANID = 8;
        public static final double kSweeperOutSpeed = 0.5;
        public static final double kSweepArmSpeed = 0.1;
    }

    public static final class StiltConstants {
        public static final int kLeftStiltMotorCANID = 14;
        public static final int kRightStiltMotorCANID = 15;
        public static final int kLeftStiltEncoderPort1 = 6;
        public static final int kLeftStiltEncoderPort2 = 7;
        public static final int kRightStiltEncoderPort1 = 8;
        public static final int kRightStiltEncoderPort2 = 9;
        public static final boolean kLeftStiltEncoderReversed = false; 
        public static final boolean kRightStiltEncoderReversed = false;
        public static final double kPStilts = 1.2;
        public static final double kIStilts = 0;
        public static final double kDStilts = 0;
        public static final double kEncoderCountsFullTravel = 100;  //Need to set once we know value  
        public static final double kMinDesiredRPM = 50; //  RPM - need to tweak
        public static final double kMaxDesiredRPM = 100; // RPM - need to tweak
        public static final double kEncoderCountsMinRPMZone = 5;  //Need to set, this is where we want to insure minRPM
        public static final double kEncoderCountsTargetZone = 0;  //Allow for a zone to reach target instead of absolute
        public static final double kManualSpeed = 0.1; //PWM speed for manual movement
    }

    //quick conversion method between inches and meters
    private static double inchesToMeters(double input){
        return input/39.3701;
    }

}
