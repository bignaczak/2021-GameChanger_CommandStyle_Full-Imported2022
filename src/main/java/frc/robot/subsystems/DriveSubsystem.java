// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/*----------------------------------------------------------------------------*/
/*  This code was written by programming mentor M. Kuehnel for reference      */
/*  Last Update:  3/8/2021                                                    */
/*  Status:  Base Function Succesful in Tele-Op, Path following untested      */
/*  Revision: 2.0                                                             */
/*                                                                            */
/*  Note -- Controls to run drivtrain for a tank drive setup                  */
/*       -- All required methods per the end to end trajectory following added*/   
/*          V1.0 untested, no comments                                        */
/*          V2.0 added comments, tested basic tele-op drive methods           */
/*                                                                            */
/*    methods:                                                                */
/*                                                                            */
/*                                                                            */
/* ---------------------------------------------------------------------------*/


package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix.sensors.PigeonIMU;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.Robot;
import frc.robot.Constants.DriveConstants;


public class DriveSubsystem extends SubsystemBase {
  
  //Speed Controllers
  public CANSparkMax mLeftMotor1 = new CANSparkMax(DriveConstants.kLeftMotor1CANID, MotorType.kBrushless);
  public CANSparkMax mLeftMotor2 = new CANSparkMax(DriveConstants.kLeftMotor2CANID, MotorType.kBrushless);
  public CANSparkMax mRightMotor1 = new CANSparkMax(DriveConstants.kRightMotor1CANID, MotorType.kBrushless);
  public CANSparkMax mRightMotor2 = new CANSparkMax(DriveConstants.kRightMotor2CANID, MotorType.kBrushless);

  // The motors on the left side of the drive.
  public SpeedControllerGroup mLeftMotors = 
  new SpeedControllerGroup(mLeftMotor1, mLeftMotor2);
  
  // The motors on the right side of the drive.
  public SpeedControllerGroup mRightMotors = 
  new SpeedControllerGroup(mRightMotor1, mRightMotor2);

  // The robot's drive
  public DifferentialDrive mDrive = 
  new DifferentialDrive(mLeftMotors, mRightMotors);

  // The left-side drive encoder
  public Encoder mLeftEncoder = 
  new Encoder(DriveConstants.kLeftMotorEncoderPort1, DriveConstants.kLeftMotorEncoderPort2);
  
  // The right-side drive encoder
  public Encoder mRightEncoder =
  new Encoder(DriveConstants.kRightMotorEncoderPort1, DriveConstants.kRightMotorEncoderPort2);

  //The PID Controllers
  public PIDController mLeftPIDController = new PIDController(DriveConstants.kPDriveVel, 0, 0);
  public PIDController mRightPIDController = new PIDController(DriveConstants.kPDriveVel,0, 0);

  // The gyro sensor
  public PigeonIMU mGyro = new PigeonIMU(DriveConstants.kIMUCANID);
  public double [] ypr = new double[3];
  public double [] xyz = new double[3];
 
  // Odometry class for tracking robot pose
  public DifferentialDriveOdometry mOdometry;

  
  /**
   * Constructor - Creates a new DriveTrain Subsystem.
   */
  public DriveSubsystem() {
    //Insure right motors are set to inverted 
    mRightMotor1.setInverted(DriveConstants.kRightMotorReversed);
    mRightMotor2.setInverted(DriveConstants.kRightMotorReversed);
    //Insure motors are set to brake mode 
    mLeftMotor1.setIdleMode(IdleMode.kBrake);
    mLeftMotor2.setIdleMode(IdleMode.kBrake);
    mRightMotor1.setIdleMode(IdleMode.kBrake);
    mRightMotor2.setIdleMode(IdleMode.kBrake);

    // Setup and reset the Encoders
    //mAlternateLeftEncoder.setPositionConversionFactor(DriveConstants.kDrivetrain_encDistPerPulse);
    //mAlternateRightEncoder.setPositionConversionFactor(DriveConstants.kDrivetrain_encDistPerPulse);
    mLeftEncoder.setDistancePerPulse(DriveConstants.kDrivetrain_encDistPerPulse);
    mRightEncoder.setDistancePerPulse(DriveConstants.kDrivetrain_encDistPerPulse);
    mRightEncoder.setReverseDirection(DriveConstants.kRightMotorEncoderReversed);
    resetEncoders();

    //Creates a new Odometry Class 
    mOdometry = new DifferentialDriveOdometry(getGyroRotation2d());  //requires a rotation2d object 
    
  } //End of Constructor
  

    //M. Kuehnel:  Pigeon IMU does not have a native get2drotation object so we have to create one
  public Rotation2d getGyroRotation2d(){
    mGyro.getYawPitchRoll(ypr);  //Gets IMU Data and puts Yaw, Pitch, Roll into YPR[]
    double rotationAngle = ypr[0]; // gets yaw
    rotationAngle=Math.toRadians(rotationAngle);  // Convert to Radians for Rotation 2d object
    return new Rotation2d(rotationAngle);  //returns a rotation2d object based on current gyto angle
  }


  public double getGyroAngle(){
    mGyro.getYawPitchRoll(ypr);
    return ypr[0];
  }

  @Override
  public void periodic() {
    // Update the odometry in the periodic block during autonomous routine
    if (Robot.Apollo_AutonomousCommand != null){
      mOdometry.update(getGyroRotation2d(), mLeftEncoder.getDistance(), mRightEncoder.getDistance());
    }
      //SmartDashboard.putNumber("Drivetrain LeftSide Encoder: ", mLeftEncoder.getDistance());
    SmartDashboard.putNumber("Drivetrain LeftSide Encoder: ", mLeftEncoder.getDistance());
    SmartDashboard.putNumber("Drivetrain RightSide Encoder: ", mRightEncoder.getDistance());
    SmartDashboard.putNumber("Linear Distance Traveled (cm)", getAverageEncoderDistance()*100);
    SmartDashboard.putNumber("Current Gyro Heading", getHeading());
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return mOdometry.getPoseMeters();
  }

  /**
   * Returns the current wheel speeds of the robot.
   *
   * @return The current wheel speeds.
   */
  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(mLeftEncoder.getRate(), mRightEncoder.getRate());
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    mOdometry.resetPosition(pose, getGyroRotation2d());
  }

  /**
   * Drives the robot using arcade controls.
   *
   * @param fwd the commanded forward movement
   * @param rot the commanded rotation
   */
  public void arcadeDrive(double fwd, double rot) {
    mDrive.arcadeDrive(fwd, rot);
  }

  

  /**
   * Controls the left and right sides of the drive directly with voltages.
   *
   * @param leftVolts  the commanded left output
   * @param rightVolts the commanded right output
   */
  public void tankDriveVolts(double leftVolts, double rightVolts) {
    mLeftMotors.setVoltage(leftVolts);
    mRightMotors.setVoltage(-rightVolts);
    mDrive.feed();
  }

  /**
   * Resets the drive encoders to currently read a position of 0.
   */
  public void resetEncoders() {
    mLeftEncoder.reset();
    mRightEncoder.reset();
  }

  /**
   * Gets the average distance of the two encoders.
   *
   * @return the average of the two encoder readings
   */
  public double getAverageEncoderDistance() {
    return (mLeftEncoder.getDistance() + mRightEncoder.getDistance()) / 2.0;
  }

  /**
   * Gets the left drive encoder.
   *
   * @return the left drive encoder
   */
  public Encoder getLeftEncoder() {
    return mLeftEncoder;
  }

  /**
   * Gets the right drive encoder.
   *
   * @return the right drive encoder
   */
  public Encoder getRightEncoder() {
    return mRightEncoder;
  }

  /**
   * Sets the max output of the drive.  Useful for scaling the drive to drive more slowly.
   *
   * @param maxOutput the maximum output to which the drive will be constrained
   */
  public void setMaxOutput(double maxOutput) {
    mDrive.setMaxOutput(maxOutput);
  }

  /**
   * Zeroes the heading of the robot.
   */
  public void zeroHeading() {
    mGyro.setYaw(0);
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {
    mGyro.getYawPitchRoll(ypr);  //Gets IMU Data and puts Yaw, Pitch, Roll into YPR[]
    return ypr[0]; // gets yaw angle 
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    mGyro.getRawGyro(xyz);  // Fills yxz array with data in degrees per second
    return xyz[2];  // returns rotation around the Z or vertical axis.  should be the rotation
    //The example code returned an inverse.   Not sure if due to installation??
    //return -m_gyro.getRate();
  }



  /******************************************************************************************/
  /*                                                                                        */
  /*                                                                                        */ 
  /*    The following methods are the normal drive processing, ramping and methods          */
  /*    that our team has used for last 2 seasons for manual driving                        */
  /*                                                                                        */
  /*    These allow for more flexibility than simply using available arcadeDrive Classes    */
  /*                                                                                        */
  /*                                                                                        */
  /******************************************************************************************/

  //Drive Command to send values to DriveBase Motors
  public void teleopDrive(double xAxis, double yAxis){
    
    double[] driveprocessoutput = driveProcessing(xAxis, yAxis);  //determine motor output per controller
    tankDrivePWM(driveprocessoutput[0], driveprocessoutput[1]);  //send output to motor speed controllers
   
  }  //End of teleopDrive

  public void tankDrivePWM(double left, double right){
    mLeftMotor1.set(left);
    mLeftMotor2.set(left);
    mRightMotor1.set(right);
    mRightMotor2.set(right);
  }
  public double[] driveProcessing (double xAxis, double yAxis){

    double returnValue[] = {0,0};
    double strafe = xAxis;
    double fwd = -yAxis;  //Invert Flightstick values as forward is negative

    //First check to see if input is within deadzone
    strafe = deaden(strafe, DriveConstants.xAxisDeadband);
    fwd = deaden(fwd, DriveConstants.yAxisDeadband);
    
    //Run Field Oriented if Desired and IMU is installed and ready
    if (DriveConstants.kUseFieldOreint){
        double gyroInputInRadians = Math.toRadians(getHeading());
        double temp  =  fwd*Math.cos(gyroInputInRadians) + strafe*Math.sin(gyroInputInRadians);
        strafe   = -fwd*Math.sin(gyroInputInRadians) + strafe*Math.cos(gyroInputInRadians);
        fwd   =  temp;
    }

    //Run Ramping if Desired on driver input to make low speed easier to maintain
    if (DriveConstants.kUseDriveRamping){ 
      fwd = ramp(fwd, 0.5, DriveConstants.yAxisRampingDeadband);
      strafe = ramp(strafe, 0.5, DriveConstants.xAxisRampingDeadband);
    }
    
    //Bind the output to insure it is not above 100%
    double L = boundSpeed(fwd + strafe);
    double R = boundSpeed(fwd - strafe);

    // R = Robot.rightSideRequiresInversion? -R: R;  
    // I inverted motor controller in constructor for right side so no need to invert right side
    
    returnValue[0] = L;
    returnValue[1] = R;
    return returnValue;

  } // End of driveProcessing


  //Ramping power method. Can be called in drive processing if desired  (useDriveRamping constant)
  private double ramp(double input, double rampRange, double deadBand){
    double rampedOutput = 0;

    if(Math.abs(input)>0 && Math.abs(input)<= deadBand){  //Deadband zone
      rampedOutput = 0;
    } else if (Math.abs(input)> deadBand && Math.abs(input) <=rampRange){ //Ramping Zone
      rampedOutput = 3*input*input*input + Math.signum(input)*0.125;
    } else{  // Linear Range
     rampedOutput = input;
    }
  
    return rampedOutput;
  }  //End of ramp method

  //Quick Binding of speed to not go above 100%
  public double boundSpeed(double inputSpeed){
    return (Math.abs(inputSpeed) >= 1? Math.signum(inputSpeed)*1:inputSpeed);
  }

  
  public static double deaden(double input, double deadband) {
    deadband = Math.min(1, deadband);
    deadband = Math.max(0, deadband);
   
    if (Math.abs(input) - deadband < 0) {
      return 0;
    }else 
      return Math.signum(input) * ((Math.abs(input) - deadband) / (1 - deadband));
  } //End of deaden


  public static double bound(double input, double limitAbsolute){
     return Math.abs(input)<=Math.abs(limitAbsolute)?input:Math.signum(input)*limitAbsolute;
  } //End of bound


} //End of Class
