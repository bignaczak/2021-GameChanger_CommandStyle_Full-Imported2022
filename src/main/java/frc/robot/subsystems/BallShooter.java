// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Robot;

public class BallShooter extends SubsystemBase {

  public static CANSparkMax mShooterMotor;
  public static CANPIDController mShooterPID;
  public static CANEncoder mShooterEncoder;


  /** Creates a new BallShooter. */
  public BallShooter() {
    mShooterMotor = new CANSparkMax(ShooterConstants.kShooterMotorCANID, MotorType.kBrushless);
    mShooterMotor.clearFaults();
    mShooterPID = mShooterMotor.getPIDController();
    mShooterPID.setP(0.04);  //provide a base value until updated by accuracy challenge
    mShooterEncoder = mShooterMotor.getEncoder();
    resetEncoder();
  }



  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Actual Shooter RPM: ", mShooterEncoder.getVelocity());
    SmartDashboard.putBoolean("Shooter at Speed", shooterAtSpeed(Robot.Apollo_RobotContainer.Apollo_Dashboard));
  }


  public void resetEncoder(){
    mShooterEncoder.setPosition(0);
  }

  //Method used to update SmartDashboard "At Speed" Indicator
  public boolean shooterAtSpeed(UserInterface Dashboard){
    boolean returnValue = false;
    
    if (mShooterMotor.get() > 0){ //Only update if shooter motor is running
      if (mShooterEncoder.getVelocity() == setShooterForZone(Dashboard.getZoneSelection())[1]){
        returnValue = true;
      }else
      returnValue = false;
    }
    
    return returnValue;
  }  //End of ShooterAtSpeed Method


  public void runAtSpeed(double targetRPM){
    mShooterPID.setReference(targetRPM, ControlType.kVelocity);
  }

  public void runAtMax(){
    mShooterMotor.set(1);  //Full 100% power to motor controller
  }

  public void stopShooterMotor(){
    mShooterMotor.set(0);
  }


  //Method will be passed zone and then set shooter motor to run appropriately
  //Note motor stopping will need to be handled by Command structure
  public void accuracyChallenge(String zone) {
    double accuracy_kP = 0;
    double accuracy_Reference = 0;

    //set the kP and Reference values based on the zone
    accuracy_kP = setShooterForZone(zone)[0];
    accuracy_Reference = setShooterForZone(zone)[1];

    //Run Manually or Update the PID controller with correct parameters at start PID Control
    if (zone == "Manual"){
      mShooterMotor.set(1);  //Set maximum output to motor 
    } else
    mShooterPID.setP(accuracy_kP);
    mShooterPID.setReference(accuracy_Reference, ControlType.kVelocity);
    
  }  //End of AccuracyChallenge



  //Method is called by Accuracy Challenge to set the Target Reference Speed and assing kP value  
  public double[] setShooterForZone(String zone) {
    double kP_Shooter = 0.0;
    double Reference_Shooter = 0.0; 
    
    switch (zone){
        case "G":  kP_Shooter = ShooterConstants.pG; Reference_Shooter = ShooterConstants.kGreenZoneRPM; break;
        case "B":  kP_Shooter = ShooterConstants.pB; Reference_Shooter = ShooterConstants.kBlueZoneRPM; break;
        case "Y":  kP_Shooter = ShooterConstants.pY; Reference_Shooter = ShooterConstants.kYellowZoneRPM; break;
        case "R":  kP_Shooter = ShooterConstants.pR; Reference_Shooter = ShooterConstants.kRedZoneRPM; break;
    }
    
    double[] returnArray = {kP_Shooter, Reference_Shooter};
    return returnArray;

  }  //End of setKp

  

}
