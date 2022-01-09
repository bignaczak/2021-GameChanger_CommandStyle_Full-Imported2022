// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/*----------------------------------------------------------------------------*/
/*  This code was written by programming mentor M. Kuehnel for reference      */
/*  Last Update:  3/9/2021                                                    */
/*  Status:  Untested                                                         */
/*           Stilts will be used in Green Zone to allow robot to shoot        */
/*           into the upper scoring location                                  */
/*                                                                            */
/*  Current Revision: 1.0                                                     */
/*                                                                            */
/*  Note -- Basing off the 2019 Stilts code since not using NEOs              */
/*       -- Will use velocity based PID Control for stilts                    */
/*       -- Initial Assumption is positive velocity extends                   */
/*       -- V1.0  Initial Code Creation                                        */
/*                                                                            */
/*                                                                            */
/*    Methods:                                                                */
/*        periodic()  Will be used to update dashboard                        */
/*        resetEncoders()   Sets encoder position to 0                        */
/*        getAverageEncoder()   Returns average of encoder distances          */
/*        configurePIDControllers(CANPIDController pid)   sets PID values     */
/*        stopStilts()  stops Stilt Motors                                    */
/*        boolean runStilts(String motion)  extends or retracts stilts        */
/*        runStiltsManual(String motion, String side) manual stilt control    */
/*                                                                            */
/* ---------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.StiltConstants;


public class Stilts extends SubsystemBase {

  private VictorSPX mLeftStiltMotor;
  private VictorSPX mRightStiltMotor;
  private Encoder  mLeftStiltEncoder;
  private Encoder  mRightStiltEncoder;
  private PIDController mLeftStiltPID;
  private PIDController mRightStiltPID;
  
  /** CONSTRUCTOR - Creates a new Stilts. */
  public Stilts() {
    mLeftStiltMotor = new VictorSPX(StiltConstants.kLeftStiltMotorCANID);  
    mRightStiltMotor = new VictorSPX(StiltConstants.kRightStiltMotorCANID); 
    mLeftStiltEncoder = new Encoder(StiltConstants.kLeftStiltEncoderPort1,StiltConstants.kLeftStiltEncoderPort2);
    mLeftStiltEncoder = new Encoder(StiltConstants.kRightStiltEncoderPort1,StiltConstants.kRightStiltEncoderPort2);
    //resetEncoders();
    mLeftStiltPID = new PIDController(StiltConstants.kPStilts, StiltConstants.kIStilts, StiltConstants.kDStilts);
    mRightStiltPID = new PIDController(StiltConstants.kPStilts, StiltConstants.kIStilts, StiltConstants.kDStilts);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Stilt Encoder Count", getAverageEncoder());
  }

  public void resetEncoders(){
    mLeftStiltEncoder.reset();;
    mRightStiltEncoder.reset();
  }


  public double getAverageEncoder(){
    return  (mLeftStiltEncoder.getDistance() + mRightStiltEncoder.getDistance())/2;
  }

  public void stopStilts(){
    mLeftStiltMotor.set(ControlMode.PercentOutput, 0);
    mLeftStiltMotor.set(ControlMode.PercentOutput, 0);
    SmartDashboard.putBoolean("Stilts Active", false);
  }

  //Use for manually running Stilts, assuming positive PWM is extending
  public void runStiltsManual(String motion, String side){
    boolean extendRequested = false;
    double manualPWM = 0;

    if (motion == "Extend"){extendRequested = true;}
    manualPWM = extendRequested?StiltConstants.kManualSpeed:-StiltConstants.kManualSpeed;
    
    switch (side){
      case "Left":  mLeftStiltMotor.set(ControlMode.PercentOutput, manualPWM);
      case "Right":  mRightStiltMotor.set(ControlMode.PercentOutput, manualPWM);
      SmartDashboard.putBoolean("Stilts Active", true);
    }

  } //End of RunStiltsManual Method

  /*Since we will use Velocity PID Control to run stilts to ensure levelling
  * We will need to control the target velocity based on current position
  * Thus we will calculate the targetReference for RPM based on encoder count % of full travel
  */
  public boolean runStiltsAuto(String motion){
    boolean stiltsAtTargetPosition = false;
    double currentStiltPosition = getAverageEncoder();
    double percentageOfTravel = 0;
    double targetVelocity = 0;//Target Velocity for PID Control, units are RPM
    double targetEncoderCounts = 0;

    //Set target EncoderCounts to 0 or full travel based on direction passed to method
    switch (motion){
      case "Extend":  targetEncoderCounts = StiltConstants.kEncoderCountsFullTravel; break;
      case "Retract":  targetEncoderCounts = 0; break;
    }

    //calculate the Target Reference Speed based on travel position
    percentageOfTravel = (Math.abs(currentStiltPosition - targetEncoderCounts))/StiltConstants.kEncoderCountsFullTravel;
    targetVelocity = percentageOfTravel*StiltConstants.kMaxDesiredRPM;
    if (percentageOfTravel <= StiltConstants.kEncoderCountsMinRPMZone){
      targetVelocity = StiltConstants.kMinDesiredRPM;  //Sets to minimum speed once we get close to end of travel
    }
    if (motion == "Retract"){
      targetVelocity = -targetVelocity;  //Sets velocity to negative if retracting
    }

    //set the Velocity PID based on desired direction and calculated Target RPM & output to motor controllers
    mLeftStiltPID.setSetpoint(targetVelocity);
    mRightStiltPID.setSetpoint(targetVelocity);    
    mLeftStiltMotor.set(ControlMode.Velocity, mLeftStiltPID.calculate(mLeftStiltEncoder.getRate()));
    mRightStiltMotor.set(ControlMode.Velocity, mRightStiltPID.calculate(mRightStiltEncoder.getRate()));

    //Check to see if we have reached end of travel
    if ( (Math.abs(targetEncoderCounts - StiltConstants.kEncoderCountsTargetZone) - currentStiltPosition) == 0  ){
      stiltsAtTargetPosition = true;
      stopStilts();
    } else{
      stiltsAtTargetPosition = false;
    }

    //Update SmartDashboard if stilts are at full extension
    if (  (currentStiltPosition - (StiltConstants.kEncoderCountsFullTravel - StiltConstants.kEncoderCountsTargetZone)) >=0){
      SmartDashboard.putBoolean("Stilts Fully Extended", true);
    } else {
      SmartDashboard.putBoolean("Stilts Fully Retracted", false);
    }

    return stiltsAtTargetPosition;
  } //end of runStilts Method

  
}
