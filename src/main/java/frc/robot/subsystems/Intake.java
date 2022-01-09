// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase {

  public static CANSparkMax mIntakeArmSweeper;
  public static CANSparkMax mIntakeArmLift;
  
  /** Creates a new Intake. */
  public Intake() {
    mIntakeArmSweeper = new CANSparkMax(IntakeConstants.kArmSweeperCANID, MotorType.kBrushless);//CAN ID: 7
    mIntakeArmLift = new CANSparkMax(IntakeConstants.kArmLiftCANID, MotorType.kBrushless);//CAN ID: 8
    init();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void init(){
    mIntakeArmSweeper.clearFaults();
    mIntakeArmLift.clearFaults();
  }

  public void moveSweeperArm(String direction, double speed){
    if (direction == "Down"){
        mIntakeArmLift.set(-speed);
    }else if (direction == "Up"){
        mIntakeArmLift.set(speed);
    }
  }


public void stopSweeperArm(){
    mIntakeArmLift.set(0);
}

public void sweep(String direction, double speed){
    if (direction == "In"){
        mIntakeArmSweeper.set(speed);    
    } else if (direction == "Out")
        mIntakeArmSweeper.set(-speed);
    }

public void stopSweep(){
    mIntakeArmSweeper.set(0);
}



} //End of Intake subsystem
