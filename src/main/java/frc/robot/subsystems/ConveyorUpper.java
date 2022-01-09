// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ConveyorConstants;

public class ConveyorUpper extends SubsystemBase {

  public static CANSparkMax mConveyorUpper;

  /** Creates a new Conveyor Upper */
  public ConveyorUpper() {
    mConveyorUpper = new CANSparkMax(ConveyorConstants.kConveyorUpperCANID, MotorType.kBrushless);//CAN ID: 5
    init();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void init() {
    mConveyorUpper.clearFaults(); 
  }

  public void runConveyorUpper(String direction, double speed){
    if (direction == "In"){
        mConveyorUpper.set(speed);
    }else if (direction == "Out"){
        mConveyorUpper.set(-speed);
    }
  }

  public void stopUpper(){
    mConveyorUpper.set(0);
  }

  
  
} // End of ConveyorUpper Subsystem
