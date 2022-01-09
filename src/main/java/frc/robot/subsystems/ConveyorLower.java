// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ConveyorConstants;

public class ConveyorLower extends SubsystemBase {

  public static CANSparkMax mConveyorLower;

  

  /** Creates a new Conveyor. */
  public ConveyorLower() {
    mConveyorLower = new CANSparkMax(ConveyorConstants.kConveyorBottomCANID, MotorType.kBrushless);//CAN ID: 5
    init();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void init() {
    mConveyorLower.clearFaults();
  }

  public void runConveyorLower(String direction, double speed){
    if (direction == "In"){
        mConveyorLower.set(speed);
    }else if (direction == "Out"){
        mConveyorLower.set(-speed);
    }
  }

  public void stopLower(){
    mConveyorLower.set(0);
  }
  

} // End of ConveyorLower Subystem
