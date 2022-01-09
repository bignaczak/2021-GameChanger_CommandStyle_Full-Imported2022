// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Button;

/**
 * Add your docs here.
 */
public class LoneJoystickButton extends Button {
  
  JoystickButton include;
  JoystickButton exclude1;
  JoystickButton exclude2;

  public LoneJoystickButton(JoystickButton include, JoystickButton exclude1, JoystickButton exclude2) {

    this.include = include;
    this.exclude1 = exclude1;
    this.exclude2 = exclude2;

  }
  
  @Override
  public boolean get() {
    if(include.get() && !exclude1.get() && !exclude2.get()) {
      return true;
    }
    return false;
  }
}
