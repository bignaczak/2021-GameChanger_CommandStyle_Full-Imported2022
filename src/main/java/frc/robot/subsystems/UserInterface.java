// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class UserInterface extends SubsystemBase {

  public SendableChooser<String> autonSelector = new SendableChooser<>();
  public SendableChooser<String> shooterSelector = new SendableChooser<>();


  /** Creates a new UserInterface. */
  public UserInterface() {
    createInitialDisplayContent();  //when instantiating a new subsystem, all options set with defaults
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void createInitialDisplayContent(){
    //Populate Entries for Autonomous 
    autonSelector.setDefaultOption("AutoNavSlalom", "AutoNavSlalom");
    autonSelector.addOption("Galactic SearchA", "Galactic SearchA");
    autonSelector.addOption("Galactic SearchA", "Galactic SearchB");
    autonSelector.addOption("AutoNavBarrel", "AutoNavBarrel");
    autonSelector.addOption("AutoNavSlalom", "AutoNavSlalom");
    autonSelector.addOption("AutoNavBounce", "AutoNavBounce");
    SmartDashboard.putData("Autonomous Choices", autonSelector);
    SmartDashboard.putNumber("Current Auton Path Segment: ", 0);

    //Populate Entries for Accuracy Challenge
    shooterSelector.setDefaultOption("Manual", "Manual");
    shooterSelector.addOption("Green", "G");
    shooterSelector.addOption("Yellow", "Y");
    shooterSelector.addOption("Blue", "B");
    shooterSelector.addOption("Red", "R");

    //Populate Entries for Vision
    SmartDashboard.putBoolean("Valid Target Found",false);
    SmartDashboard.putBoolean("Auto Alignment Active", false);
    SmartDashboard.putNumber("tx", 0);
    SmartDashboard.putNumber("ty", 0);

    //Populate Entries for Shooter
    SmartDashboard.putNumber("Target Shooter RPM: ", 0);
    SmartDashboard.putNumber("Actual Shooter RPM: ", 0);
    SmartDashboard.putBoolean("Shooter at Speed", false);

    //Populate Entries for DriveSubsystem
    SmartDashboard.putNumber("Drivetrain LeftSide Encoder: ", 0);
    SmartDashboard.putNumber("Drivetrain RightSide Encoder: ", 0);
    SmartDashboard.putNumber("Linear Distance Traveled (cm)", 0);
    SmartDashboard.putNumber("Current Gyro Heading", 0);
 
    //Populate Entries for Stilts
    SmartDashboard.putNumber("Stilt Encoder Count", 0);
    SmartDashboard.putBoolean("Stilts Fully Extended", false); 
    SmartDashboard.putBoolean("Stilts Active", false);
  }

  public String getAutonomousSelection(){
    return autonSelector.getSelected();
  }

  public String getZoneSelection(){
    return shooterSelector.getSelected();
  }
  
}
