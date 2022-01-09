// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.io.IOException;
import java.nio.file.Path;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;


// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html

public class runAutoNavSlalom extends SequentialCommandGroup {

  private static int step = 1;
    
  //Need to upate as we develop paths
  //Need to create Paths and Trajectories for each path
  private final String slalomTrajectory1JSON = "paths/D1toD8.wpilib.json";
  private final String slalomTrajectory2JSON = "paths/D8 to D10.wpilib.json";
  private Path slalomTrajectory1Path;
  private Path slalomTrajectory2Path;
  private Trajectory slalomTrajectory1;
  private Trajectory slalomTrajectory2;
  private RamseteController Apollo_RamseteController = new RamseteController(AutoConstants.kRamseteB, AutoConstants.kRamseteZeta);
  private SimpleMotorFeedforward Apollo_MotorFF = new  SimpleMotorFeedforward(DriveConstants.ksVolts, DriveConstants.kvVoltSecondsPerMeter, DriveConstants.kaVoltSecondsSquaredPerMeter);
  private RamseteCommand followPathSegment1;
  private RamseteCommand followPathSegment2;


  /** Creates a new runAutoNavSlalom. */
  public runAutoNavSlalom(TrajectoryConfig config, DriveSubsystem drivetrain) {
    followPathSegment1 = new RamseteCommand (slalomTrajectory1, drivetrain::getPose, Apollo_RamseteController, Apollo_MotorFF, DriveConstants.kDriveKinematics, drivetrain::getWheelSpeeds, new PIDController(DriveConstants.kPDriveVel, 0, 0), new PIDController(DriveConstants.kPDriveVel, 0, 0), drivetrain::tankDriveVolts, drivetrain);
    followPathSegment2 = new RamseteCommand (slalomTrajectory2, drivetrain::getPose, Apollo_RamseteController, Apollo_MotorFF, DriveConstants.kDriveKinematics, drivetrain::getWheelSpeeds, new PIDController(DriveConstants.kPDriveVel, 0, 0), new PIDController(DriveConstants.kPDriveVel, 0, 0), drivetrain::tankDriveVolts, drivetrain);
    InstantCommand startPose = new InstantCommand(() -> drivetrain.resetOdometry(slalomTrajectory1.getInitialPose()), drivetrain);
    InstantCommand updateStep = new InstantCommand(() -> updateStep(step));
    InstantCommand stopRobot = new InstantCommand(() -> drivetrain.tankDriveVolts(0, 0), drivetrain);                        

    addCommands(startPose, updateStep, followPathSegment1, updateStep, followPathSegment2, stopRobot);
    
  }

  private void updateStep(int pathSegment){
    SmartDashboard.putNumber("Current Auton Path Segment: ", pathSegment);
    step +=1;
  }

  //This will be run once wen the command is initially scheduled
  public void initialize(){
    try {
      slalomTrajectory1Path = Filesystem.getDeployDirectory().toPath().resolve(slalomTrajectory1JSON);
      slalomTrajectory2Path =Filesystem.getDeployDirectory().toPath().resolve(slalomTrajectory2JSON);
      slalomTrajectory1 = TrajectoryUtil.fromPathweaverJson(slalomTrajectory1Path);
      slalomTrajectory2 = TrajectoryUtil.fromPathweaverJson(slalomTrajectory2Path);
      } catch (IOException ex) {
        DriverStation.reportError("Unable to open trajectory: " + slalomTrajectory1JSON, ex.getStackTrace());
        DriverStation.reportError("Unable to open trajectory: " + slalomTrajectory2JSON, ex.getStackTrace());
      }
      System.out.println("Time to complete path 1 = " + slalomTrajectory1.getTotalTimeSeconds());
      System.out.println("Time to complete path 2 = " + slalomTrajectory2.getTotalTimeSeconds());      
  }  //end of initialize method for command

  public void execute(){

  }


}//End of runAutoNavSlalom CommandGroup
