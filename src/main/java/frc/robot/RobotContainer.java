// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/*----------------------------------------------------------------------------*/
/*  This code was written by programming mentor M. Kuehnel for reference      */
/*  Last Update:  3/13/2021                                                    */
/*  Status:  Base Function Succesful in Tele-Op on chassis bot in V1.0        */
/*           Subsystems and Commands not pertinent to driving commented out   */
/*                                                                            */
/*  CUrrent Revision: 3.0                                                     */
/*                                                                            */
/*  Note -- Revision 1.0 was first time using command structure               */
/*       -- All commands and subsystems are declared here                     */
/*          V1.0 tested on chassis bot with only DriveSubsystem               */
/*          V2.0 added comments, plus added all conveyor and vision           */
/*          V2.0 did not run on bot due to parallel command group error       */
/*            based on multiple commands linked to same subsystem             */
/*          V3.0 Broke apart Conveyor into (2) subsystems for error in 2.0    */                                                                     
/*                                                                            */
/*    Subsystems Registered:                                                  */
/*       Apollo_Shooter     BallShooter Subsystem                             */
/*       Apollo_ConveyorLower    Conveyor Lower Motor Subsystem               */
/*       Apollo_ConveyorUpper    Conveyor Upper Motor Subsystem               */     
/*       Apollo_Drivetrain  DriveSubsystem                                    */     
/*       Apollo_Intake      Intake Subsystem                                  */
/*       Apollo_Dashboard   UserInterface Subsystem                           */
/*       Apollo_Vision      VisionTracking Subsystem                          */
/*       Apollo_Stilts      Stilts Subsystem                                  */
/*                                                                            */
/*    Commands Linked to Buttons or Scheduled:                                */
/*                                                                            */
/* ---------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
// import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.ConveyorConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.runAutoNavSlalom;
import frc.robot.subsystems.BallShooter;
import frc.robot.subsystems.ConveyorLower;
import frc.robot.subsystems.ConveyorUpper;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Stilts;
import frc.robot.subsystems.UserInterface;
import frc.robot.subsystems.VisionTracking;
import frc.robot.utils.DoubleButton;
import frc.robot.utils.LoneJoystickButton;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */

public class RobotContainer {
  // The robot's subsystems
  public DriveSubsystem Apollo_Drivetrain = new DriveSubsystem();
  public BallShooter Apollo_Shooter = new BallShooter();
  public ConveyorUpper Apollo_ConveyorUpper = new ConveyorUpper();
  public ConveyorLower Apollo_ConveyorLower = new ConveyorLower();
  public Intake Apollo_Intake = new Intake();
  public UserInterface Apollo_Dashboard = new UserInterface();
  public VisionTracking Apollo_Vision = new VisionTracking();
  public Stilts Apollo_Stilts = new Stilts();


  // The driver and secondary controllers and Buttons
  Joystick DriverFlightstick = new Joystick(OIConstants.kDriverControllerPort);
  XboxController SecondaryController = new XboxController(OIConstants.kSecondaryControllerPort);
  JoystickButton xboxStart = new JoystickButton(SecondaryController, Button.kStart.value);  //Start Button
  POVButton xboxZero = new POVButton(SecondaryController, 0);  //Up on the POV 
  POVButton xbox180 = new POVButton(SecondaryController, 180); //down on the POV
  JoystickButton xboxB = new JoystickButton(SecondaryController, Button.kB.value);  // B Button
  JoystickButton xboxX = new JoystickButton(SecondaryController, Button.kX.value);  // X Button
  JoystickButton xboxA = new JoystickButton(SecondaryController, Button.kA.value);  // A Button
  JoystickButton xboxY = new JoystickButton(SecondaryController, Button.kY.value);  // Y Button
  // JoystickButton xboxLeftBumper = new JoystickButton(SecondaryController, Button.kBumperLeft.value);  // Left Bumper
  JoystickButton xboxLeftBumper = new JoystickButton(SecondaryController, Button.kLeftBumper.value);  // Left Bumper
  JoystickButton xboxRightBumper = new JoystickButton(SecondaryController, Button.kRightBumper.value);  // Right Bumper
  LoneJoystickButton lonexboxA = new LoneJoystickButton(xboxA, xboxLeftBumper, xboxRightBumper); // In case we want to use A button without bumpers
  LoneJoystickButton lonexboxY = new LoneJoystickButton(xboxY, xboxLeftBumper, xboxRightBumper); // In case we want to use A button without bumpers


  //Defining a new instance of the multi-button Class DoubleButton
  public DoubleButton LeftBumpAndA = new DoubleButton(xboxLeftBumper, xboxA);
  public DoubleButton RightBumpAndA = new DoubleButton(xboxRightBumper, xboxA);
  public DoubleButton LeftBumpAndY = new DoubleButton(xboxLeftBumper, xboxY);
  public DoubleButton RightBumpAndY = new DoubleButton(xboxRightBumper, xboxY);
  

  // Commands and Command Groups
  Command runConveyorUpperOut = new RunCommand(() -> Apollo_ConveyorUpper.runConveyorUpper("Out", ConveyorConstants.kConveyorSpitSpeed), Apollo_ConveyorUpper);
  Command runConveyorBottomOut = new RunCommand(() -> Apollo_ConveyorLower.runConveyorLower("Out", ConveyorConstants.kConveyorSpitSpeed), Apollo_ConveyorLower);
  Command runSweeperOut = new RunCommand(() -> Apollo_Intake.sweep("Out", IntakeConstants.kSweeperOutSpeed), Apollo_Intake);
  ParallelCommandGroup spitOutBalls = new ParallelCommandGroup(runConveyorUpperOut, runConveyorBottomOut, runSweeperOut);

  Command stopConveyorUpper = new InstantCommand(() -> Apollo_ConveyorUpper.stopUpper(), Apollo_ConveyorUpper);
  Command stopConveyorLower = new InstantCommand(() -> Apollo_ConveyorLower.stopLower(), Apollo_ConveyorLower);
  Command stopIntakeSweep = new InstantCommand(() -> Apollo_Intake.stopSweep(), Apollo_Intake);
  ParallelCommandGroup stopFullConveyorAndSweeper = new ParallelCommandGroup(stopConveyorUpper, stopConveyorLower, stopIntakeSweep);
  ParallelCommandGroup stopLowerConveyorAndSweeper = new ParallelCommandGroup(stopConveyorLower,stopIntakeSweep);

  Command runConveyorUpperIn = new RunCommand(() -> Apollo_ConveyorUpper.runConveyorUpper("In", 0.5*SecondaryController.getLeftTriggerAxis()), Apollo_ConveyorUpper);
  Command feedBallsToShooter = new ConditionalCommand(runConveyorUpperIn, stopConveyorUpper, () -> SecondaryController.getLeftTriggerAxis() > ConveyorConstants.kConveyorFeedDeadband);

  Command runConveyorLowerIn = new RunCommand(() -> Apollo_ConveyorLower.runConveyorLower("In", ConveyorConstants.kConveyorBottomFeedSpeed), Apollo_ConveyorLower);
  Command runSweeperIn = new RunCommand(() -> Apollo_Intake.sweep("In", SecondaryController.getRightTriggerAxis()), Apollo_Intake);
  ParallelCommandGroup runBallsIn = new ParallelCommandGroup(runConveyorLowerIn, runSweeperIn);
  Command intakeBalls = new ConditionalCommand(runBallsIn, stopLowerConveyorAndSweeper, () -> SecondaryController.getRightTriggerAxis() > ConveyorConstants.kConveyorFeedDeadband );

  
  Command runShooter = new RunCommand(() -> Apollo_Shooter.accuracyChallenge(Apollo_Dashboard.getZoneSelection()), Apollo_Shooter, Apollo_Dashboard);                                     
  Command stopShooter = new InstantCommand( () -> Apollo_Shooter.stopShooterMotor(), Apollo_Shooter);
  ParallelCommandGroup stopShooterAndUpper = new ParallelCommandGroup(stopShooter, stopConveyorUpper);
  
  Command waitForShooter = new WaitUntilCommand(() -> Apollo_Shooter.shooterAtSpeed(Apollo_Dashboard));
  ParallelCommandGroup autoShoot = new ParallelCommandGroup(runShooter,waitForShooter.andThen(runConveyorUpperIn));


  Command autoAlign = new FunctionalCommand(
    () -> SmartDashboard.putBoolean("Auto Alignment Active", true), //Runnable onInit
    () -> Apollo_Vision.alignToTarget(Apollo_Vision.getVisionTargetStatus(), Apollo_Vision.getVisionTargetHorizontalError(), Apollo_Vision.getVisionTargetVerticalError(), Apollo_Dashboard.getZoneSelection(), Apollo_Drivetrain), // Runnable onExecute
    interrupted -> Apollo_Drivetrain.arcadeDrive(0, 0),  //End Block
    () -> Apollo_Vision.alignToTarget(Apollo_Vision.getVisionTargetStatus(), Apollo_Vision.getVisionTargetHorizontalError(), Apollo_Vision.getVisionTargetVerticalError(), Apollo_Dashboard.getZoneSelection(), Apollo_Drivetrain), //isFinished Boolean Supplier
    Apollo_Drivetrain, Apollo_Vision// Require the drive and vision subsystems
    );

  Command autoZoneShoot = new SequentialCommandGroup(autoAlign.andThen( () -> SmartDashboard.putBoolean("Auto Alignment Active", false)), autoShoot );
  Command runShooterType = new ConditionalCommand(runShooter, autoZoneShoot, ()->Apollo_Dashboard.getZoneSelection() == "Manual");
  
  Command extendLeftStiltManual = new RunCommand (() -> Apollo_Stilts.runStiltsManual("Extend", "Left"), Apollo_Stilts);
  Command retractLeftStiltManual = new RunCommand ( () -> Apollo_Stilts.runStiltsManual("Retract", "Left"), Apollo_Stilts);
  Command extendRightStiltManual = new RunCommand (() -> Apollo_Stilts.runStiltsManual("Extend", "Right"), Apollo_Stilts);
  Command retractRightStiltManual = new RunCommand ( () -> Apollo_Stilts.runStiltsManual("Retract", "Right"), Apollo_Stilts);
  Command stopStilts = new InstantCommand( () ->Apollo_Stilts.stopStilts(), Apollo_Stilts);

  Command autoExtendStilts = new FunctionalCommand(
    () -> SmartDashboard.putBoolean("Stilts Active", true), //Runnable onInit
    () -> Apollo_Stilts.runStiltsAuto("Extend"), // Runnable onExecute
    interrupted -> Apollo_Stilts.stopStilts(),  //End Block
    () -> Apollo_Stilts.runStiltsAuto("Extend"), //isFinished Boolean Supplier
    Apollo_Stilts// Requires the stilt subsystems
    );

  Command autoRetractStilts = new FunctionalCommand(
    () -> SmartDashboard.putBoolean("Stilts Active", true), //Runnable onInit
    () -> Apollo_Stilts.runStiltsAuto("Retract"), // Runnable onExecute
    interrupted -> Apollo_Stilts.stopStilts(),  //End Block
    () -> Apollo_Stilts.runStiltsAuto("Retract"), //isFinished Boolean Supplier
    Apollo_Stilts// Requires the stilt subsystems
    );

  

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    //registers the subsystems so their periodic method is run by scheduler
    Apollo_Drivetrain.register();  
    Apollo_Shooter.register();
    Apollo_ConveyorUpper.register();
    Apollo_ConveyorLower.register();
    Apollo_Intake.register();
    Apollo_Dashboard.register();
    Apollo_Vision.register();
    Apollo_Stilts.register();
    
    //Sets the drivetrain to look for a run command that has no interruption
    //This could also be accomplished with a perpetual command.   Following WPILIB example though
    Apollo_Drivetrain.setDefaultCommand(new RunCommand(() -> Apollo_Drivetrain.teleopDrive(DriverFlightstick.getRawAxis(0), DriverFlightstick.getRawAxis(1)), Apollo_Drivetrain));
    
    // Configure the button bindings for commands tied to buttons
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

    //Reset Drive Encoders 
    xboxStart.whenPressed(new InstantCommand(Apollo_Drivetrain::resetEncoders, Apollo_Drivetrain));

    //Will run Motor if "Manual" is selected from Dashboard or full autoShoot if Zone is selected
    xboxX.whenPressed(runShooterType).whenReleased(stopShooterAndUpper);
  
    //Run Conveyor Upper towards Shooter
    feedBallsToShooter.schedule(); //note this is a conditional command based on Trigger > xx threshold
      
    //Run Conveyor Lower and Intake Sweep In
    intakeBalls.schedule();  //note this is a conditional command based on Trigger > xx threshold
    
    //Spit Out Balls
    xboxB.whenPressed(spitOutBalls).whenReleased(stopFullConveyorAndSweeper);
    
    //Intake Arm Raise and Lower
    xboxZero.whenPressed(() -> Apollo_Intake.moveSweeperArm("Up", IntakeConstants.kSweepArmSpeed)).whenReleased(()-> Apollo_Intake.stopSweeperArm());
    xbox180.whenPressed(() -> Apollo_Intake.moveSweeperArm("Lower", IntakeConstants.kSweepArmSpeed)).whenReleased(()-> Apollo_Intake.stopSweeperArm());
 
    //Manually run the Stilts
    LeftBumpAndA.whenPressed(extendLeftStiltManual).whenReleased(stopStilts);
    RightBumpAndA.whenPressed(extendRightStiltManual).whenReleased(stopStilts);
    LeftBumpAndY.whenPressed(retractLeftStiltManual).whenReleased(stopStilts);
    RightBumpAndY.whenPressed(retractRightStiltManual).whenReleased(stopStilts);

    //Auto run the Stilts
    lonexboxA.whenPressed(autoExtendStilts).whenReleased(stopStilts);
    lonexboxY.whenPressed(autoRetractStilts).whenReleased(stopStilts);

 
  }

  

  public String getAutonomousSelection(){
    return Apollo_Dashboard.getAutonomousSelection();
  } 

  public Command getAutonomousCommand(String autonSelection) {
     
    // Create a voltage constraint to ensure we don't accelerate too fast
    var autoVoltageConstraint =
        new DifferentialDriveVoltageConstraint(
            new SimpleMotorFeedforward(
                DriveConstants.ksVolts,
                DriveConstants.kvVoltSecondsPerMeter,
                DriveConstants.kaVoltSecondsSquaredPerMeter),
            DriveConstants.kDriveKinematics,
            10);

    // Create config for trajectory
    TrajectoryConfig config =
        new TrajectoryConfig(
                AutoConstants.kMaxSpeedMetersPerSecond,
                AutoConstants.kMaxAccelerationMetersPerSecondSquared)
            // Add kinematics to ensure max speed is actually obeyed
            .setKinematics(DriveConstants.kDriveKinematics)
            // Apply the voltage constraint
            .addConstraint(autoVoltageConstraint);

    
    SequentialCommandGroup autonCommandGroup = new runAutoNavSlalom(config, Apollo_Drivetrain);  //set this as default to initialize        
    switch (autonSelection){
      case "AutoNavSlalom":
        autonCommandGroup = new runAutoNavSlalom(config, Apollo_Drivetrain);
      break;
    }

    return autonCommandGroup;

  }  //End of Autonomous Command
}
