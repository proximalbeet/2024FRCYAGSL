// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OIConstants;
import frc.robot.commands.LimelightArmCmd;
import frc.robot.commands.PickupCmd;
import frc.robot.commands.RotateToAprilTagCmd;
import frc.robot.commands.ShooterOutCmd;
import frc.robot.commands.SwerveJoystickCmd;
import frc.robot.commands.ZeroGyroCmd;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;


import java.util.function.DoubleSupplier;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;



/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  // Autonomous chooser for SmartDashboard
  private SendableChooser<Command> autonChooser = new SendableChooser<>();
  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem swerveDrive = new SwerveSubsystem();
  private final ArmSubsystem armSubsystem = new ArmSubsystem();
  private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
  private final LimelightSubsystem limelightSubsystem = new LimelightSubsystem();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandJoystick driverController =
      new CommandJoystick(OIConstants.kDriverControllerPort);
    
    private final CommandJoystick driverController2 =
      new CommandJoystick(OIConstants.kDriverControllerPort2);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    if(DriverStation.getAlliance().isPresent()) {
      SwerveSubsystem.allianceColor = DriverStation.getAlliance().get();
    } else {
      SwerveSubsystem.allianceColor = null;
    }

    // Configure the trigger bindings
    configureBindings();
    configureAutonomous();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    swerveDrive.setDefaultCommand(new SwerveJoystickCmd(swerveDrive,
      // Invert X Axis - WPIlib is forward-positive, joystick is down-positive
      axisDeadband(driverController, Constants.OIConstants.kDriveXAxis, Constants.OIConstants.kDriveDeadband, true),
      // Invert Y Axis - WPILib is left-positive, joystick is right-positive
      axisDeadband(driverController, Constants.OIConstants.kDriveYAxis, Constants.OIConstants.kDriveDeadband, true),
      axisDeadband(driverController, Constants.OIConstants.kRotAxis, Constants.OIConstants.kRotDeadband, true)

      
    ));
      driverController.button(Constants.OIConstants.kRotateToApriltagButton).whileTrue(new ZeroGyroCmd(swerveDrive));

      driverController2.button(Constants.OIConstants.kPickupButton).whileTrue(new PickupCmd(armSubsystem,shooterSubsystem));


      driverController2.button(Constants.OIConstants.kLimelightArmButton).whileTrue(new LimelightArmCmd(limelightSubsystem, swerveDrive, 
      axisDeadband(driverController, Constants.OIConstants.kDriveXAxis, Constants.OIConstants.kDriveDeadband, true), 
      axisDeadband(driverController, Constants.OIConstants.kDriveYAxis, Constants.OIConstants.kDriveDeadband, true)
      ));

      driverController2.button(Constants.OIConstants.kRotateToApriltagButton).whileTrue(new RotateToAprilTagCmd(swerveDrive, limelightSubsystem, 
      axisDeadband(driverController, Constants.OIConstants.kDriveXAxis, Constants.OIConstants.kDriveDeadband, true), 
      axisDeadband(driverController, Constants.OIConstants.kDriveYAxis, Constants.OIConstants.kDriveDeadband, true),
      axisDeadband(driverController, Constants.OIConstants.kRotAxis, Constants.OIConstants.kDriveDeadband, true)
      ));
     
      driverController2.button(Constants.OIConstants.kShootoutButton).whileTrue(new ShooterOutCmd(shooterSubsystem));
  //  driverController2.button(Constants.OIConstants.kShootoutButton)
  //                             .whileTrue(new ShooterOutCmd(
  //                                     shooterSubsystem, 
  //                                     () -> Constants.ShooterConstants.leftPowerN, 
  //                                     () -> Constants.ShooterConstants.rightPowerN));
  }

  private DoubleSupplier axisDeadband(CommandGenericHID controller, int axis, double deadband, boolean inverted) {
    double invertedMultiplier = inverted ? -1 : 1;
    return () -> {
      double axisOut = controller.getRawAxis(axis);
      return (Math.abs(axisOut) > deadband) ? axisOut * invertedMultiplier : 0;
    }; 
  }

 // Set up the autonomous routines
 private void configureAutonomous() {
  // Register named commands for pathplanner
  // This must be done before initializing autos
  NamedCommands.registerCommand("PickupCmd", new PickupCmd(armSubsystem, shooterSubsystem));
  NamedCommands.registerCommand("LimelightArmCmd", new LimelightArmCmd(limelightSubsystem, swerveDrive, 
      axisDeadband(driverController, Constants.OIConstants.kDriveXAxis, Constants.OIConstants.kDriveDeadband, true), 
      axisDeadband(driverController, Constants.OIConstants.kDriveYAxis, Constants.OIConstants.kDriveDeadband, true)
      ));
  NamedCommands.registerCommand("ShooterOutCmd", new ShooterOutCmd(shooterSubsystem));
  NamedCommands.registerCommand("ZeroGyroCmd", new ZeroGyroCmd(swerveDrive));

  autonChooser.setDefaultOption("NONE", Commands.print("No autonomous command selected!"));
  
  autonChooser.addOption("Safety Auto", new PathPlannerAuto("Safety Auto"));
  autonChooser.addOption("Top Auto", new PathPlannerAuto("Top Auto"));
  autonChooser.addOption("Center Auto", new PathPlannerAuto("Center Auto"));
  autonChooser.addOption("Bottom Auto", new PathPlannerAuto("Bottom Auto"));


  SmartDashboard.putData("autonDropdown", autonChooser);
}

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return Commands.print("no auton");
  }
}
