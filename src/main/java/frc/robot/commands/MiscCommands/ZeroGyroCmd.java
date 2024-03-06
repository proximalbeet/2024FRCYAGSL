// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.MiscCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;

public class ZeroGyroCmd extends Command {
  /** Creates a new ZeroGyroCmd. */
   private SwerveSubsystem swerve;

  public ZeroGyroCmd(SwerveSubsystem swerve) {
    this.swerve = swerve;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(swerve);
  }

  // Called when the command is initially scheduled.
  // Does not seem to run when using an InstantCommand
  @Override
  public void initialize() {
    swerve.zeroGyro();
  }

  @Override
  public void execute() {
    swerve.zeroGyro();
  }
}