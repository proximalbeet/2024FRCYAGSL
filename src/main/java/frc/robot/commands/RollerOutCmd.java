// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.RollerSubsystem;

public class RollerOutCmd extends Command {
  /** Creates a new RollerOutCmd. */
  RollerSubsystem rollerSubsystem;
  public RollerOutCmd(RollerSubsystem rollerSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(rollerSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    rollerSubsystem.ActivateShooterVelocity(Constants.Roller.rollerOutputVelocity);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    rollerSubsystem.StopRoller();

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
