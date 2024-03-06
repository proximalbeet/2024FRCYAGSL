// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.NoteHolderSubsystem;

public class NoteHolderOutCmd extends Command {
  public NoteHolderSubsystem holderSubsystem;
  /** Creates a new NoteHolderOutCmd. */
  public NoteHolderOutCmd(NoteHolderSubsystem Holder) {
    holderSubsystem = Holder;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(Holder);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //TODO create a constant
    holderSubsystem.ActivateShooterVelocity(1);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    holderSubsystem.StopRoller();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
