// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.IntakeSubsystem;

public class PickupCmd extends Command {
  public IntakeSubsystem intakeSubsystem;
  /** Creates a new PickupCmd. */
  public PickupCmd(IntakeSubsystem Intake) {
    // Use addRequirements() here to declare subsystem dependencies.
    intakeSubsystem = Intake;
    addRequirements(Intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //TODO create a constant
    intakeSubsystem.ActivateIntakeVelocity(Constants.Shooter.leftShooterMotorIn, Constants.Shooter.rightShooterMotorIn);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intakeSubsystem.StopIntake();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
