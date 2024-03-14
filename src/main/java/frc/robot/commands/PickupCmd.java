// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.NoteHolderSubsystem;

public class PickupCmd extends Command {
  public IntakeSubsystem intakeSubsystem;
  public ArmSubsystem armSubsystem;
  public NoteHolderSubsystem noteHolderSubsystem;;
  /** Creates a new PickupCmd. */
  public PickupCmd(IntakeSubsystem Intake, ArmSubsystem Arm, NoteHolderSubsystem NoteHolder) {
    // Use addRequirements() here to declare subsystem dependencies.
    intakeSubsystem = Intake;
    armSubsystem = Arm;
    noteHolderSubsystem = NoteHolder;
    addRequirements(Intake, Arm, NoteHolder);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //TODO create a constant
    intakeSubsystem.ActivateIntakeVelocity(Constants.Shooter.leftShooterMotorIn, Constants.Shooter.rightShooterMotorIn);
    //Find good angle
    armSubsystem.driveArm(0.15);
    noteHolderSubsystem.ActivateHolderVelocity(1);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intakeSubsystem.StopIntake();
    noteHolderSubsystem.StopHolder();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
