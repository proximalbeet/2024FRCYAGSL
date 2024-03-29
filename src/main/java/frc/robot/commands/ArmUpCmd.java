// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;

public class ArmUpCmd extends Command {
  ArmSubsystem armSubsystem;
  /** Creates a new ArmUpCmd. */
  public ArmUpCmd(ArmSubsystem Arm) {
    armSubsystem = Arm;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(Arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
   
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
     armSubsystem.driveArm(0.2);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    armSubsystem.driveArm(0.15);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return armSubsystem.getArmPosition() > .2;
  }
}
