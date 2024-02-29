// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.RollerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class PickupCmd extends Command {
  public  ArmSubsystem armSubsystem;
  public  ShooterSubsystem shooterSubsystem;
  public RollerSubsystem rollerSubsystem;
  /** Creates a new PickupCmd. */
  public PickupCmd(ArmSubsystem Arm, ShooterSubsystem Shooter, RollerSubsystem rollerSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(Arm, Shooter, rollerSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //TODO find the perfect ground position to pick up from the ground
    armSubsystem.driveArm(0);
    shooterSubsystem.ActivateShooter(Constants.Shooter.leftShooterMotorIn, Constants.Shooter.rightShooterMotorIn);
    rollerSubsystem.ActivateShooterVelocity(Constants.Roller.rollerInputVelocity);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //TODO find a spot for the arm to sit at when command is done
    shooterSubsystem.DeactivateShooter();
    armSubsystem.driveArm(1000);
    rollerSubsystem.ActivateShooterVelocity();

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
