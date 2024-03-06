// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.NoteHolderSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html

public class ScoringSequentialCmd extends SequentialCommandGroup {
  ArmSubsystem armSubsystem;
  ShooterSubsystem shooterSubsystem;
  NoteHolderSubsystem noteHolderSubsystem;

  //ShooterSubsystem
  /** Creates a new ScoringSequentialCmd. */
  public ScoringSequentialCmd(ArmSubsystem arm, ShooterSubsystem shooter, NoteHolderSubsystem holder) {
    armSubsystem = arm;
    shooterSubsystem = shooter;
    noteHolderSubsystem = holder;
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    BooleanSupplier bS = () -> {
      return armSubsystem.isInRange(Constants.Shooter.DEFAULT_ANGLE);
    };

    addCommands(

    new ArmUpCmd(arm),

    new WaitUntilCommand(bS),

    new ShooterOutCmd(shooter),

    //TODO find good value
    new WaitCommand(Constants.Shooter.SPIN_UP_TIME),
 
    new NoteHolderCmd(holder)
    );
  }
}
