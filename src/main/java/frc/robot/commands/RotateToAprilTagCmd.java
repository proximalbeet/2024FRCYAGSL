// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.Optional;
import java.util.function.DoubleSupplier;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;
import swervelib.SwerveController;
import frc.robot.Constants;
import frc.robot.subsystems.LimelightSubsystem;

public class RotateToAprilTagCmd extends Command {
  /** Creates a new RotateToAprilTagCmd. */
  private DoubleSupplier moveX, moveY, turnTheta;
  double desiredPose;
  SwerveSubsystem swerveDrive;
  LimelightSubsystem limelight;


  //public RotateToAprilTagCmd() {

  
    /** Beta command for aiming at an apriltag. */
  public RotateToAprilTagCmd(SwerveSubsystem swerveDrive, LimelightSubsystem limelight, DoubleSupplier vX, DoubleSupplier vY, DoubleSupplier rotate) {
    this.swerveDrive = swerveDrive;
    this.limelight = limelight;
  
    moveX = vX;
    moveY = vY;
    turnTheta = rotate;
  
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(swerveDrive, limelight);

    }
  

  // Called when the command is initially scheduled.
  @Override
  public void initialize() 
  {
    swerveDrive.inner.setHeadingCorrection(true);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double correctedMoveX = Math.pow(moveX.getAsDouble(), 3) * Constants.Swerve.kMaxSpeed;
    double correctedMoveY = Math.pow(moveY.getAsDouble(), 3) * Constants.Swerve.kMaxSpeed;
    double correctedTurnTheta = turnTheta.getAsDouble() * Constants.Swerve.kRotSpeed;

     // Gets apriltag position, if the Limelight returns null (tag not found), return early
     // 4 is red, blue is 7
    Optional<Alliance> ally = DriverStation.getAlliance();
    if (ally.isPresent()) {
        if (ally.get() == Alliance.Red) {
            desiredPose = -limelight.getTargetPose(4);
        }
       if (ally.get() == Alliance.Blue) {
            desiredPose = -limelight.getTargetPose(7);
        }
    }
    else{
      //Use red alliance april tag for testing
      desiredPose = -limelight.getTargetPose(4);
    }
    if(desiredPose == 0) {
      ChassisSpeeds desiredSpeeds = swerveDrive.inner.swerveController.getRawTargetSpeeds(correctedMoveX, correctedMoveY, correctedTurnTheta);
      swerveDrive.inner.drive(SwerveController.getTranslation2d(desiredSpeeds), desiredSpeeds.omegaRadiansPerSecond, true, false);

    }
    else {
     SmartDashboard.putNumber("testDesiredPose", desiredPose);

     double desiredYaw = desiredPose;

     ChassisSpeeds desiredSpeeds = swerveDrive.inner.swerveController.getRawTargetSpeeds(
       correctedMoveX, correctedMoveY,
       swerveDrive.inner.getPose().getRotation().getRadians() + (desiredYaw * Math.PI/180),
       swerveDrive.inner.getPose().getRotation().getRadians()
     );
     swerveDrive.inner.drive(SwerveController.getTranslation2d(desiredSpeeds), desiredSpeeds.omegaRadiansPerSecond, true, false);

    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) 
  {
    swerveDrive.inner.setHeadingCorrection(false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}