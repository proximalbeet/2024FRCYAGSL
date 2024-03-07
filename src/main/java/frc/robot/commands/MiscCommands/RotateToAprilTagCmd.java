// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.MiscCommands;

import java.util.Optional;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Misc.LimelightSubsystem;
import frc.robot.subsystems.SwerveSubsystem;


public class RotateToAprilTagCmd extends Command {

  LimelightSubsystem limelight;
  SwerveSubsystem swerveDrive;
  Pose2d measuredPose;
  Pose2d currentPose;
  private DoubleSupplier moveX, moveY;
  double lastUpdateTime;
  Rotation2d desiredYaw;

  /** Drive command for aiming at the speaker while moving. 
   * measuredPose = pose measured from Limelight
   * currentPose = the pose the robot believes it is at
   * 
   * moveX = main driver x axis joystick inputs
   * moveY = main driver y axis joystick inputs
  */
  public RotateToAprilTagCmd(LimelightSubsystem limelight, SwerveSubsystem swerveDrive, DoubleSupplier vX, DoubleSupplier vY) {
    this.limelight = limelight;
    this.swerveDrive = swerveDrive;

    measuredPose = limelight.getMeasuredPose();
    currentPose = swerveDrive.getPose();

    moveX = vX;
    moveY = vY;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(limelight, swerveDrive);
  }

  @Override
  public void initialize() {
    // allows the robot to use rotate to pose
    swerveDrive.setHeadingCorrection(true);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Sets time measurement recieved
    double timestamp = Timer.getFPGATimestamp() - limelight.getLatency();

    // Makes joystick inputs useable for command
    double correctedMoveX = Math.pow(moveX.getAsDouble(), 3) * Constants.Swerve.kMaxSpeed;
    double correctedMoveY = Math.pow(moveY.getAsDouble(), 3) * Constants.Swerve.kMaxSpeed;
    
    // Only check limelight once per second
    if (timestamp - lastUpdateTime >= 1) {
      measuredPose = limelight.getMeasuredPose();

      // Two tag measuring is stable, one tag measuring is not
      if(limelight.hasTarget() && limelight.tagCount() >= 2) {
        swerveDrive.addVisionMeasurement(new Pose2d(measuredPose.getX(), measuredPose.getY(), measuredPose.getRotation()), timestamp, VecBuilder.fill(0.01, 0.01, 999999999));
        lastUpdateTime = timestamp;
     }
    }

    swerveDrive.updatePose();

    // Relative positions from the robot to the speaker.
    double offsetX;
    double offsetY;
    double offsetZ;

    // Can't aim towards our shooter if we don't know what team we're on.
    Optional<Alliance> alliance = DriverStation.getAlliance();
    if(alliance.isEmpty()) {
      return;
    }

    // Calculate XY offset between robot and speaker,
    // convert to angle, then convert to field-centric angle
    if(alliance.get() == Alliance.Blue) {
      offsetX = Constants.Field.BLUE_SPEAKER_X - currentPose.getX();
      offsetY = Constants.Field.BLUE_SPEAKER_Y - currentPose.getY();
      offsetZ = Constants.Field.BLUE_SPEAKER_Z - Constants.Shooter.SHOOTER_HEIGHT;
    } else {
      offsetX = Constants.Field.RED_SPEAKER_X - currentPose.getX();
      offsetY = Constants.Field.RED_SPEAKER_Y - currentPose.getY();
      offsetZ = Constants.Field.RED_SPEAKER_Z - Constants.Shooter.SHOOTER_HEIGHT;
    }

    // calculate direct distance and ground distance to the speaker
    double floorDistance = Math.hypot(offsetX, offsetY);
    double directDistance = Math.hypot(floorDistance, offsetZ);

    // calculate pitch and yaw from the shooter to the speaker
    desiredYaw = new Rotation2d(offsetX, offsetY);
    
    // Deadband so we don't oscillate
    if (Math.abs(currentPose.getRotation().getDegrees() - desiredYaw.getDegrees()) > 0.25) {
      swerveDrive.driveAbsolute(correctedMoveX, correctedMoveY, desiredYaw);
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swerveDrive.setHeadingCorrection(false);
  }

  // Returns true when the command should end during autonomous.
  @Override
  public boolean isFinished() {
    if (DriverStation.isAutonomousEnabled() && Math.abs(swerveDrive.getYaw().getDegrees() - desiredYaw.getDegrees()) > 0.25) {
      return true;
    } else {
      return false;
    }
  }
}