// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.io.File;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.hal.HALUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import swervelib.SwerveController;
import swervelib.SwerveDrive;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

public class SwerveSubsystem extends SubsystemBase {

  public SwerveDrive inner;
  public static int robotTeamNumber = HALUtil.getTeamNumber();
  public static Alliance allianceColor;
  private Pigeon2 _pigeon = new Pigeon2(20);
  // public static Pigeon2 gyro;

  /** Creates a new ExampleSubsystem. */
  public SwerveSubsystem() {
    //TODO find can id
    // gyro = new Pigeon2(20);
      // Set the verbosity before initializing the swerve drive.
    SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;
    try {
      inner = new SwerveParser(new File(Filesystem.getDeployDirectory(), "swerve")).createSwerveDrive(Constants.Swerve.kMaxSpeed);
    } catch(Exception e) {
      throw new RuntimeException(e);
    }
    configurePathplanner();
  }

   private void configurePathplanner() {
    AutoBuilder.configureHolonomic(
      // Supply methods for Pathplanner to use to control the robot
      this::getPose,
      this::resetOdometry,
      this::getRobotVelocity,
      this::setChassisSpeeds,
      // PID and speed constants
      Constants.Swerve.kPathPlannerConfig,
      () -> {
        // Boolean supplier that controls when the path will be mirrored for the red alliance
        // This should flip the path being followed to the red side of the field.
        var alliance = DriverStation.getAlliance();
        return alliance.isPresent() ? alliance.get() == DriverStation.Alliance.Red : false;
      },
      this
    );
  }
  
  @Override
  public void periodic() {
     // Put the measured team number to the dashboard for diagnostics
    SmartDashboard.putNumber("robotTeamNumber", robotTeamNumber);
    // inner.setGyro(gyro.getRotation3d());
  }

   public Pose2d getPose() { return inner.getPose(); }
  public ChassisSpeeds getRobotVelocity() { return inner.getRobotVelocity(); }
  public void setChassisSpeeds(ChassisSpeeds chassisSpeeds) { inner.setChassisSpeeds(chassisSpeeds); }
  public Rotation2d getYaw() { return inner.getYaw(); }
  

  public void driveAbsolute(double moveX, double moveY, Rotation2d rotation2d) {
    ChassisSpeeds desiredSpeeds = inner.swerveController.getRawTargetSpeeds(
      moveX,
      moveY,
      rotation2d.getRadians(),
      getYaw().getRadians()
    );
    inner.drive(SwerveController.getTranslation2d(desiredSpeeds), desiredSpeeds.omegaRadiansPerSecond, true, false);
  }

   //TODO! Document
  public void setHeadingCorrection(boolean correctionEnabled) {
    inner.setHeadingCorrection(correctionEnabled);
  }

  public void addVisionMeasurement(Pose2d measuredPose, double timestamp, Matrix<N3, N1> stdDeviation) {
    inner.addVisionMeasurement(measuredPose, timestamp, stdDeviation);
  }

  /** Update the pose estimator without a vision reading. */
  public void updatePose() {
    inner.swerveDrivePoseEstimator.update(inner.getYaw(), inner.getModulePositions());
  }

  public void resetOdometry(Pose2d pose) {
    inner.resetOdometry(pose);
  }

  public void zeroGyro() {
    inner.zeroGyro();
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
