// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.io.File;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import swervelib.SwerveDrive;
import swervelib.parser.SwerveParser;

public class SwerveSubsystem extends SubsystemBase {

  public SwerveDrive swerveDrive;

  /** Creates a new ExampleSubsystem. */
  public SwerveSubsystem() {
    try {
      swerveDrive = new SwerveParser(new File(Filesystem.getDeployDirectory(), "swerve")).createSwerveDrive(Constants.SwerveConstants.kMaxSpeed);
    } catch(Exception e) {
      throw new RuntimeException(e);
    }
    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
