// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LimelightSubsystem extends SubsystemBase {
  /** Creates a new Limelight. */
  public LimelightSubsystem() {}
  
  NetworkTable networkTable = NetworkTableInstance.getDefault().getTable("limelight");

  public void initialize(){
  networkTable.getEntry("ledMode").setNumber(0);
  networkTable.getEntry("pipeline").setNumber(4);
  }
  
  /** Gets the measured pose according to the Limelight MegaTag pipeline. */
  public Pose2d getMeasuredPose() {
    double[] rawPose = networkTable.getEntry("botpose_wpiblue").getDoubleArray(new double[6]);

    // Convert raw pose (X, Y, Z, Roll, Pitch, Yaw)
    // into a Pose2d. We don't typically use 3D coords,
    // so Z, Roll, and Pitch are discarded here.
    return new Pose2d(new Translation2d(rawPose[0], rawPose[1]), new Rotation2d(rawPose[5]));
  }

  public Rotation3d getMeasuredRotation() {
    double[] rawPose = networkTable.getEntry("botpose").getDoubleArray(new double[6]);

    // Convert raw pose (X, Y, Z, Roll, Pitch, Yaw) to Rotation3D.
    return new Rotation3d(rawPose[3], rawPose[4], rawPose[5]);
  }

  public double getTargetPose(int tid) {
    if(networkTable.getEntry("tid").getInteger(-1) != tid) {
      return 0;
    }

    return networkTable.getEntry("tx").getDouble(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
