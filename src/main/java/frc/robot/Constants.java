// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;


/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static final class SwerveConstants {
  
  

  // public static final double kWheelDiameterMeters = Units.inchesToMeters(4);
  //       public static final double kDriveMotorGearRatio = 1 / 6.75;
  //       public static final double kTurningMotorGearRatio = 70 / 150;
  }
  

  public static class OIConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kDriverControllerPort2 = 1;

    public static final int kPresetHeadingZeroButton = 2;
    //TODO find better buttons
    public static final int kPickupButton = 3;
    public static final int kZeroGyroButton = 2;
    public static final int kShootoutButton = 1;
    public static final int kShootinButton = 2;
    public static final int kLimelightArmButton = 4;
    public static final int kRotateToAprilTagButton = 5;



    public static final int kDriveXAxis = 1;
    public static final int kDriveYAxis = 0;
    public static final int kRotAxis = 2;

    /** Deadband for the driving axes, 0.5 = 50% */
    public static final double kDriveDeadband = 0.05;
    /** Deadband for the turning axis, 0.5 = 50% */
    public static final double kRotDeadband = 0.05;
  }

    // Pathplanner
  public static final class Swerve {
    /** Max speed of the robot, in meters per second. */
    public static final double kMaxSpeed = 17;

    /** Max rot speed of drive base in radians per second. */
    public static final double kRotSpeed =  17;
    //public static final double kRotSpeed =  0.05 * (2 * Math.PI);

        //TODO! Configure the path follower
        public static final HolonomicPathFollowerConfig kPathPlannerConfig = 
            new HolonomicPathFollowerConfig(
                // Best Values So Far: kP: 3.14, kI: 0, kD: 0.1
                /*
                new PIDConstants(4.5, 0, 0.05),
                new PIDConstants(3.14, 0, 0.1),
                */
                new PIDConstants(4.5, 0, 0.06),
                new PIDConstants(5.5, 0, 0.01),
                //TODO dont know if this needs to be updated later or 
                //TODO ask nate what this does (Idk either)
                4.5,
                0.4,
                new ReplanningConfig(false, true)
            );
    }
    public static final class Shooter {
      //TODO find replacement numbers for shooter motors
      public static final double leftShooterMotorIn = 1;
      public static final double rightShooterMotorIn = 1;

      public static final double leftShooterMotorOut = -1;
      public static final double rightShooterMotorOut = -1;

       //TODO! adjust for shooter
       public static final double SHOOTER_HEIGHT = 0;

       public static final double DEFAULT_ANGLE = 0.34;

       public static final double SPIN_UP_TIME = 0.25;
       
    }

     public static final class Field {

      public static final double BLUE_SPEAKER_X = 0;
      public static final double BLUE_SPEAKER_Y = 5.547879;
      public static final double BLUE_SPEAKER_Z = 2.032004;

      public static final double RED_SPEAKER_X = 16.57938;
      public static final double RED_SPEAKER_Y = 5.547879;
      public static final double RED_SPEAKER_Z = 2.032004;
    }

    public static final class Lighting {
      public static final int lightPort = 0;
    }

    

}
