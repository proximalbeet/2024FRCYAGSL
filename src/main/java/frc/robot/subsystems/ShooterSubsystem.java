// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {
  /** Creates a new ShooterSubsystem. */
    public CANSparkMax leftShooter;
    public CANSparkMax rightShooter;

  public ShooterSubsystem() {
    leftShooter = new CANSparkMax(10, MotorType.kBrushless);
    rightShooter = new CANSparkMax(11, MotorType.kBrushless);
    rightShooter.setInverted(true);
  }

  public void SetLeftShooterSpeed(double percent) {
    leftShooter.set(percent);
  }

  public void SetRightShooterSpeed(double percent) {
    rightShooter.set(percent);
  }

  public void ActivateShooter(double leftPercent, double rightPercent){
    leftShooter.set(leftPercent);
    rightShooter.set(rightPercent);
  }

  public void DeactivateShooter(){
    leftShooter.set(0);
    rightShooter.set(0);
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
