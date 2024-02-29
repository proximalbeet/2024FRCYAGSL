// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class RollerSubsystem extends SubsystemBase {
  public CANSparkMax rollerMotor;
  private SparkPIDController pidRoller;

  /** Creates a new RollerSubsystem. */
  public RollerSubsystem() {
    rollerMotor = new CANSparkMax(12, MotorType.kBrushless);
    pidRoller = rollerMotor.getPIDController();

  }

  public void ActivateShooterVelocity(double rmpRoler){
    pidRoller.setReference(rmpRoler, ControlType.kVelocity);
  }

  public void ActivateShooterVelocity(){
    pidRoller.setReference(Constants.Roller.rollerHoldVelocity, ControlType.kVelocity);
  }

  public void StopRoller(){
    rollerMotor.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
