// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class NoteHolderSubsystem extends SubsystemBase {
   public CANSparkMax rollerMotor;
   public SparkPIDController pidRoller;
  /** Creates a new NoteHolderSubsystem. */
  public NoteHolderSubsystem() {
    //TODO create a constant
    rollerMotor = new CANSparkMax(12, MotorType.kBrushless);
    pidRoller = rollerMotor.getPIDController();
  }

  public void ActivateShooterVelocity(double rmpRoler){
    pidRoller.setReference(rmpRoler, ControlType.kVelocity);
  }

  public void ActivateShooterVelocity(){
    //TODO create a constant
    pidRoller.setReference(0.2, ControlType.kVelocity);
  }

  public void StopRoller(){
    rollerMotor.set(0);
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
