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

  public void ActivateHolderVelocity(double rmpRoler){
    pidRoller.setReference(rmpRoler, ControlType.kVelocity);
  }



  public void StopHolder(){
    rollerMotor.set(0);
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
