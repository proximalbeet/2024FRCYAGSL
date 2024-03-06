// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {
   public CANSparkMax intakeMotorTop, intakeMotorBottom;
  private SparkPIDController pidIntakeTop, pidIntakeBottom;
  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {
    //TODO create a constant
    intakeMotorTop = new CANSparkMax(13, MotorType.kBrushless);
    //TODO create a constant
    intakeMotorBottom = new CANSparkMax(14, MotorType.kBrushless);

  pidIntakeTop = intakeMotorTop.getPIDController();
  
  pidIntakeBottom = intakeMotorBottom.getPIDController();

  intakeMotorBottom.setInverted(true);
 }
   public void ActivateIntakeVelocity(double rpmIntakeTop, double rmpIntakeBottom){
    pidIntakeTop.setReference(rpmIntakeTop, ControlType.kVelocity);
    pidIntakeBottom.setReference(rmpIntakeBottom, ControlType.kVelocity);
  }


  public void StopIntake(){
    intakeMotorTop.set(0);
    intakeMotorBottom.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
