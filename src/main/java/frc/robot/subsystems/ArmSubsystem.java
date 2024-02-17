// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAlternateEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmSubsystem extends SubsystemBase {
  
  public CANSparkMax ArmMotor;
  public RelativeEncoder throughBoreAbsoluteEncoder;
  public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput;

  SparkPIDController armPID;
  public double desiredArmPosition;
  public ArmSubsystem(){
        ArmMotor = new CANSparkMax(9, MotorType.kBrushless);
        throughBoreAbsoluteEncoder = ArmMotor.getAlternateEncoder(SparkMaxAlternateEncoder.Type.kQuadrature,8192);
        armPID = ArmMotor.getPIDController();
        ArmMotor.setIdleMode(IdleMode.kBrake);

        /**
         * By default, the PID controller will use the Hall sensor from a NEO for its
         * feedback device. Instead, we can set the feedback device to the alternate
         * encoder object
         */
        armPID.setFeedbackDevice(throughBoreAbsoluteEncoder);
    
        /**
         * From here on out, code looks exactly like running PID control with the 
         * built-in NEO encoder, but feedback will come from the alternate encoder
         */ 
    
        // PID coefficients
        kP = 0.1; 
        kI = 1e-4;
        kD = 1; 
        kIz = 0; 
        kFF = 0; 
        kMaxOutput = 1; 
        kMinOutput = -1;
    
        // set PID coefficients
        armPID.setP(kP);
        armPID.setI(kI);
        armPID.setD(kD);
        armPID.setIZone(kIz);
        armPID.setFF(kFF);
        armPID.setOutputRange(kMinOutput, kMaxOutput);
    
        // display PID coefficients on SmartDashboard
        SmartDashboard.putNumber("P Gain", kP);
        SmartDashboard.putNumber("I Gain", kI);
        SmartDashboard.putNumber("D Gain", kD);
        SmartDashboard.putNumber("I Zone", kIz);
        SmartDashboard.putNumber("Feed Forward", kFF);
        SmartDashboard.putNumber("Max Output", kMaxOutput);
        SmartDashboard.putNumber("Min Output", kMinOutput);
        SmartDashboard.putNumber("Set Rotations", 0);
  }
   
  @Override
  public void periodic() {
 
   SmartDashboard.putNumber("armPosition", throughBoreAbsoluteEncoder.getPosition());
  }

  public void driveArm(double position) {

    armPID.setReference(position, CANSparkMax.ControlType.kPosition);

  }

  public double getArmPosition() { return throughBoreAbsoluteEncoder.getPosition();}

}

 // /** Creates a new Arm. */null
  // public void SetArmSpeed(double percent) {
  //   ArmMotor.set(percent);
  //   SmartDashboard.putNumber("arm power (%)", percent);
  //   SmartDashboard.putNumber("arm motor current (amps)", ArmMotor.getOutputCurrent());
  //   SmartDashboard.putNumber("arm motor temperature (C)", ArmMotor.getMotorTemperature());
  // }
