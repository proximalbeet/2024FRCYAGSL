// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Misc;

import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class LightingSubsystem extends SubsystemBase {
   PWMSparkMax LEDSetting = null;

  /** Creates a new LEDLights. */
  public LightingSubsystem() {
    LEDSetting = new PWMSparkMax(Constants.Lighting.lightPort);
    
  }
    
  //TODO add this function to every action
  public void setLEDValue(double setColor) {
    LEDSetting.set(setColor);
}

public double getLEDLights() {
  return LEDSetting.get();
}

}
