// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.utils;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PowerMonitor extends SubsystemBase {
  
  private final PowerDistribution pdh;

  /** Creates a new PDH. */
  public PowerMonitor() {
    pdh = new PowerDistribution(1, ModuleType.kRev);
  }

  public void logAllPorts() {
    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    logAllPorts();
  }
}
