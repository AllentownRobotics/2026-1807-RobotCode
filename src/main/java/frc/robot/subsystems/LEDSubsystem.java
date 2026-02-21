// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.EnumMap;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.blinkinConstants;

public class LEDSubsystem extends SubsystemBase {
  /** Creates a new LEDSubsystem. */
  Spark blinkin;

  EnumMap<blinkinConstants.LEDPatterns, Double> patternmap = new EnumMap<>(blinkinConstants.LEDPatterns.class);
  /**
     *put blinkin constants on the pattern map
     */
  public LEDSubsystem() {
    //make a new spark for blinkin
    blinkin = new Spark(blinkinConstants.blinkinID);
    
    blinkin = new Spark(blinkinConstants.blinkinID);

    patternmap.put(blinkinConstants.LEDPatterns.OFF, blinkinConstants.off);
    patternmap.put(blinkinConstants.LEDPatterns.IDLE, blinkinConstants.defaultColor);
    patternmap.put(blinkinConstants.LEDPatterns.ALIGNED_WITH_HUB, blinkinConstants.alignedWithHub);
    patternmap.put(blinkinConstants.LEDPatterns.CLIMB_COMPLETE_RED, blinkinConstants.climbCompleteRed);
    patternmap.put(blinkinConstants.LEDPatterns.CLIMB_COMPLETE_BLUE, blinkinConstants.climbCompleteBlue);
    patternmap.put(blinkinConstants.LEDPatterns.PIVOT_AT_INTAKE_POSITION, blinkinConstants.pivotAtIntakePosition);
    patternmap.put(blinkinConstants.LEDPatterns.PIVOT_AT_HOME_POSITION, blinkinConstants.pivotAtHomePosition);
    patternmap.put(blinkinConstants.LEDPatterns.FUEL_READY_TO_SHOOT, blinkinConstants.fuelInTwindexer);
    patternmap.put(blinkinConstants.LEDPatterns.FUEL_IN_TWINDEXER, blinkinConstants.fuelInTwindexer);
    patternmap.put(blinkinConstants.LEDPatterns.ALIGNED_WITH_HUMAN_PLAYER_STATION, blinkinConstants.alignedWithHumanPlayerStation);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}