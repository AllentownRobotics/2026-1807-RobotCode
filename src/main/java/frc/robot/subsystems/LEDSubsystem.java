// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.EnumMap;
import java.util.Optional;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LEDConstants;

public class LEDSubsystem extends SubsystemBase {
  /** Creates a new LEDSubsystem. */
  Spark blinkin; //establishes blinkin

  EnumMap<LEDConstants.LEDPatterns, Double> patternmap = new EnumMap<>(LEDConstants.LEDPatterns.class); //creates new enummap
  /**
     *put blinkin constants on the pattern map
     */
  public LEDSubsystem() {
    //make a new spark for blinkin
    blinkin = new Spark(LEDConstants.blinkinID);
    
    patternmap.put(LEDConstants.LEDPatterns.OFF, LEDConstants.off);
    patternmap.put(LEDConstants.LEDPatterns.IDLE, LEDConstants.idle);
    patternmap.put(LEDConstants.LEDPatterns.ALIGNED_WITH_HUB, LEDConstants.alignedWithHub);
    patternmap.put(LEDConstants.LEDPatterns.CLIMB_COMPLETE_RED, LEDConstants.climbCompleteRed);
    patternmap.put(LEDConstants.LEDPatterns.CLIMB_COMPLETE_BLUE, LEDConstants.climbCompleteBlue);
    patternmap.put(LEDConstants.LEDPatterns.PIVOT_AT_INTAKE_POSITION, LEDConstants.pivotAtIntakePosition);
    patternmap.put(LEDConstants.LEDPatterns.PIVOT_AT_HOME_POSITION, LEDConstants.pivotAtHomePosition);
    patternmap.put(LEDConstants.LEDPatterns.FUEL_READY_TO_SHOOT, LEDConstants.fuelReadyToShoot);
    patternmap.put(LEDConstants.LEDPatterns.FUEL_IN_TWINDEXER, LEDConstants.fuelInTwindexer);
    patternmap.put(LEDConstants.LEDPatterns.ALIGNED_WITH_HUMAN_PLAYER_STATION, LEDConstants.alignedWithHumanPlayerStation);

    blinkin.set(LEDConstants.idle);//sets the idle blinkin constant
  }

  public void setPattern(LEDConstants.LEDPatterns robotStatePattern){
    double pattern;//establishes pattern
    pattern = robotStatePattern.value;//states the pattern value

    /*if (robotStatePattern == LEDConstants.LEDPatterns.CLIMB_COMPLETE){
      pattern = patternmap.get(robotStatePattern);
      Optional<Alliance> alliance = DriverStation.getAlliance();
      if(alliance.isPresent()){
        if (alliance.get()== Alliance.Red){
          pattern = patternmap.get(LEDConstants.LEDPatterns.CLIMB_COMPLETE_RED);
        }
        if(alliance.get() == Alliance.Blue){
          pattern = patternmap.get(LEDConstants.LEDPatterns.CLIMB_COMPLETE_BLUE);
        }
      }
    }*/

    blinkin.set(pattern);//sets the pattern
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}