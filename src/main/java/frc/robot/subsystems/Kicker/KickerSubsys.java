// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Kicker;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.utils.Kraken;


public class KickerSubsys extends SubsystemBase {
  private Kraken bottomKickerMotor;//Establishes the Bottom kicker motor
  private Kraken topKickerMotor;//Establishes the Top kicker motor
  // private DigitalInput beamBreak;//Establishes the beam break
  /** Creates a new Kicker. */
  public KickerSubsys() {
    // beamBreak = new DigitalInput(Constants.SensorIDs.sensorID);//Creates a new beam break
    
    bottomKickerMotor  = new Kraken(Constants.KickerConstansts.bottomKickerMotorID);//Creates a new bottom kicker motor
    // topKickerMotor = new Kraken(Constants.KickerConstansts.topKickerMotorID);//Creates a new top kicker motor
  }
  /**
   * Kicks the fuel into the turret.
   * @param speed
   */
  public void kickFuel() {
    bottomKickerMotor.setMotorSpeed(Constants.KickerConstansts.bottomKickerSpeed);//Set the bottom kicker motor's speed to a constant
    // topKickerMotor.setMotorSpeed(Constants.KickerConstansts.topKickerMotorSpeedKick);//Set the top kicker motor's speed to a constant
  }
  /**
   * Expels the fuel to the ground
   */
  public void expelFuel() {
    bottomKickerMotor.setMotorSpeed(Constants.KickerConstansts.bottomKickerSpeed);//Set the bottom kicker motor's speed to a constant
    // topKickerMotor.setMotorSpeed(Constants.KickerConstansts.topKickerMotorSpeedExpel);//Set the top kicker motor's speed to a constant
  }
  /**
   * Stops all kicker motors
   */
  public void stopKickerMotors() {
    bottomKickerMotor.stopMotor();//Stops the bottom kicker motor
    // topKickerMotor.stopMotor();//Stops the top kicker motor
  }
  
  /**
   * Determines if fuel is in the Kicker
   * @return
   */
  // public boolean isFuelInKicker() {
  //   return beamBreak.get();//Returns true if beam is broken and false if beam is not
  // }

  @Override
  public void periodic() {
    // SmartDashboard.putBoolean("is fuel in the kicker", isFuelInKicker());
    // This method will be called once per scheduler run
  }
}
