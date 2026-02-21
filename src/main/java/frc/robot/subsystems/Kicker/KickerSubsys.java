// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Kicker;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.utils.Kraken;

public class KickerSubsys extends SubsystemBase {
  private Kraken bottomKickerMotor;//Establishes the Bottom kicker motor
  private Kraken topKickerMotor;//Establishes the Top kicker motor
  private DigitalInput beamBreak;//Establishes the beam break
  private double motorVelocity;//Creates a double called motor velocity
  private boolean motorSpinning;//Creates a boolean called motorSpinning
  /** Creates a new Kicker. */
  public KickerSubsys() {
    beamBreak = new DigitalInput(Constants.SensorIDs.sensorID);//Creates a new beam break
    
    bottomKickerMotor  = new Kraken(Constants.KickerConstansts.bottomKickerMotorIDConstants);//Creates a new bottom kicker motor
    topKickerMotor = new Kraken(Constants.KickerConstansts.topKickerMotorIDConstants);//Creates a new top kicker motor
  }
  /**
   * Kicks the fuel into the turret.
   * @param speed
   */
  public void kickFuel() {
    bottomKickerMotor.setMotorSpeed(Constants.KickerConstansts.bottomKickerSpeed);//Set the bottom kicker motor's speed to a constant
    topKickerMotor.setMotorSpeed(Constants.KickerConstansts.topKickerMotorSpeedKick);//Set the top kicker motor's speed to a constant
  }
  /**
   * Expels the fuel to the ground
   */
  public void expelFuel() {
    bottomKickerMotor.setMotorSpeed(Constants.KickerConstansts.bottomKickerSpeed);//Set the bottom kicker motor's speed to a constant
    topKickerMotor.setMotorSpeed(Constants.KickerConstansts.topKickerMotorSpeedExpel);//Set the top kicker motor's speed to a constant
  }
  /**
   * Stops all motors
   */
  public void stopMotors() {
    bottomKickerMotor.stopMotor();//Stops the bottom kicker motor
    topKickerMotor.stopMotor();//Stops the top kicker motor
  }
  /**
   * Gets the value of the beam break
   */
  public boolean getSensorValue() {
    if (motorVelocity == 0) {
      motorSpinning = false;
    } else {
      motorSpinning = true;
    }
    return beamBreak.get();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
