// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Kicker;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.utils.Kraken;

public class Kicker extends SubsystemBase {
  private Kraken bottomKickerMotor;
  private Kraken topKickerMotor;
  private DigitalInput beamBreak;
  private double motorVelocity;
  private boolean motorSpinning;
  /** Creates a new Kicker. */
  public Kicker() {
    beamBreak = new DigitalInput(Constants.SensorIDs.sensorID);
    
    bottomKickerMotor  = new Kraken(Constants.KickerConstansts.bottomKickerMotorIDConstants);
    topKickerMotor = new Kraken(Constants.KickerConstansts.topKickerMotorIDConstants);
  }
  /**
   * Kicks the fuel into the turret.
   * @param speed
   */
  public void kickFuel() {
    bottomKickerMotor.setMotorSpeed(Constants.KickerConstansts.bottomKickerSpeed);
    topKickerMotor.setMotorSpeed(Constants.KickerConstansts.topKickerMotorSpeedKick);
  }

  public void expelFuel() {
    bottomKickerMotor.setMotorSpeed(Constants.KickerConstansts.bottomKickerSpeed);
    topKickerMotor.setMotorSpeed(Constants.KickerConstansts.topKickerMotorSpeedExpel);
  }

  public void stopMotors() {
    bottomKickerMotor.stopMotor();
    topKickerMotor.stopMotor();
  }

  public boolean getSensorValue() {
    if (motorVelocity == 0) {
      motorSpinning = false;
    } else {
      motorSpinning = true;
    }
    return motorSpinning;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
