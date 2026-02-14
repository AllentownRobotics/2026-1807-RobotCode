// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Kicker;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.utils.Kraken;

public class Kicker extends SubsystemBase {
  private Kraken bottomKickerMotor;
  private Kraken topKickerMotor;
  /** Creates a new Kicker. */
  public Kicker() {
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

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
