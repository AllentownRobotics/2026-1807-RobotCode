// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Shooter;

import com.ctre.phoenix6.signals.MotorAlignmentValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.utils.Kraken;

public class FlywheelSubsys extends SubsystemBase {
  /** Creates a new Flywheel. */
  private Kraken leftFlywheelKraken;
  private double flywheelTolerance;
  private Kraken rightFlywheelKraken;
  private double targetVelocity;

  public FlywheelSubsys() {
    flywheelTolerance = 0.1; // rps
    leftFlywheelKraken = new Kraken(46);
    rightFlywheelKraken = new Kraken(40);
    rightFlywheelKraken.follow(45, MotorAlignmentValue.Opposed);

    leftFlywheelKraken.setCoastMode();
    leftFlywheelKraken.setRotorToSensorRatio(1);
    leftFlywheelKraken.setSensorToMechanismRatio(1); // needs to be changed

    leftFlywheelKraken.setPIDValues(0.25, 0, 0, 0.14, 0.12, 0.02, 0);

    SmartDashboard.putNumber("Target Velocity", 0);
  }

  /** sets flywheel to a user wanted velocity from smart dash */
  public void setFlywheelVelocity() {
    targetVelocity = 26; // realistically reaches 25 with current KV
        // SmartDashboard.getNumber("Target Velocity", 0); // gets velocity from smart dash
    targetVelocity =
        MathUtil.clamp(
            targetVelocity, 0, 1000); // clamps so velocity can't ever be below 0 or above 5 rps.
    leftFlywheelKraken.setVelocity(targetVelocity); // sets the velocity for the motor to get to
  }

  /**
   * Checks if flywheel is in a tolerable range of where it needs to be.
   *
   * @return boolean - true or false depending on if its there or not.
   */
  public boolean isFlywheelAtVelocity() {
    if ((Math.abs(targetVelocity - (leftFlywheelKraken.currentVelocityInRPS()))
        <= flywheelTolerance)) {
      return true;
    } else {
      return false;
    }
  }

  public void setIdleSpeed() {
    targetVelocity =
        SmartDashboard.getNumber("Target Velocity", 0); // gets velocity from smart dash
    leftFlywheelKraken.setVelocity(targetVelocity); // sets the velocity for the motor to get to
  }

  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Target Flywheel Velocity", targetVelocity);
    SmartDashboard.putNumber(
        "Current Flywheel velocity (rps)", leftFlywheelKraken.currentVelocityInRPS());
  }
}
