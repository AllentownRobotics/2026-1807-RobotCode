// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Shooter;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.utils.Kraken;

public class FlywheelSubsys extends SubsystemBase {
  /** Creates a new Flywheel. */
  private Kraken leftFlywheelKraken;
  private double flywheelTolerance;
  private Kraken rightFlywheelKraken;
  private double targetVelocity;
  

  private TalonFX leftFlywheel;
  private TalonFX rightFlywheel;

  public FlywheelSubsys() {
    flywheelTolerance = 0.1; // rps
    // leftFlywheelKraken = new Kraken(46);
    // rightFlywheelKraken = new Kraken(41);
    // rightFlywheelKraken.follow(46, MotorAlignmentValue.Opposed);
    // leftFlywheelKraken.setCoastMode();
    // leftFlywheelKraken.setRotorToSensorRatio(1);
    // leftFlywheelKraken.setSensorToMechanismRatio(1); // needs to be changed

    leftFlywheel = new TalonFX(46);
    rightFlywheel = new TalonFX(41);
    rightFlywheel.setControl(new Follower(46, MotorAlignmentValue.Opposed));
    var config = new TalonFXConfiguration();

    config.Slot0.kP = 999999;
    config.Slot0.kD = 0.1;
    config.Slot0.kV = 8;
    // config.Slot0.kD = 0.1;
    // config.Slot0.
    // config.TorqueCurrent.PeakForwardTorqueCurrent = 50;
    // config.TorqueCurrent.PeakReverseTorqueCurrent = 0;
    config.MotorOutput.PeakForwardDutyCycle = 1;
    config.MotorOutput.PeakReverseDutyCycle = 0;
    config.MotorOutput.withNeutralMode(NeutralModeValue.Coast);

    // leftFlywheel.setControl(new VelocityDutyCycle(10));
    leftFlywheel.getConfigurator().apply(config);
    rightFlywheel.getConfigurator().apply(config);
    // rightFlywheel.setControl(new Follower(46, MotorAlignmentValue.Opposed));




    // var config = new TalonFXConfiguration();

    

    // leftFlywheelKraken.setPIDValues(0.25, 0, 0, 1.5, 0.12, 0.02, 0);

    SmartDashboard.putNumber("Target Velocity", 0);
  }

  /** sets flywheel to a user wanted velocity from smart dash */
  public void setFlywheelVelocity() {
    // targetVelocity = 26; // realistically reaches 25 with current KV
    //     // SmartDashboard.getNumber("Target Velocity", 0); // gets velocity from smart dash
    // targetVelocity =
    //     MathUtil.clamp(
    //         targetVelocity, 0, 1000); // clamps so velocity can't ever be below 0 or above 5 rps.
    // leftFlywheelKraken.setVelocity(targetVelocity); // sets the velocity for the motor to get to
    leftFlywheel.setControl(new VelocityDutyCycle(28).withUpdateFreqHz(300));
    // leftFlywheel.setControl(new velocity)
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
    // targetVelocity =
    //     SmartDashboard.getNumber("Target Velocity", 0); // gets velocity from smart dash
    // leftFlywheelKraken.setVelocity(targetVelocity); // sets the velocity for the motor to get to
    leftFlywheel.setControl(new VelocityDutyCycle(0));
  }

  public void stopSpeed(){
    leftFlywheel.stopMotor();
  }

  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Target Flywheel Velocity", targetVelocity);
    SmartDashboard.putNumber(
        "Current Flywheel velocity (rps)", leftFlywheel.getVelocity().getValueAsDouble());
    SmartDashboard.putNumber("motor applied current", rightFlywheel.getVelocity().getValueAsDouble());
  }
}
