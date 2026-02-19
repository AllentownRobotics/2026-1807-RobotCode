// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.TwindexerSubsys;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.utils.Kraken;

public class TwindexerSubsys extends SubsystemBase {
  /** Creates a new TwindexerSubsystem. */
  private final Kraken twindexerMotor;
  private final PIDController twindexerPIDController;
  private final DigitalInput hopperFullBeambreak;
  /**
   * True if the twindexer is spinning, and false if it is not spinning.
   */
  private boolean isTwindexerSpinning;
  /**
   * The current speed of the twindexer.
   */
  private double twindexerSpeed;
  /**
   * Detects if the hopper is full of fuel by periodically getting values from a beambreak at the top of the hopper.
   */
  private boolean isHopperFull;
  /**
   * this is how to write method/variable descriptions :)
   */
  public TwindexerSubsys() {
    hopperFullBeambreak = new DigitalInput(Constants.TwindexerConstants.fullBeambreak);
    twindexerPIDController = new PIDController(Constants.TwindexerConstants.kp, Constants.TwindexerConstants.ki, Constants.TwindexerConstants.kd);
    twindexerMotor = new Kraken(Constants.MotorIDs.twindexerMotor);
    twindexerMotor.setCoastMode();
  }
  /**
   * Sets the desired speed (setpoint) for the twindexer PID.
   */
  public void setTwindexerDesiredSpeed(double speed) {
    twindexerPIDController.setSetpoint(Constants.TwindexerConstants.desiredTwindexerSpeed);
  }

  /**
   * Sets twindexer motor to a desired speed using our PID setpoint and values.
   * @param speed the motor speed as a double
   */
  public void setTwindexerSpeed() {
    twindexerMotor.setMotorSpeed(
      twindexerPIDController.calculate(
        twindexerMotor.getVelocity()
      ) + twindexerMotor.getVelocity()
    );
  }
  /**
   * Gets the twindexer's velocity from twindexerSpeed and determines if the twindexer is spinning.
   * @return motorSpinning: boolean representing if the twindexer is spinning (true) or if it is not spinning (false)
   */
  public boolean getTwindexerSpinning() {
    boolean motorSpinning;
    //checks twindexerSpeed to see if it is above 0, returns true if it is
    if (twindexerSpeed > 0) {
      motorSpinning = true;
    } else {
      motorSpinning = false;
    }
    return motorSpinning;
  }

  public void stopTwindexer() {
    // twindexerMotor.setMotorSpeed(
    //   twindexerPIDController.calculate(
    //     twindexerMotor.getVelocity(), 0
    //   ) + twindexerMotor.getVelocity()
    // );
    twindexerMotor.setMotorSpeed(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    isHopperFull = !hopperFullBeambreak.get();
    twindexerSpeed = twindexerMotor.getVelocity();
    isTwindexerSpinning = getTwindexerSpinning();
    //displays whether or not the twindexer is spinning as a boolean
    SmartDashboard.putBoolean("Twindexer is spinning: ", isTwindexerSpinning);
    SmartDashboard.putBoolean("Is the hopper full: ", isHopperFull);
  }
}
