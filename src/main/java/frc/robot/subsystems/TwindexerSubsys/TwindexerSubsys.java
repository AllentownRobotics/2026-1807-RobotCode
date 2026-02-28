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
  private final Kraken twindexerMotor; // The motor that powers the twindexer.
  private final PIDController twindexerPIDController; // The PID controller used on the twindexer motor.
  private final DigitalInput hopperFullBeambreak; // The beambreak at the top of the hopper that returns values representing whether the hopper is full of fuel or not.
  
  /**
   * Boolean that returns true if the twindexer is spinning, and false if it is not spinning.
   */
  private boolean isTwindexerSpinning;

  /**
   * The current speed of the twindexer.
   */
  private double twindexerSpeed;

  /**
   * Returns true if hopper beambreak is broken, and false if it is not broken.
   */
  private boolean isHopperFull;

  public TwindexerSubsys() {
    hopperFullBeambreak = new DigitalInput(Constants.TwindexerConstants.fullBeambreakID); // Assigns beambreak ID to hopper beambreak.

    twindexerPIDController = new PIDController(
      // Assigns PID constants to PIDController.
      Constants.TwindexerConstants.kp, 
      Constants.TwindexerConstants.ki, 
      Constants.TwindexerConstants.kd); 
    
    twindexerMotor = new Kraken(Constants.MotorIDs.twindexerMotor); // Assigns motor ID to twindexer motor.
    twindexerMotor.setCoastMode();
  }

  /**
   * Sets the desired speed (setpoint) for the twindexer PID.
   */
  public void setTwindexerDesiredSpeed(double speed) {
    twindexerPIDController.setSetpoint(Constants.TwindexerConstants.desiredTwindexerSpeed);//
  }

  /**
   * Sets twindexer motor to a desired speed using PID with our setpoint and values.
   */
  public void setTwindexerSpeed() {
    // Calculates motor speed by getting motor velocity and using PID.
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

    // Checks twindexerSpeed to see if it is above 0. Returns true if it is, and false if it is 0.
    if (twindexerSpeed > 0) {
      motorSpinning = true;
    } else {
      motorSpinning = false;
    }//unneccessary

    return motorSpinning;
  }

  /**
   * Sets twindexer motor speed to 0 so that it slows to a stop.
   */
  public void stopTwindexer() {
    // Commented code is using PID to stop the twindexer (currently not necessary because motor is on coast mode).
    //
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

    //Displays whether or not the twindexer is spinning as a boolean.
    SmartDashboard.putBoolean("Twindexer is spinning: ", isTwindexerSpinning);

    //Displays whether or not the hopper beambreak is broken as a boolean.
    SmartDashboard.putBoolean("Is the hopper full: ", isHopperFull);
  }
}
