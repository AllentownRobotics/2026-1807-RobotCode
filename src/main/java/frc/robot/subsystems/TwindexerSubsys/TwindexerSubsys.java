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

  /** 
   * The motor that powers the twindexer.
   */
  private final Kraken twindexerMotor; 

  /** 
   * The PID controller used on the twindexer motor.
   */ 
  private final PIDController twindexerPIDController; 

  /**
   * The beam break at the top of the hopper that returns values representing whether the hopper is full of fuel or not.
   */
  private final DigitalInput hopperFullBeamBreak; 
  
  /**
   * The beam break on the right side of the hopper (when looking at it from the front) towards the bottom to detect if there is any fuel in the hopper on this side.
   */
  private final DigitalInput rightHopperBeamBreak;

  /**
   * The beam break on the left side of the hopper (when looking at it from the front) towards the bottom to detect if there is any fuel in the hopper on this side.
   */
  private final DigitalInput leftHopperBeamBreak;
  /**
   * Boolean that returns true if the twindexer is spinning, and false if it is not spinning.
   */
  private boolean isTwindexerSpinning;

  /**
   * The current speed of the twindexer.
   */
  private double twindexerSpeed;

  /**
   * Returns true if full hopper beam break is broken, and false if it is not broken.
   */
  private boolean isHopperFull;

  /**
   * Returns true if bottom right hopper beam break is broken, and false if it is not broken.
   */
  private boolean rightHopperHasFuel;

  /**
   * Returns true if bottom left hopper beam break is broken, and false if it is not broken.
   */
  private boolean leftHopperHasFuel;

  /** Creates a new TwindexerSubsystem. */
  public TwindexerSubsys() {
    // Assigns beam break IDs to their corresponding beam breaks.
    hopperFullBeamBreak = new DigitalInput(Constants.TwindexerConstants.fullBeamBreakID);
    rightHopperBeamBreak = new DigitalInput(Constants.TwindexerConstants.rightBeamBreakID);
    leftHopperBeamBreak = new DigitalInput(Constants.TwindexerConstants.leftBeamBreakID);

    twindexerPIDController = new PIDController(
      Constants.TwindexerConstants.kp, 
      Constants.TwindexerConstants.ki, 
      Constants.TwindexerConstants.kd);  // Assigns PID constants to PIDController.
    
    twindexerMotor = new Kraken(Constants.MotorIDs.twindexerMotorID); // Assigns motor ID to twindexer motor.
    twindexerMotor.setCoastMode();
  }

  /**
   * Sets the desired speed (setpoint) for the twindexer PID.
   */
  public void setTwindexerDesiredSpeed(double speed) {
    twindexerPIDController.setSetpoint(Constants.TwindexerConstants.desiredTwindexerSpeed); // Assigns setpoint value to PID controller.
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
   * Sets twindexer motor to a desired speed using PID ONLY if one of the beam breaks detects fuel. Used in auto.
   */
  public void autosSetTwindexerSpeed() {
    if (rightHopperHasFuel || leftHopperHasFuel) {
      twindexerMotor.setMotorSpeed(
        twindexerPIDController.calculate(
          twindexerMotor.getVelocity()
        ) + twindexerMotor.getVelocity()
      );
    } else {
      twindexerMotor.setMotorSpeed(0);
    }
  }

  /**
   * Gets the twindexer's velocity from twindexerSpeed and determines if the twindexer is spinning.
   * @return motorSpinning: boolean representing if the twindexer is spinning (true) or if it is not spinning (false)
   */
  public boolean getTwindexerSpinning() {

    boolean isMotorSpinning = false;
    // Checks twindexerSpeed to see if it is above 0. Returns true if it is, and false if it is 0.
    if (twindexerSpeed > 0) {
      return isMotorSpinning;
    }

    return isMotorSpinning;
  }

  /**
   * Sets twindexer motor speed to 0 so that it slows to a stop.
   */
  public void stopTwindexer() {
    // Commented code is for using PID to stop the twindexer (currently not necessary because motor is on coast mode).
    //
    // twindexerMotor.setMotorSpeed(
    //   twindexerPIDController.calculate(
    //     twindexerMotor.getVelocity(), 0
    //   ) + twindexerMotor.getVelocity()
    // );
    twindexerMotor.setMotorSpeed(0); // Sets motor speed to 0 - coast mode will let it slow to a stop.
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    // Gets the value of all three beam breaks. 
    // Sets hopper variables to the opposite value to represent on SmartDashboard if there is fuel rather than if the beam break sensor detects the beam.
    isHopperFull = !hopperFullBeamBreak.get();
    rightHopperHasFuel = rightHopperBeamBreak.get();
    leftHopperHasFuel = leftHopperBeamBreak.get();

    // Gets the velocity/speed of the motor.
    twindexerSpeed = twindexerMotor.getVelocity();

    // Gets if the motor is spinning.
    isTwindexerSpinning = getTwindexerSpinning();

    //Displays whether or not the twindexer is spinning as a boolean.
    SmartDashboard.putBoolean("Twindexer is spinning: ", isTwindexerSpinning);

    //Displays whether or not the hopper beam break is broken as a boolean.
    SmartDashboard.putBoolean("Is the hopper full: ", isHopperFull);
  }
}
