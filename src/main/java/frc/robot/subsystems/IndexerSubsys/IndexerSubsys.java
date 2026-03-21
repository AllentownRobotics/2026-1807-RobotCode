// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.IndexerSubsys;

// import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.utils.Kraken;

public class IndexerSubsys extends SubsystemBase {

  /** 
   * The motor that powers the indexer.
   */
  private final Kraken indexerMotor; 

  /**
   * The beam break at the top of the hopper that returns values representing whether the hopper is full of fuel or not.
   */
  private final DigitalInput topHopperBeamBreak; 
  
  /**
   * The beam break near the bottom of the hopper that detects if there is any fuel in the hopper on this side.
   */
  private final DigitalInput bottomHopperBeamBreak;

  /**
   * Boolean that returns true if the indexer is spinning, and false if it is not spinning.
   */
  private boolean isIndexerSpinning;

  /**
   * The current speed of the indexer.
   */
  private double indexerSpeed;

  /**
   * Returns false if full hopper beam break is broken, and true if it is not broken.
   */
  private boolean hopperFullBeamBreak;

  /**
   * Returns opposite value of hopperFullBeamBreak to display on SmartDashboard.
   */
  private boolean isHopperFull;

  /**
   * Returns false if bottom hopper beam break is broken, and true if it is not broken.
   */
  private boolean bottomFuelBeamBreak;

  /**
   * Returns opposite value of bottomFuelBeamBreak to display on SmartDashboard.
   */
  private boolean doesHopperHaveFuel;

  private double indexerCurrent;

  /** Creates a new IndexerSubsystem. */
  public IndexerSubsys() {
    // Assigns beam break IDs to their corresponding beam breaks.
    topHopperBeamBreak = new DigitalInput(Constants.IndexerConstants.topBeamBreakID);
    bottomHopperBeamBreak = new DigitalInput(Constants.IndexerConstants.bottomBeamBreakID);
    
    // Assigns motor ID to indexer motor.
    indexerMotor = new Kraken(Constants.IndexerConstants.indexerMotorID); 
    indexerMotor.setCoastMode();
    indexerMotor.setMotorCurrentLimits(Constants.IndexerConstants.indexerCurrentLimit);
    
    indexerCurrent = indexerMotor.getSupplyCurrent();

    // Assigns PID values to indexer motor.
    indexerMotor.setPIDValues(0, 0, 0, 0, Constants.IndexerConstants.kv, Constants.IndexerConstants.ka, 0);
  }

  // The motor methods are not used for Seneca because the robot is temporarily built to use a non-motorized indexer 

  /**
   * Sets the desired speed (setpoint) for the indexer PID.
  */
  // 
  // Old PID setpoint method (Note: indexerPIDController no longer exists)
  //
  // public void setIndexerDesiredSpeed(double speed) {
  //   // Assigns setpoint value to PID controller.
  //   indexerPIDController.setSetpoint(Constants.IndexerConstants.desiredIndexerSpeed);
  // }

  /**
   * Sets indexer motor to a desired speed using PID with our setpoint and values.
   */
  public void setIndexerSpeed() {
    // Calculates motor speed by getting motor velocity and using PID.
    // indexerMotor.setMotorSpeed(
    //   indexerPIDController.calculate(
    //     indexerMotor.getVelocity()
    //   ) + indexerMotor.getVelocity() 
    // );
    indexerMotor.setVelocity(Constants.IndexerConstants.indexerVelocity);
  }

  /**
   * Sets indexer motor to a desired speed using PID ONLY if one of the beam breaks detects fuel. Used in auto.
   */
  public void autosSetIndexerSpeed() {
    // The commented PID loop works but is inefficient.

    // if (bottomHopperBeamBreak.get()) {
    //   indexerMotor.setMotorSpeed(
    //     indexerPIDController.calculate(
    //       indexerMotor.getVelocity()
    //     ) + indexerMotor.getVelocity()
    //   );
    // } else {
    //   indexerMotor.setMotorSpeed(0);
    // }

    // Uses PID to get motor up to speed set in constants.
    if (bottomHopperBeamBreak.get()) {
      indexerMotor.setVelocity(Constants.IndexerConstants.indexerVelocity);
    } else {
      indexerMotor.setMotorSpeed(0);
    }
    
  } 

  /**
   * Gets the indexer's velocity from indexerSpeed and determines if the indexer is spinning.
   * @return motorSpinning: boolean representing if the indexer is spinning (true) or if it is not spinning (false)
   */
  public boolean getIndexerSpinning() {

    boolean isMotorSpinning = false;
    // Checks indexerSpeed to see if it is above 0. Returns true if it is, and false if it is 0.
    if (indexerSpeed > 0) {
      return isMotorSpinning;
    }

    return isMotorSpinning;
  } // (Not urgent) TODO is there a simpler way to do this?

  /**
   * Sets indexer motor speed to 0 so that it slows to a stop.
   */
  public void stopIndexer() {
    // Commented code is for using PID to stop the indexer (currently not necessary because motor is on coast mode).
    //
    // indexerMotor.setMotorSpeed(
    //   indexerPIDController.calculate(
    //     indexerMotor.getVelocity(), 0
    //   ) + indexerMotor.getVelocity()
    // );
    indexerMotor.setMotorSpeed(0); // Sets motor speed to 0 - coast mode will let it slow to a stop.
  }

  // The get hopper methods are for other subsystems/autos. They are currently not used for the Indexer subsystem itself.

  /**
   * Gives the value of topHopperBeamBreak.
   * @return topHopperBeamBreak: the value of the beam break (true if not broken, false if broken by fuel).
   */
  public boolean getTopHopper() {
    return hopperFullBeamBreak;
  }

  /**
   * Gives the value of bottomHopperBeamBreak.
   * @return bottomHopperBeamBreak: the value of the beam break (true if not broken, false if broken by fuel).
   */
  public boolean getBottomHopper() {
    return bottomFuelBeamBreak;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    // Gets the value of the top hopper beam break. 
    hopperFullBeamBreak = topHopperBeamBreak.get(); 

    //Gets the value of the bottom hopper beam break.
    bottomFuelBeamBreak = bottomHopperBeamBreak.get();

    // Sets the variables to the opposite value to represent on SmartDashboard if there is fuel,
    // rather than if the beam break sensor detects the beam.
    isHopperFull = !hopperFullBeamBreak;
    doesHopperHaveFuel = !bottomFuelBeamBreak;

    // Gets the velocity/speed of the motor.
    indexerSpeed = indexerMotor.getVelocity();

    // Gets if the motor is spinning.
    isIndexerSpinning = getIndexerSpinning();

    //Displays whether or not the indexer is spinning as a boolean.
    SmartDashboard.putBoolean("Indexer is spinning: ", isIndexerSpinning);

    //Displays whether or not the hopper is full as a boolean.
    SmartDashboard.putBoolean("Is the hopper full: ", isHopperFull);

    //Displays whether or not there is any fuel in the hopper as a boolean.
    SmartDashboard.putBoolean("Is there fuel in hopper: ", doesHopperHaveFuel);

    // Displays the indexer's current.
    SmartDashboard.putNumber("Indexer current: ", indexerCurrent);
  }
}
