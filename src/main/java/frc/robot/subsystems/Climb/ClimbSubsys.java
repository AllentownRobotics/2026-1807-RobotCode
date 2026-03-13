// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Climb;

import static edu.wpi.first.units.Units.Volts;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimbConstants;
import frc.utils.Kraken;

public class ClimbSubsys extends SubsystemBase {
  private Kraken rightClimbMotor, leftClimbMotor;
  private CANcoder climbEncoder;
  private double desiredSetpoint;

  /** Creates a new Climb. */
  public ClimbSubsys() {
    rightClimbMotor = new Kraken(ClimbConstants.rightClimbMotorID);
    leftClimbMotor = new Kraken(ClimbConstants.leftClimbMotorID);
    climbEncoder = new CANcoder(ClimbConstants.leftClimbCANCoderID);
    //add another encoder

    //Resets Motors
    rightClimbMotor.restoreFactoryDefaults();
    leftClimbMotor.restoreFactoryDefaults();

    leftClimbMotor.setInverted();

    //Right motor follows the left motor, they are inverted (opposed)
    rightClimbMotor.follow(ClimbConstants.leftClimbMotorID, MotorAlignmentValue.Aligned);

    //Encoder for PID, only need one for left because right follows it
    leftClimbMotor.addEncoder(climbEncoder);

    //Gear Ratio and Circumference 
    leftClimbMotor.setRotorToSensorRatio(ClimbConstants.climbGearing);
    leftClimbMotor.setSensorToMechanismRatio(ClimbConstants.climbEncoderToMechanismRatio);

    //PID Values, can be changed in constants
    leftClimbMotor.setPIDValues(ClimbConstants.CLIMB_P, ClimbConstants.CLIMB_I, ClimbConstants.CLIMB_D, 
    ClimbConstants.CLIMB_SFF, ClimbConstants.CLIMB_VFF, ClimbConstants.CLIMB_AFF, ClimbConstants.CLIMB_GFF);

    //Starts motors in brake mode
    leftClimbMotor.setBrakeMode();
    rightClimbMotor.setBrakeMode();

    //Prevents motors from breaking, sets limits for their speed
    leftClimbMotor.setMotorCurrentLimits(40);
    leftClimbMotor.setSoftLimits(ClimbConstants.softLimitMinPosition, ClimbConstants.softLimitMaxPosition);

    //Desired setpoint starts off as home position, Encoder starts at 0, desired encoder position moves your robot to the desired setpoint
    desiredSetpoint = ClimbConstants.climbHomePosition;
    climbEncoder.setPosition(ClimbConstants.climbHomePosition);
    leftClimbMotor.setDesiredEncoderPosition(desiredSetpoint);
  }

  //For manual version of the command, change speed in constants
  public void setClimbSpeed(double speed){
    leftClimbMotor.setMotorSpeed(speed);
  }

  //Creates a desired setpoint for the mechanism to go to
  public void setClimbSetpoint(double setpoint){
    desiredSetpoint = setpoint;
    leftClimbMotor.setDesiredEncoderPosition(desiredSetpoint);
  }

  //Adjusts position of climb by a set increment
  public void adjustPositionIncremently(double increment){
    desiredSetpoint += increment;
    leftClimbMotor.setDesiredEncoderPosition(desiredSetpoint);
  }

  //Gets climb position in inches
  public double getClimbPositionInInches(){
    return leftClimbMotor.getPosition()*2*Math.PI*ClimbConstants.climbSprocketRadius;
  }

  //Returns a boolean as to if the mechanism has reached its desired position
  public BooleanSupplier isAtPosition(double targetPosition) {
    return () -> Math.abs(getClimbPositionInInches() - targetPosition) <= ClimbConstants.positionTolerance;
  }

  //Emergency stop
  public void stopClimb(){
    leftClimbMotor.stopMotor();
  }

  //Emergency stop
  public void stopClimbVolts(){
    leftClimbMotor.setVolts(Voltage.ofBaseUnits(0, Volts));
  }

  public void resetEncoderPos(){
    climbEncoder.setPosition(0);
  }
  
  @Override
  public void periodic() {
    
    SmartDashboard.putNumber("position in inches", getClimbPositionInInches());
    // This method will be called once per scheduler run
  }
}
