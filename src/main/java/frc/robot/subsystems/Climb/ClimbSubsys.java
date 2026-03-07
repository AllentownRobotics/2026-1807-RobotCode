// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Climb;

import static edu.wpi.first.units.Units.Volts;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
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
    climbEncoder = new CANcoder(ClimbConstants.climbCANCoderID);

    rightClimbMotor.restoreFactoryDefaults();
    leftClimbMotor.restoreFactoryDefaults();

    leftClimbMotor.setNotInverted();

    //Right motor follows the left motor, they both go the same way
    rightClimbMotor.follow(ClimbConstants.leftClimbMotorID, MotorAlignmentValue.Aligned);

    leftClimbMotor.addEncoder(climbEncoder);

    leftClimbMotor.setRotorToSensorRatio(ClimbConstants.climbGearing);
    leftClimbMotor.setSensorToMechanismRatio(ClimbConstants.climbEncoderToMechanismRatio);

    leftClimbMotor.setPIDValues(ClimbConstants.CLIMB_P, ClimbConstants.CLIMB_I, ClimbConstants.CLIMB_D, 
    ClimbConstants.CLIMB_SFF, ClimbConstants.CLIMB_VFF, ClimbConstants.CLIMB_AFF, ClimbConstants.CLIMB_GFF);

    leftClimbMotor.setBrakeMode();
    rightClimbMotor.setBrakeMode();

    leftClimbMotor.setMotorCurrentLimits(40);
    leftClimbMotor.setSoftLimits(ClimbConstants.softLimitMinPosition, ClimbConstants.softLimitMaxPosition);

    desiredSetpoint = ClimbConstants.climbHomePosition;
    climbEncoder.setPosition(0);
    leftClimbMotor.setDesiredEncoderPosition(desiredSetpoint);
  }

  /*public void setClimbSpeed(double speed){
    rightClimbMotor.setMotorSpeed(speed);
  }*/

  public void setClimbSetpoint(double setpoint){
    desiredSetpoint = setpoint;
    leftClimbMotor.setDesiredEncoderPosition(desiredSetpoint);
  }

  public void adjustPositionIncremently(double increment){
    desiredSetpoint += increment;
    leftClimbMotor.setDesiredEncoderPosition(desiredSetpoint);
  }

  public double getClimbPositionInInches(){
    return leftClimbMotor.getPosition();
  }

  public BooleanSupplier isAtPosition(double targetPosition) {
    double currentPosition = getClimbPositionInInches();
    if ( (targetPosition - Constants.ClimbConstants.positionTolerance >= currentPosition ) && 
         (targetPosition + Constants.ClimbConstants.positionTolerance <= currentPosition)) {
      return () -> true;
    }
    return () -> false;
  }

  public void stopClimb(){
    leftClimbMotor.setVolts(Voltage.ofBaseUnits(0, Volts));
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
