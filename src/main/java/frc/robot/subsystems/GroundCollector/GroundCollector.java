// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.GroundCollector;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix6.hardware.CANcoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.collectorConstants;
import frc.robot.Constants.pivotConsants;
import frc.utils.Kraken;

public class GroundCollector extends SubsystemBase {
  private Kraken pivotMotor;//establishes the pivot motor
  private CANcoder pivotEncoder;//establishes the pivot encoder
  private double desiredSetpoint;//set pivot encoder position
  private DigitalInput lowerLimitSwitch, upperLimitSwitch;//establishes the 2 pivot limit switches
  private Kraken collectorMotor;//establishes the collector motor
  private PIDController feedbackLoop = new PIDController(pivotConsants.kP, pivotConsants.kI, pivotConsants.kD);


  public GroundCollector(){

    pivotMotor = new Kraken(pivotConsants.pivotMotorID);//make a new motor
    //pivotEncoder = new CANcoder(pivotConsants.pivotEncoderID);//make a new encoder

    lowerLimitSwitch = new DigitalInput(pivotConsants.lowerLimitSwitchPort);//make a new lower limit switch
    upperLimitSwitch = new DigitalInput(pivotConsants.upperLimitSwitchPort);//make a new upper limit switch

    //pivotMotor.addEncoder(pivotEncoder);//add the encoder
    //set PID values
    //pivotMotor.setPIDValues(pivotConsants.kP, pivotConsants.kI, pivotConsants.kD, pivotConsants.kS, pivotConsants.kV, pivotConsants.kA, pivotConsants.kG);

    pivotMotor.setCoastMode();//stop motor 

    pivotMotor.setMotorCurrentLimits(0);//set current limit 
    //pivotMotor.setSoftLimits(pivotConsants.softLimitMinPosition, pivotConsants.softLimitMaxPosition);//set limits for motor

    desiredSetpoint = pivotConsants.homePosition;//sets desiredSetpoint to the needed position
    //pivotEncoder.setPosition(0);//change if needed - sets position of the encoder
    //pivotMotor.setDesiredEncoderPosition(desiredSetpoint);//sets the encoder to desiredSetpoint
  }

  public void pivotMotorSpin(){
    pivotMotor.setMotorSpeed(
      feedbackLoop.calculate(
        pivotMotor.getPosition()
      )
    );
  }

  public void stopPivotMotor(){
    pivotMotor.stopMotor();//stops the motor
  }
  //sets the encoder position
  public void setPivotPosition(double setpoint){
    desiredSetpoint = setpoint;
    feedbackLoop.setSetpoint(setpoint);
  }

  public void adjustPositionIncrimentally(double increment){
    desiredSetpoint += increment;
    pivotMotor.setDesiredEncoderPosition(desiredSetpoint);
  }
 //gets position of the pivot
  public double getPivotPositionInInches(){
    return pivotMotor.getPosition();
  }
  //gets true/false if the lower limit switch is reached
  public boolean isLowerLimitSwitchReached(){
    return lowerLimitSwitch.get();
  }
  //gets true/false if the lower limit switch is reached
  public boolean isUpperLimitSwitchReached(){
    return upperLimitSwitch.get();
  }

  //if the pivot is at a certain desired position, return true, otherwise return false
  public BooleanSupplier isAtPosition(double targetPosition){
    double currentPosition = getPivotPositionInInches();
    if((targetPosition - Constants.pivotConsants.positionTolerance >= currentPosition)&&(targetPosition + Constants.pivotConsants.positionTolerance <= currentPosition)){
    return () -> true;
    }
   return () -> false;
  }  

  //if the pivot is crossing the beambreak, stop the pivot from moving forwards


  //makes a new collector motor
  public void collector(){
    collectorMotor = new Kraken(collectorConstants.collectorMotorID);
  }
  //sets collector motor speed
  public void collectorMotorSpin(){
    collectorMotor.setMotorSpeed(0.1);//change motor speed
  }
  //stops collector motor
  public void stopCollectorMotor(){
    collectorMotor.setBrakeMode();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }


}
