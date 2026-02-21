// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.GroundCollector;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix6.hardware.CANcoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.collectorConstants;
import frc.robot.Constants.pivotConsants;
import frc.utils.Kraken;

public class GroundCollector extends SubsystemBase {
  private Kraken pivotMotor;//establishes the pivot motor
  private CANcoder pivotEncoder;//establishes the pivot encoder
  private double desiredSetpoint;//set pivot encoder position
  private DigitalInput intakeLimitSwitch, homeLimitSwitch;//establishes the 2 pivot limit switches
  private Kraken collectorMotor;//establishes the collector motor
  private PIDController feedbackLoop = new PIDController(pivotConsants.kP, pivotConsants.kI, pivotConsants.kD);//establishes pid constants
  private double gain;//establishes constant for pid loop

  public GroundCollector(){

    pivotMotor = new Kraken(pivotConsants.pivotMotorID);//make a new motor
    pivotEncoder = new CANcoder(pivotConsants.pivotEncoderID);//make a new encoder

    intakeLimitSwitch = new DigitalInput(pivotConsants.intakeLimitSwitchPort);//make a new lower limit switch
    homeLimitSwitch = new DigitalInput(pivotConsants.homeLimitSwitchPort);//make a new upper limit switch

    pivotMotor.addEncoder(pivotEncoder);//add the encoder

    collectorMotor = new Kraken(collectorConstants.collectorMotorID);//makes a new collector motor


    //set PID values
    //pivotMotor.setPIDValues(pivotConsants.kP, pivotConsants.kI, pivotConsants.kD, pivotConsants.kS, pivotConsants.kV, pivotConsants.kA, pivotConsants.kG);

    pivotMotor.setCoastMode();//stop motor 

    //pivotMotor.setMotorCurrentLimits(0);//set current limit 
    //pivotMotor.setSoftLimits(pivotConsants.softLimitMinPosition, pivotConsants.softLimitMaxPosition);//set limits for motor

    //desiredSetpoint = pivotConsants.homePosition;//sets desiredSetpoint to the needed position
    pivotEncoder.setPosition(0);//change if needed - sets position of the encoder
    //pivotMotor.setDesiredEncoderPosition(desiredSetpoint);//sets the encoder to desiredSetpoint
  }
  //pid loop for motor speed
  public void pivotMotorSpin(){
    //pivotMotor.setMotorSpeed(
      //feedbackLoop.calculate(
        //pivotMotor.getPosition()
      //)
    //);
    gain = feedbackLoop.calculate(
      pivotMotor.getPosition()
    );
    pivotMotor.setMotorSpeed(gain);
    //if lower limit and upper limit switch is reached, set pivot motor speed to 0
    if (isintakeLimitSwitchReached() || ishomeLimitSwitchReached()){
      pivotMotor.setMotorSpeed(0);
    }
  }
  //stops the motor
  public void stopPivotMotor(){
    pivotMotor.stopMotor();
  }
  //sets the encoder position
  public void setPivotPosition(double setpoint){
    desiredSetpoint = setpoint;
    feedbackLoop.setSetpoint(setpoint);
  }
  //adjusts position of pivot incrimentally
  public void adjustPositionIncrimentally(double increment){
    desiredSetpoint += increment;
    pivotMotor.setDesiredEncoderPosition(desiredSetpoint);
  }
 //gets position of the pivot
  public double getPivotPositionInInches(){
    return pivotMotor.getPosition();
  }
  //gets true/false if the lower limit switch is reached
  public boolean isintakeLimitSwitchReached(){
    return intakeLimitSwitch.get();
  }
  //gets true/false if the lower limit switch is reached
  public boolean ishomeLimitSwitchReached(){
    return homeLimitSwitch.get();
  }

  //if the pivot is at a certain desired position, return true, otherwise return false
  public BooleanSupplier isAtPosition(double targetPosition){
    double currentPosition = getPivotPositionInInches();
    if((targetPosition - Constants.pivotConsants.positionTolerance >= currentPosition)&&(targetPosition + Constants.pivotConsants.positionTolerance <= currentPosition)){
    return () -> true;
    }
   return () -> false;
  }  

  //sets collector motor speed
  public void collectorMotorSpin(){
    collectorMotor.setMotorSpeed(0.1);//change motor speed
  }
  //stops collector motor
  public void stopCollectorMotor(double speed){
    collectorMotor.setBrakeMode();

  }
//only start the collector motor when the pivot reaches its intake position
    public void startCollectorMotor(){
      if (isintakeLimitSwitchReached()){
        collectorMotor.setMotorSpeed(Constants.collectorConstants.collectorMotorSpeed);
      } else{
        collectorMotor.setMotorSpeed(0);
      }
    }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("gain",gain);
    SmartDashboard.putNumber("motor posiition", pivotMotor.getPosition());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }


}
