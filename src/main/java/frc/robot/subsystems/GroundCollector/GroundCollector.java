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

    intakeLimitSwitch = new DigitalInput(pivotConsants.intakeLimitSwitchPort);//make a new intake limit switch
    homeLimitSwitch = new DigitalInput(pivotConsants.homeLimitSwitchPort);//make a new home limit switch

    pivotMotor.addEncoder(pivotEncoder);//add the encoder to the motor

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
  /**
   *pid loop for motor speed
   */
  public void pivotMotorSpin(){
    //pivotMotor.setMotorSpeed(
      //feedbackLoop.calculate(
        //pivotMotor.getPosition()
      //)
    //);

    gain = feedbackLoop.calculate(
      pivotMotor.getPosition()
    );//uses PID loop to calculate motor speed

    pivotMotor.setMotorSpeed(gain);//sets speed to value given by PID loop

    /*
     if lower limit and upper limit switch is reached, set pivot motor speed to 0
     */
    if (isIntakeLimitSwitchReached() || isHomeLimitSwitchReached()){
      pivotMotor.setMotorSpeed(0);
    }
  }

  /**
   *stops the pivot motor
   */
  public void stopPivotMotor(){
    pivotMotor.stopMotor();
  }

  /**
   * sets the desired encoder position
   * @param setpoint
   */
  public void setPivotPosition(double setpoint){
    desiredSetpoint = setpoint;
    feedbackLoop.setSetpoint(setpoint);//gives the PID loop the needed setpoint
  }

 /**
  * gets angle of the pivot in degrees
  * @return position of pivot motor
  */
  public double getPivotPosition(){
    return pivotMotor.getPosition() * 360;//converts rotations into degrees
  }
  
  /**
   * returns true/false if the intake limit switch is reached
   * @return true or false if limit switch is pressed
   */
  public boolean isIntakeLimitSwitchReached(){
    return intakeLimitSwitch.get();//gets state of digital imput as boolean
  }
  
  /**
   * gets true/false if the home limit switch is reached
   * @return true or false if limit switch is pressed
   */
  public boolean isHomeLimitSwitchReached(){
    return homeLimitSwitch.get();//gets state of digital imput as boolean
  }

  /**
   * if the pivot is at a certain desired position, return true, otherwise return false
   * @param targetPosition
   * @return true or false if the pivot is at a certain position
   */
  public BooleanSupplier isAtPosition(double targetPosition){
    double currentPosition = getPivotPosition();
    if((targetPosition - Constants.pivotConsants.positionTolerance >= currentPosition)&&(targetPosition + Constants.pivotConsants.positionTolerance <= currentPosition)){
    return () -> true;
    }
   return () -> false;
  }  

  /**
   * sets collector motor speed
   */
  public void collectorMotorSpin(){
    collectorMotor.setMotorSpeed(0.1);//change motor speed
  }

  /**
   * stops collector motor
   */
  public void stopCollectorMotor(double speed){
    collectorMotor.setBrakeMode();
  }

  /**
    * only start the collector motor when the pivot reaches its intake position,
    * when limit switches stop the pivot, the collector starts spinning
    */
    public void startCollectorMotor(){
      if (!isIntakeLimitSwitchReached()){//checking if the intake limit switch is not pressed
        collectorMotor.setMotorSpeed(Constants.collectorConstants.collectorMotorSpeed);//sets the collector speed if its pressed
      } else {
        collectorMotor.setMotorSpeed(0);//otherwise keep as 0 - stops motor
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
