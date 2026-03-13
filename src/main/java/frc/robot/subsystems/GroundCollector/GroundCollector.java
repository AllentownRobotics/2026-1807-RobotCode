// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.GroundCollector;

import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.hardware.CANcoder;
// import edu.wpi.first.units.Unit
import com.ctre.phoenix6.signals.GravityTypeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.units.VoltageUnit;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Mechanism;
import frc.robot.Constants;
import frc.robot.Constants.pivotConsants;
import frc.robot.Constants.collectorConstants;
import frc.utils.Kraken;

public class GroundCollector extends SubsystemBase {
  private Kraken pivotMotor;//establishes the pivot motor
  private CANcoder pivotEncoder;//establishes the pivot encoder
  private double desiredSetpoint;//set pivot encoder position
  private DigitalInput intakeLimitSwitch, homeLimitSwitch;//establishes the 2 pivot limit switches
  private Kraken collectorMotor;//establishes the collector motor
  private PIDController pivotFeedbackLoop = new PIDController(pivotConsants.kP, pivotConsants.kI, pivotConsants.kD);//establishes pid constants for pivot
  private PIDController collectorFeedbackLoop = new PIDController(collectorConstants.collectorP, collectorConstants.collectorI, collectorConstants.collectorD);//establishes pid constants for collector
  private double valueOfPIDLoop;//establishes constant for pid loop
  private double collectorValueOfPIDLoop;
  public double voltage; 
  private ProfiledPIDController rotationController;
  private double maxVelocity;
  private SysIdRoutine pivotSysID;

  private PIDController tempController = new PIDController(Constants.pivotConsants.kP, pivotConsants.kI, pivotConsants.kD);

  public GroundCollector(){
    pivotMotor = new Kraken(pivotConsants.pivotMotorID);//make a new motor
    pivotEncoder = new CANcoder(pivotConsants.pivotEncoderID);//make a new encoder
    voltage = 0;

    // intakeLimitSwitch = new DigitalInput(pivotConsants.intakeLimitSwitchPort);//make a new intake limit switch
    // homeLimitSwitch = new DigitalInput(pivotConsants.homeLimitSwitchPort);//make a new home limit switch

    pivotMotor.addEncoder(pivotEncoder);//add the encoder to the motor

    collectorMotor = new Kraken(collectorConstants.collectorMotorID);//makes a new collector motor

    pivotMotor.setBrakeMode();//stop motor 

    pivotMotor.setPIDValues( Constants.pivotConsants.kP, pivotConsants.kI, pivotConsants.kD, pivotConsants.kS, pivotConsants.kV, pivotConsants.kA, pivotConsants.kG);

    pivotMotor.setMotorCurrentLimits(40);

    //desiredSetpoint = pivotConsants.pivotInPosition;//sets desiredSetpoint to the needed position

    pivotEncoder.setPosition(0);//change if needed - sets position of the encoder

    //pivotMotor.setDesiredEncoderPosition(desiredSetpoint);//sets the encoder to desiredSetpoint


    pivotMotor.setRotorToSensorRatio(20);
    pivotMotor.setSensorToMechanismRatio(1);//gear ratio 1:20

    maxVelocity = 0.4;
    pivotMotor.krakenConfiguration.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
    pivotMotor.kraken.getConfigurator().apply(pivotMotor.krakenConfiguration);
    //pivotMotor.setInverted();
    pivotSysID = new SysIdRoutine(new Config(Volts.of(0.3).per(Second),Volts.of(1), null, (state) -> SignalLogger.writeString("Collector Sys Id", state.toString())), new Mechanism(pivotMotor::setVolts, null, this));

    rotationController = new ProfiledPIDController(0.4,0,0, new Constraints(maxVelocity, maxVelocity * 5));

    pivotMotor.setSoftLimits(-0.25, // prevent us from overdriving the motor
    0.01);


    SignalLogger.start();
  }

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction){
    return pivotSysID.quasistatic(direction);
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction){
    return pivotSysID.dynamic(direction);
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

    valueOfPIDLoop = pivotFeedbackLoop.calculate(
      pivotMotor.getPosition()
    );//uses PID loop to calculate motor speed

    pivotMotor.setMotorSpeed(valueOfPIDLoop);//sets speed to value given by PID loop

    /*
     if lower limit or upper limit switch is reached, set pivot motor speed to 0
     */
    // if (isIntakeLimitSwitchReached() || isHomeLimitSwitchReached()){
    //   pivotMotor.setMotorSpeed(0);
    // }
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
    
    // pivotMotor.setDesiredEncoderPosition(desiredSetpoint);
    // pivotFeedbackLoop.setSetpoint(setpoint);//gives the PID loop the needed setpoint
  } //update to setPivotRotations, say what setpoint is in comments

  public void setPivotPositionFeedforwards(double setpoint, double feedforwards){
    desiredSetpoint = setpoint;
    pivotMotor.setDesiredEncoderPosition(desiredSetpoint, voltage);
  }


 /**
  * gets angle of the pivot in degrees
  * @return position of pivot motor
  */
  public double getPivotPosition(){
    return pivotMotor.getPosition();//converts rotations into degrees
  }


  public void drivePivotVolts(Double voltage){
    pivotMotor.setVolts(voltage);
  }
  
  /**
   * returns true/false if the intake limit switch is reached
   * @return true or false if limit switch is pressed
   */
  // public boolean isIntakeLimitSwitchReached(){
  //   return intakeLimitSwitch.get();//gets state of digital imput as boolean
  // }
  
  /**
   * gets true/false if the home limit switch is reached
   * @return true or false if limit switch is pressed
   */
  // public boolean isHomeLimitSwitchReached(){
  //   return homeLimitSwitch.get();//gets state of digital imput as boolean
  // }

  /**
   * returns if the pivot is at a certain desired position
   * @param targetPosition
   * @return if pivot position is at intake position
   */
  public boolean isAtPosition(double targetPosition){
    double currentPosition = getPivotPosition();
    return (targetPosition - Constants.pivotConsants.positionTolerance >= currentPosition)
    &&(targetPosition + Constants.pivotConsants.positionTolerance <= currentPosition);
  }  

  /**
   * sets collector motor speed
   */
  public void setCollectorMotorSpeed(){

    collectorValueOfPIDLoop = collectorFeedbackLoop.calculate(
      collectorMotor.getPosition()
    );//uses PID loop to calculate motor speed

    collectorMotor.setMotorSpeed(collectorValueOfPIDLoop);
  }

  /**
   * stops collector motor
   */
  public void stopCollectorMotor(){
    collectorMotor.stopMotor();
  }

  /**
    * only start the collector motor when the pivot reaches its intake position,
    * when pivot is at the intake position, the collector starts spinning
    */
    public void startCollectorMotor(){
      /*if (!isIntakeLimitSwitchReached()){//checking if the intake limit switch is not pressed
        collectorMotor.setMotorSpeed(collectorValueOfPIDLoop);//sets the collector speed if its pressed
      } else {
        collectorMotor.setMotorSpeed(0);//otherwise keep as 0 - stops motor
      }*/

      if (Math.abs(Constants.pivotConsants.pivotOutPosition - pivotMotor.getPosition()) <= pivotConsants.positionTolerance){
        collectorMotor.setMotorSpeed(collectorValueOfPIDLoop);//sets collector speed if at the intake psoition
      } else {
        collectorMotor.setMotorSpeed(0);//otherwise keep as 0 - stops motor
      }
    }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("valueOfPIDLoop",valueOfPIDLoop);
    SmartDashboard.putNumber(" pivot arm position", pivotMotor.getPosition());
    SmartDashboard.putNumber("Collector valueOfPIDLoop", collectorValueOfPIDLoop);
    SmartDashboard.putNumber("Encoder position", pivotEncoder.getPosition().getValueAsDouble());
    SmartDashboard.putNumber("kraken volts", pivotMotor.kraken.getMotorVoltage().getValueAsDouble());
    SmartDashboard.putNumber("desired voltage", voltage);
    SmartDashboard.putNumber("desired state", desiredSetpoint);


    // State goalState = new State(desiredSetpoint, 0);
    // State currenState = new State(pivotEncoder.getPosition().getValueAsDouble(), pivotEncoder.getVelocity().getValueAsDouble());

    double correction = rotationController.calculate(pivotEncoder.getPosition().getValueAsDouble(), desiredSetpoint);
    double targetVelocity = correction + rotationController.getSetpoint().velocity;

    SmartDashboard.putNumber("target vel", targetVelocity);
    double closedLoopVoltage = tempController.calculate(pivotEncoder.getVelocity().getValueAsDouble(), targetVelocity);

    // closedLoopVoltage + pivotConsants.kS + targetVelocity * pivotConsants.kV
    double finalVoltage = closedLoopVoltage + (pivotConsants.kS * Math.signum(targetVelocity) + targetVelocity * pivotConsants.kV);

    SmartDashboard.putNumber("computed voltage", finalVoltage);
    pivotMotor.setVolts(finalVoltage);
  }


}
