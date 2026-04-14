// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Shooter;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.spark.config.EncoderConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.Interpolator;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.hoodConstants;
import frc.robot.subsystems.Drive.CommandSwerveDrivetrain;
import frc.utils.Kraken;
import java.util.Optional;

public class HoodSubsys extends SubsystemBase {
  /** Creates a new Hood. */
  private Kraken hoodMotor;
  private TalonFX motorHood;
  private CommandSwerveDrivetrain drive;
  private double zeroHoodState;
  private CANcoder hoodEncoder;
  private double currentHoodState;
  private double targetHoodState;
  private double distanceToHub;
  private double autoTargetHoodState;
  private double hubX;
  private double hubY;
  private double hoodTolerance;
  private ProfiledPIDController hoodController;
  double hoodRate;
  double Angle;
  private double desiredAngle;
  private LinearFilter filter;
  private LinearFilter filter2;
  public double smoothedDistance; // averaged distance after linear filter
  public MedianFilter medianFilter;
  private double medianDistance;
  private double smoothedMedianDistance;

  

  InterpolatingTreeMap<Double, Double> hoodMap;
  // private double hoodPositionError;
  // private double hoodPositionTolerance; // put in constants
  // private double targetY;
  // private double targetX;

  public HoodSubsys(CommandSwerveDrivetrain drive) {
    this.drive = drive;
    hoodTolerance = 0.1; // degrees
    // incrementAngl
    hoodMap =
        new InterpolatingTreeMap(
            InverseInterpolator.forDouble(),
            Interpolator.forDouble()); // makes a new interpolating table, use case is for degree
    // calculation

    motorHood = new TalonFX(45);
    hoodEncoder = new CANcoder(50);
    
    var config = new TalonFXConfiguration();
    var encoderConfig = new CANcoderConfiguration();
    config.Slot0.kP = 0.15;
    config.Slot0.kI = 0.21;
    // config.Slot0.kD = 0.002;
    // config.Slot0.kD = 0.01;
    config.Slot0.kV = 0.8;
    config.Slot0.kS = 0.2;
    config.MotorOutput.withNeutralMode(NeutralModeValue.Brake);
    config.Feedback.withRemoteCANcoder(hoodEncoder);
    // config.Feedback.withRemoteCANcoder(hoodEncoder);
    config.Feedback.RotorToSensorRatio = 3;
    config.Feedback.SensorToMechanismRatio = 10.0 / 360.0; // 10 to 1 gear ratio converted to degrees with a step down gear ratio
    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    

    // sensor to mechanism ratio is 10:1
    // rotor to sensor ratio is 3:1
    // rotor to mechanism is 30:1

    
    encoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
    
    hoodEncoder.getConfigurator().apply(encoderConfig);

    
    hoodEncoder.setPosition(0);
    // slot0Configs.kP = 40; // A position error of 2.5 rotations results in 12 V output
    // slot0Configs.kI = 0; // no output for integrated error
    // slot0Configs.kD = 0; // A velocity error of 1 rps results in 0.1 V output
    motorHood.getConfigurator().apply(config);

    filter = LinearFilter.movingAverage(15);
    medianFilter = new MedianFilter(15);
    filter2 = LinearFilter.movingAverage(45); // go back to 30
    
    // hoodEncoder.getConfigurator().apply(encoderConfig);
    

    

    // hoodMotor = new Kraken(45); // make constants for this
    // hoodEncoder = new CANcoder(50); // make constants for this
    // hoodMotor.addEncoder(hoodEncoder);

    // hoodMotor.setBrakeMode(); // sets break mode when not in use

    // hoodMotor.setRotorToSensorRatio(30);
    // hoodMotor.setSensorToMechanismRatio(20); // needs to be changed, gear ratio of the mechanism
    // hoodMotor.resetEncoder();
    // hoodEncoder.setPosition(0.0);
    
    // hoodMotor.setMotorCurrentLimits(35);    // MAKE SURE TO SET THIS BEFORE TESTING

    // hoodMotor.setPIDValues(hoodConstants.hoodkP, hoodConstants.hoodkI, hoodConstants.hoodkD, hoodConstants.hoodkS, hoodConstants.hoodkV, hoodConstants.hoodkA, hoodConstants.hoodkG);
    


    // hoodController = new ProfiledPIDController(hoodConstants.hoodkP, hoodConstants.hoodkI, hoodConstants.hoodkD, new Constraints(0.05, 0.075));
    // hoodController.enableContinuousInput(0, 1.15);
    SmartDashboard.putNumber("Hood target", 0);

    // arbitrary numbers for testing change once testing
    // "Key" in our case represents distance, value is degrees of rotation



    // hoodMap.put(2.0, 0.0);
    // hoodMap.put(3.01, 5.0);
    // hoodMap.put(3.2549025209279647, 9.0);
    // hoodMap.put(3.549, 11.0);
    // hoodMap.put(3.98631579274598, 13.0);
    // hoodMap.put(4.7629567823908365, 24.0);


    hoodMap.put(5.4 , 17.0);  //30 rps
    hoodMap.put(5.0 , 15.8);  //28 rps
    hoodMap.put(4.5, 14.0); //28 rps
    hoodMap.put(4.0, 11.6); //28 rps
    hoodMap.put(3.5, 10.0); //28 rps
    hoodMap.put(3.0, 6.7); //26 rps
    hoodMap.put(2.5 , 5.2); //24 rps
    hoodMap.put(2.0, 3.2); //23 rps
    hoodMap.put(0.0, 3.2);
    // hoodMap.put()




    // hoodMap.put(4.3 , 12); 
   // hoodMap.put(5.196783129652328 , 16.0);

  }


  public void setHoodAutomatically(){

    final PositionVoltage m_request = new PositionVoltage(20).withSlot(0);
// set position to 10 rotations
    motorHood.setControl(m_request);
  }

  /**
   * automatically calculates which alliance you are and using that side hub calculates distance to
   * it allowing the use of a interpolating table to automatically calculate what degree of rotation
   * the hub must be at to make a shot.
   */
  public void setHoodAutomaticallyFromDistance() {

    // calculates the hub constant
    Optional<Alliance> ally = DriverStation.getAlliance();
    if (ally.isPresent()) {
      if (ally.get() == Alliance.Red) {
        hubX = Constants.turretConstants.RED_HUB.getX();
        hubY = Constants.turretConstants.RED_HUB.getY();
      }
      if (ally.get() == Alliance.Blue) {
        hubX = Constants.turretConstants.BLUE_HUB.getX();
        hubY = Constants.turretConstants.BLUE_HUB.getY();
      }
    }
    double currentPosX = drive.getState().Pose.getX();
    double currentPosY = drive.getState().Pose.getY();
    Translation2d currentPoint = new Translation2d(currentPosX, currentPosY);
    Translation2d targetPoint = new Translation2d(hubX, hubY);

    // distance formula using the hub as x2 and current drive pose as x1
    
    distanceToHub = currentPoint.getDistance(targetPoint);

    medianDistance = medianFilter.calculate(distanceToHub);
    smoothedDistance = filter.calculate(distanceToHub);

    smoothedMedianDistance = filter2.calculate(medianDistance);


    
    

        // Math.sqrt(
        //     Math.pow(hubX - drive.getState().Pose.getX(), 2)
        //         + Math.pow(hubY -  drive.getState().Pose.getY(), 2));
    autoTargetHoodState =
        hoodMap.get(
            smoothedMedianDistance); // using distanceToHub, gets the value using that "key" from the hub
    // table
    autoTargetHoodState =
        MathUtil.clamp(
            autoTargetHoodState,
            0,
            45); // clamps between 0 - 90 so if it ever breaks it will never go
    // below 0 degrees or above 90 degrees.


    
    setHoodAngle(autoTargetHoodState);

    // hoodMotor.setDesiredEncoderPosition(
    //     autoTargetHoodState / 360); // applies that position in rotations

    // various smartDashboard numbers to test user wanted values
    // SmartDashboard.putNumber("Distance in meters to da HUB", distanceToHub);
    // SmartDashboard.putNumber("Autonomous Target Hood State", autoTargetHoodState);
    SmartDashboard.putNumber("Median distance output", medianDistance);
    SmartDashboard.putNumber("smoothed distanced", smoothedDistance);
    SmartDashboard.putNumber("Smooted Median distance", smoothedMedianDistance);
  }

  public void setHoodAngle(double angle){
    Angle = angle;
    // desiredAngle = angle;
    final PositionVoltage m_request = new PositionVoltage(Angle).withSlot(0);
// set position to 10 rotations
    motorHood.setControl(m_request);
  }

  

  public void incrementHoodAngle(double increment){
    Angle += increment;
    setHoodAngle(Angle);
  }

  public double manualHoodChecks(){
    Optional<Alliance> ally = DriverStation.getAlliance();
    if (ally.isPresent()) {
      if (ally.get() == Alliance.Red) {
        hubX = Constants.turretConstants.RED_HUB.getX();
        hubY = Constants.turretConstants.RED_HUB.getY();
      }
      if (ally.get() == Alliance.Blue) {
        hubX = Constants.turretConstants.BLUE_HUB.getX();
        hubY = Constants.turretConstants.BLUE_HUB.getY();
      }
    }

    double currentPosX = drive.getState().Pose.getX();
    double currentPosY = drive.getState().Pose.getY();
    Translation2d currentPoint = new Translation2d(currentPosX, currentPosY);
    Translation2d targetPoint = new Translation2d(hubX, hubY);

    // distance formula using the hub as x2 and current drive pose as x1
    
    distanceToHub = currentPoint.getDistance(targetPoint);
    return distanceToHub;
  }

  public void manualSetHoodAngle() {
    targetHoodState = 0.2;
    currentHoodState = hoodEncoder.getAbsolutePosition().getValueAsDouble() * 360;
    targetHoodState = MathUtil.clamp(targetHoodState, 0, 45);
    // hoodRate = hoodController.calculate(hoodEncoder.getAbsolutePosition().getValueAsDouble(), targetHoodState);
    // hoodMotor.setMotorSpeed(-hoodRate);
    hoodMotor.setDesiredEncoderPosition(targetHoodState);


    // hoodMotor.setDesiredEncoderPosition(targetHoodState / 360);
  }
  /**
   * Checks if hood is in a tolerable range of where it needs to be.
   *
   * @return boolean - true or false depending on if its there or not.
   */
  public boolean isHoodWithinTolerance() {
    if (Math.abs(targetHoodState - (hoodEncoder.getAbsolutePosition().getValueAsDouble() * 360))
        <= hoodTolerance) {
      return true;
    } else {
      return false;
    }
  }

  public boolean HoodWithinTolerance(){
    return Math.abs(autoTargetHoodState - motorHood.getPosition().getValueAsDouble()) <= 0.4;
  }

  /** Sends hood to the home state. */
  public void setHoodToHome() {
    zeroHoodState = 0.5;
    currentHoodState = hoodEncoder.getAbsolutePosition().getValueAsDouble() * 360;
    // targetHoodState = MathUtil.clamp(zeroHoodState, 0, 45);
    // hoodMotor.setDesiredEncoderPosition(zeroHoodState);
    // hoodRate = -hoodController.calculate(hoodEncoder.getAbsolutePosition().getValueAsDouble(), zeroHoodState);
    // hoodMotor.setMotorSpeed(hoodRate + hoodController.getSetpoint().velocity);
    // hoodEncoder.setPosition(0.2);
    hoodMotor.setDesiredEncoderPosition(0.21);
    SmartDashboard.putNumber("Is hood trying to go to 0?", targetHoodState);
    SmartDashboard.putNumber("where da hood at", currentHoodState);
  }

  public void setHoodSpeed(){
    hoodMotor.setMotorSpeed(-0.4);
  }

  public void stopHoodMotors(){
    hoodMotor.setMotorSpeed(0);
  }

  public void setHoodSpeedBack(){
    hoodMotor.setMotorSpeed(0.4);
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    SmartDashboard.putNumber("distance to HUB", manualHoodChecks());
    SmartDashboard.putNumber("Hood current angle yo", hoodEncoder.getPosition().getValueAsDouble());
    SmartDashboard.putNumber("hOOD voltage" , motorHood.getMotorVoltage().getValueAsDouble());
    SmartDashboard.putNumber("motors position", motorHood.getPosition().getValueAsDouble());
    SmartDashboard.putNumber("Encoder reading for hood", hoodEncoder.getPosition().getValueAsDouble());
    SmartDashboard.putNumber("Increment angle", Angle);
    SmartDashboard.putNumber("INterpolating map angle", autoTargetHoodState);
    


//     final PositionVoltage m_request = new PositionVoltage(desiredAngle).withSlot(0);
// // set position to 10 rotations
//     motorHood.setControl(m_request);

    // SmartDashboard.putBoolean("Hood at target?", isHoodWithinTolerance());
  }
}
