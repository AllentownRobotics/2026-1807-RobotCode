// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Shooter;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.Interpolator;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.AimingConstants;
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
  InterpolatingTreeMap<Double, Double> feedingMap;
  // private double hoodPositionError;
  // private double hoodPositionTolerance; // put in constants
  // private double targetY;
  // private double targetX;

  public HoodSubsys(CommandSwerveDrivetrain drive) {

    feedingMap =
        new InterpolatingTreeMap(
            InverseInterpolator.forDouble(),
            Interpolator.forDouble());

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
    

    
    SmartDashboard.putNumber("Hood target", 0);

    
    // "Key" in our case represents distance, value is degrees of rotation


    hoodMap.put(5.4 , 17.0);  //30 rps
    hoodMap.put(5.0 , 15.8);  //28 rps
    hoodMap.put(4.5, 13.4); //28 rps // 14 before 10:06 right before Q60
    hoodMap.put(4.0, 11.6); //28 rps
    hoodMap.put(3.5, 7.0); //28 rps
    hoodMap.put(3.0, 6.7); //26 rps
    hoodMap.put(2.5 , 5.2); //24 rps
    hoodMap.put(2.0, 3.2); //23 rps
    hoodMap.put(0.0, 3.2);
   


    //Interpolating map data values for Feeding
    // Key is distance, value is RPS
    feedingMap.put(5.0, 8.0);
    feedingMap.put(7.0, 10.0);
    feedingMap.put(9.0, 15.0);
    feedingMap.put(11.0, 20.0);
    feedingMap.put(12.0, 25.0);
    feedingMap.put(13.5, 30.0);
    feedingMap.put(15.0, 35.0);
    feedingMap.put(17.5, 40.0);
    feedingMap.put(19.0, 45.0);
  }



   /**
   * automatically calculates which alliance you are and using that side hub, calculates distance to
   * the hub allowing the use of a interpolating table to automatically calculate what degree of rotation
   * the hood should be at.
   */
  public void setFeedingAngle(){


    double currentPosX = drive.getState().Pose.getX();
    double currentPosY = drive.getState().Pose.getY();


    Optional<Alliance> ally = DriverStation.getAlliance();
    if (ally.isPresent()) {
      if (ally.get() == Alliance.Red) {
          if(ally.get() == Alliance.Red && currentPosX <= AimingConstants.redAllianceTrench && currentPosY >= AimingConstants.middleLine){
        hubX = AimingConstants.redRightFeedingTargetX;
        hubY = AimingConstants.redRightFeedingTargetY;
        }
        if(ally.get() == Alliance.Red && currentPosX <= AimingConstants.redAllianceTrench && currentPosY <= AimingConstants.middleLine ){
        hubX = AimingConstants.redLeftFeedingTargetX;
        hubY = AimingConstants.redLeftFeedingTargetY;
       }
      }
      if (ally.get() == Alliance.Blue) {
        if(ally.get() == Alliance.Blue && currentPosX >= AimingConstants.blueAllianceTrench && currentPosY >= AimingConstants.middleLine){
        hubX = AimingConstants.blueRightFeedingTargetX;
        hubY = AimingConstants.blueRightFeedingTargetY;
        }
        if(ally.get() == Alliance.Blue && currentPosX >= AimingConstants.blueAllianceTrench && currentPosY <= AimingConstants.middleLine ){
        hubX = AimingConstants.blueLeftFeedingTargetX;
        hubY = AimingConstants.blueLeftFeedingTargetY;
       }
      }
    }

    Translation2d currentPoint = new Translation2d(currentPosX, currentPosY);
    Translation2d targetPoint = new Translation2d(hubX, hubY);

    // distance formula using the hub as x2 and current drive pose as x1
    
    distanceToHub = currentPoint.getDistance(targetPoint);

    medianDistance = medianFilter.calculate(distanceToHub);
    smoothedDistance = filter.calculate(distanceToHub);
    smoothedMedianDistance = filter2.calculate(medianDistance);

    autoTargetHoodState =
        feedingMap.get(
            smoothedMedianDistance); // using distanceToHub, gets the value using that "key" from the hub
    // table
    autoTargetHoodState =
        MathUtil.clamp(
            autoTargetHoodState,
            0,
            45); // clamps between 0 - 90 so if it ever breaks it will never go
    // below 0 degrees or above 90 degrees.


    
    setHoodAngle(autoTargetHoodState);
    
    //Smart dashboard checks
    SmartDashboard.putNumber("feeding smoothed", smoothedMedianDistance);

  }


  

  /**
   * automatically calculates which alliance you are and using that side hub, calculates distance to
   * the hub allowing the use of a interpolating table to automatically calculate what degree of rotation
   * the hood must be at to make a shot.
   */
  public void setHoodAutomaticallyFromDistance() {

    // calculates the hub constant
    Optional<Alliance> ally = DriverStation.getAlliance();
    if (ally.isPresent()) {
      if (ally.get() == Alliance.Red) {
        hubX = Constants.TargettingConstants.RED_HUB.getX();
        hubY = Constants.TargettingConstants.RED_HUB.getY();
      }
      if (ally.get() == Alliance.Blue) {
        hubX = Constants.TargettingConstants.BLUE_HUB.getX();
        hubY = Constants.TargettingConstants.BLUE_HUB.getY();
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


    SmartDashboard.putNumber("Median distance output", medianDistance);
    SmartDashboard.putNumber("smoothed distanced", smoothedDistance);
    SmartDashboard.putNumber("Smooted Median distance", smoothedMedianDistance);
  }


  
  /**
   * manually set hood to a specified angle
   * @param angle in degrees
   */
  public void setHoodAngle(double angle){
    Angle = angle;
    // desiredAngle = angle;
    final PositionVoltage m_request = new PositionVoltage(Angle).withSlot(0);
// set position to 10 rotations
    motorHood.setControl(m_request);
  }

  
/**
 * variable to increment the hood angle by a specified increment
 * @param increment
 */
  public void incrementHoodAngle(double increment){
    Angle += increment;
    setHoodAngle(Angle);
  }


  /**
   * Checks if hood is in a tolerable range of where it needs to be.
   *
   * @return boolean - true or false depending on if its there or not.
   */
  public boolean HoodWithinTolerance(){
    return Math.abs(autoTargetHoodState - motorHood.getPosition().getValueAsDouble()) <= hoodTolerance;
  }

  /** Sends hood to the home state. */
  public void setHoodToHome() {
    zeroHoodState = 0.5;
    currentHoodState = hoodEncoder.getAbsolutePosition().getValueAsDouble() * 360;
    hoodMotor.setDesiredEncoderPosition(0.21);
    SmartDashboard.putNumber("Is hood trying to go to 0?", targetHoodState);
    SmartDashboard.putNumber("where da hood at", currentHoodState);
  }


  /**
   * stops the hood motor
   */
  public void stopHoodMotors(){
    hoodMotor.setMotorSpeed(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    SmartDashboard.putNumber("Hood current angle yo", hoodEncoder.getPosition().getValueAsDouble());
    SmartDashboard.putNumber("hOOD voltage" , motorHood.getMotorVoltage().getValueAsDouble());
    SmartDashboard.putNumber("motors position", motorHood.getPosition().getValueAsDouble());
    SmartDashboard.putNumber("Encoder reading for hood", hoodEncoder.getPosition().getValueAsDouble());
    SmartDashboard.putNumber("Increment angle", Angle);
    SmartDashboard.putNumber("INterpolating map angle", autoTargetHoodState);
  }
}
