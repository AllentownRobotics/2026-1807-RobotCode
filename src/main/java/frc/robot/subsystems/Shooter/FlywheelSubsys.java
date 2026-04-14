// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Shooter;

import java.util.Optional;

import javax.print.attribute.standard.Media;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.Interpolator;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.Drive.CommandSwerveDrivetrain;
import frc.utils.Kraken;

public class FlywheelSubsys extends SubsystemBase {
  /** Creates a new Flywheel. */
  CommandSwerveDrivetrain drive;
  private Kraken leftFlywheelKraken;
  private double flywheelTolerance;
  private Kraken rightFlywheelKraken;
  private double targetVelocity;
  private double hubX;
  private double hubY;
  private double distanceToHub;
  private double medianDistance;
  private double smoothedMedianDistance;
  private double smoothedDistance;
  private LinearFilter filter;
  private LinearFilter filter2;
  private MedianFilter medianFilter;
  private double autoTargetFlywheelSpeed;

  

  private TalonFX leftFlywheel;
  private TalonFX rightFlywheel;
  InterpolatingTreeMap<Double, Double> flywheelMap;

  public FlywheelSubsys(CommandSwerveDrivetrain drive) {
    this.drive = drive;
    flywheelMap =
        new InterpolatingTreeMap(
            InverseInterpolator.forDouble(),
            Interpolator.forDouble());

    flywheelTolerance = 0.1; // rps
    // leftFlywheelKraken = new Kraken(46);
    // rightFlywheelKraken = new Kraken(41);
    // rightFlywheelKraken.follow(46, MotorAlignmentValue.Opposed);
    // leftFlywheelKraken.setCoastMode();
    // leftFlywheelKraken.setRotorToSensorRatio(1);
    // leftFlywheelKraken.setSensorToMechanismRatio(1); // needs to be changed

    leftFlywheel = new TalonFX(46);
    rightFlywheel = new TalonFX(41);
    rightFlywheel.setControl(new Follower(46, MotorAlignmentValue.Opposed));
    var config = new TalonFXConfiguration();

    config.Slot0.kP = 999999;
    config.Slot0.kD = 0.1;
    config.Slot0.kV = 8;
    // config.Slot0.kD = 0.1;
    // config.Slot0.
    // config.TorqueCurrent.PeakForwardTorqueCurrent = 50;
    // config.TorqueCurrent.PeakReverseTorqueCurrent = 0;
    config.MotorOutput.PeakForwardDutyCycle = 1;
    config.MotorOutput.PeakReverseDutyCycle = 0;
    config.MotorOutput.withNeutralMode(NeutralModeValue.Coast);

    // leftFlywheel.setControl(new VelocityDutyCycle(10));
    leftFlywheel.getConfigurator().apply(config);
    rightFlywheel.getConfigurator().apply(config);
    targetVelocity = 28;


    // rightFlywheel.setControl(new Follower(46, MotorAlignmentValue.Opposed));
    filter = LinearFilter.movingAverage(15);
    medianFilter = new MedianFilter(15);
    filter2 = LinearFilter.movingAverage(30);


    flywheelMap.put(5.4 , 30.0);  //30 rps
    flywheelMap.put(5.0 , 28.0);  //28 rps
    flywheelMap.put(4.5, 28.0); //28 rps
    flywheelMap.put(4.0, 28.0); //28 rps
    flywheelMap.put(3.5, 28.0); //28 rps
    flywheelMap.put(3.0, 26.0); //26 rps
    flywheelMap.put(2.5 , 24.0); //24 rps
    flywheelMap.put(2.0, 23.0); //23 rps
    flywheelMap.put(0.0, 23.0);

    // var config = new TalonFXConfiguration();

    

    // leftFlywheelKraken.setPIDValues(0.25, 0, 0, 1.5, 0.12, 0.02, 0);

    SmartDashboard.putNumber("Target Velocity", 0);
    // flywheelMap.put(1 , );
    // flywheelMap.put("", );


  }

  public void setFlywheelSpeedFromDistance(){
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
    autoTargetFlywheelSpeed =
        flywheelMap.get(
            smoothedMedianDistance); // using distanceToHub, gets the value using that "key" from the hub
    // table
    autoTargetFlywheelSpeed =
        MathUtil.clamp(
            autoTargetFlywheelSpeed,
            0,
            65); // clamps between 0 - 90 so if it ever breaks it will never go
    // below 0 degrees or above 90 degrees.


    leftFlywheel.setControl(new VelocityDutyCycle(autoTargetFlywheelSpeed).withUpdateFreqHz(300));
    // setHoodAngle(autoTargetHoodState);

  }

  /** sets flywheel to a user wanted velocity from smart dash */
  public void setFlywheelVelocity() {
    // targetVelocity = 26; // realistically reaches 25 with current KV
    //     // SmartDashboard.getNumber("Target Velocity", 0); // gets velocity from smart dash
    // targetVelocity =
    //     MathUtil.clamp(
    //         targetVelocity, 0, 1000); // clamps so velocity can't ever be below 0 or above 5 rps.
    // leftFlywheelKraken.setVelocity(targetVelocity); // sets the velocity for the motor to get to
    leftFlywheel.setControl(new VelocityDutyCycle(22).withUpdateFreqHz(300));
    // leftFlywheel.setControl(new velocity)
  }

  /**
   * Checks if flywheel is in a tolerable range of where it needs to be.
   *
   * @return boolean - true or false depending on if its there or not.
   */
  public boolean isFlywheelAtVelocity() {
    if ((Math.abs(targetVelocity - (leftFlywheelKraken.currentVelocityInRPS()))
        <= flywheelTolerance)) {
      return true;
    } else {
      return false;
    }
  }

  public boolean flywheelAtSpeed(){
    return Math.abs(autoTargetFlywheelSpeed - leftFlywheel.getVelocity().getValueAsDouble()) <= 5; 
  }

  public void setIdleSpeed() {
    // targetVelocity =
    //     SmartDashboard.getNumber("Target Velocity", 0); // gets velocity from smart dash
    // leftFlywheelKraken.setVelocity(targetVelocity); // sets the velocity for the motor to get to
    leftFlywheel.setControl(new VelocityDutyCycle(0));
  }

  public void stopSpeed(){
    leftFlywheel.stopMotor();
  }


  public void flywheelSpeedIncrement(double Increment){

  }

  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Target Flywheel Velocity", targetVelocity);
    SmartDashboard.putNumber(
        "Current Flywheel velocity (rps)", leftFlywheel.getVelocity().getValueAsDouble());
    SmartDashboard.putNumber("motor applied current", rightFlywheel.getVelocity().getValueAsDouble());
  }
}
