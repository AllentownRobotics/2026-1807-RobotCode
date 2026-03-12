// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Shooter;

import com.ctre.phoenix6.hardware.CANcoder;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.AimingConstants;
import frc.robot.subsystems.Drive.CommandSwerveDrivetrain;
// import frc.robot.subsystems.CommandSwerveDrivetrain;
// import frc.robot.utils.FieldConstants;
import frc.robot.utils.Kraken;
import java.util.Optional;

public class TurretSubsys extends SubsystemBase {
  /** Creates a new Turret. */
  private Kraken turretMotor;

  private CommandSwerveDrivetrain drive;
  private CANcoder turretEncoder;
  private double currentTurretState;
  private double targetTurretState;
  private double turretPositionError;
  private double turretTolerance; // put in constants
  private double targetY;
  private double targetX;
  private double robotHeading;
  private Alliance alliance;
  private double currentRobotPosX;
  private double currentRobotPosY;


  public TurretSubsys(CommandSwerveDrivetrain drive) {
    this.drive = drive;
    targetTurretState = 0.2; // degrees
    // turretMotor = new Kraken(100); // make constants for this
    // turretEncoder = new CANcoder(200); // make constants for this
    // turretMotor.addEncoder(turretEncoder);

    // turretMotor.setBrakeMode();

    // turretMotor.setRotorToSensorRatio(1);
    // turretMotor.setSensorToMechanismRatio(1); // needs to be changed
    // turretMotor.setMotorCurrentLimits(40);     MAKE SURE TO SET THIS BEFORE TESTING

    // turretMotor.setPIDValues(
    //     Constants.turretConstants.turretkP,
    //     Constants.turretConstants.turretkI,
    //     Constants.turretConstants.turretkD,
    //     Constants.turretConstants.turretkS,
    //     Constants.turretConstants.turretkV,
    //     Constants.turretConstants.turretkA,
    //     Constants.turretConstants.turretkG);
  }
  /**
   * Calculates a robot relative angle to set a turret to. <p>
   * Uses inverse tangent to calculate angle between current robot pose and Hub pose. 
   * Then uses a PID loop to get the turret to a setpoint. 
   * @return nothing cuz it a void
   */
  public void trackHUB() {

    // Alliance shift, calculates the target X and target Y of the turret for angle calculation
    Optional<Alliance> ally = DriverStation.getAlliance();
    if (ally.isPresent()) {
      if (ally.get() == Alliance.Red) {
        alliance = Alliance.Red;
        targetX = Constants.turretConstants.RED_HUB.getX();
        targetY = Constants.turretConstants.RED_HUB.getY();
      }
      if (ally.get() == Alliance.Blue) {
        alliance = Alliance.Blue;
        targetX = Constants.turretConstants.BLUE_HUB.getX();
        targetY = Constants.turretConstants.BLUE_HUB.getY();
      }
    }
    // current absolute encoder reading

    currentTurretState = turretEncoder.getAbsolutePosition().getValueAsDouble() * 360;
    robotHeading = drive.getState().Pose.getRotation().getDegrees();

    currentRobotPosX = drive.getState().Pose.getX();
    currentRobotPosY = drive.getState().Pose.getY();
    Translation2d currentPose = new Translation2d(currentRobotPosX,currentRobotPosY);

    
  
    /*calculates robot relative angle by taking the inverse tan between the hub and the robot, then by subtracting
     * robot heading allows you to get a robot relative angle*/
    targetTurretState =
        Math.toDegrees(
            Math.atan2(targetY - currentRobotPosY, targetX -currentRobotPosX)
                - drive.getState().Pose.getRotation().getRadians());

    
    // only allows angles between -pi to +pi because every angle after that can be reprsensted by a
    // smaller angle in that interval
    targetTurretState = Math.toDegrees(MathUtil.angleModulus(Math.toRadians(targetTurretState)));
    // Clamps between -25 and 25 so the degrees returned can never be above 25 or below -25.
    targetTurretState = MathUtil.clamp(targetTurretState, -20, 20);
    // tells the encoder to get to that target state. divided by 360 because it wants rotations and
    // turretState returns a degree
    turretMotor.setDesiredEncoderPosition(targetTurretState / 360);

    // various smartDashboard variables to test / tune
    SmartDashboard.putNumber("Turret current state", currentTurretState);
    SmartDashboard.putNumber("Turret target state", targetTurretState);
    SmartDashboard.putNumber("RobotPoseX", drive.getState().Pose.getX());
    SmartDashboard.putNumber("RobotPoseY", drive.getState().Pose.getY());
    SmartDashboard.putNumber("HubTargetX", targetX);
    SmartDashboard.putNumber("HubTargetY", targetY);
    SmartDashboard.putNumber("Pidgeon reading", robotHeading);
  }

  public Rotation2d getTargetTurretAngle(){
     Optional<Alliance> ally = DriverStation.getAlliance();

    if (ally.isPresent()) {
      if (ally.get() == Alliance.Red) {
        alliance = Alliance.Red;
        targetX = Constants.turretConstants.RED_HUB.getX();
        targetY = Constants.turretConstants.RED_HUB.getY();
      }
      if (ally.get() == Alliance.Blue) {
        alliance = Alliance.Blue;
        targetX = Constants.turretConstants.BLUE_HUB.getX();
        targetY = Constants.turretConstants.BLUE_HUB.getY();
      }
    }

    currentRobotPosX = drive.getState().Pose.getX();
    currentRobotPosY = drive.getState().Pose.getY();

    // currentTurretState = turretEncoder.getAbsolutePosition().getValueAsDouble() * 360;
    /*calculates robot relative angle by taking the inverse tan between the hub and the robot, then by subtracting
     * robot heading allows you to get a robot relative angle*/
    // targetTurretState =
    //     Math.toDegrees(
    //         Math.atan2(targetY - drive.getState().Pose.getY(), targetX - drive.getState().Pose.getX())); 

    // if(ally.get() == Alliance.Blue && drive.getState().Pose.getX() <= 4 || ally.get() == Alliance.Red && drive.getState().Pose.getX() >= 12 ){
    // targetTurretState =
    //     Math.toDegrees(
    //         Math.atan2(targetY -  drive.getState().Pose.getY(), targetX -  drive.getState().Pose.getX()));
    // }
    // else{
    //   targetTurretState = 0;
    // }
  if(alliance == Alliance.Blue && currentRobotPosX <= AimingConstants.blueAllianceTrench){
    //  targetTurretState =
    //     Math.toDegrees(
    //         Math.atan2(targetY - currentRobotPosY, targetX -  currentRobotPosX));

      targetTurretState = targetTurretHeading(targetX,targetY, currentRobotPosX, currentRobotPosY);
    }else{
      if(alliance == Alliance.Blue && currentRobotPosX >= AimingConstants.blueAllianceTrench && currentRobotPosY <= AimingConstants.middleLine){
        targetX = AimingConstants.blueRightFeedingTargetX;
        targetY = AimingConstants.blueRightFeedingTargetY; // orbit
        // targetTurretState =
        // Math.toDegrees(
        //     Math.atan2(targetY -  drive.getState().Pose.getY(), targetX -  drive.getState().Pose.getX()));
         targetTurretState = targetTurretHeading(targetX,targetY, currentRobotPosX, currentRobotPosY);
        // targetTurretState = targetTurretState(targetY, drive.getState().Pose.getY(), targetX, drive.getState().Pose.getX());
      } 
      if(alliance == Alliance.Blue && currentRobotPosX >= AimingConstants.blueAllianceTrench && currentRobotPosY >= AimingConstants.middleLine){
        targetX = AimingConstants.blueLeftFeedingTargetX;
        targetY = AimingConstants.blueLeftFeedingTargetY;
        //  targetTurretState = 
        // Math.toDegrees(
        //     Math.atan2(targetY -  drive.getState().Pose.getY(), targetX -  drive.getState().Pose.getX()));
        targetTurretState = targetTurretHeading(targetX,targetY, currentRobotPosX, currentRobotPosY);
        // targetTurretState = targetTurretState(targetY, drive.getState().Pose.getY(), targetX, drive.getState().Pose.getX());
      }
    }
    if(alliance == Alliance.Red && currentRobotPosX >= AimingConstants.redAllianceTrench){
    //  targetTurretState =  // targetTurretState(targetY, drive.getState().Pose.getY(), targetX, drive.getState().Pose.getX());
        // Math.toDegrees(
        //     Math.atan2(targetY -  drive.getState().Pose.getY(), targetX -  drive.getState().Pose.getX()));

      targetTurretState = targetTurretHeading(targetX,targetY, currentRobotPosX, currentRobotPosY);
    }else{
      if(alliance == Alliance.Red && currentRobotPosX <= AimingConstants.redAllianceTrench && currentRobotPosY >= AimingConstants.middleLine){
        targetX = AimingConstants.redRightFeedingTargetX;
        targetY = AimingConstants.redRightFeedingTargetX; 
        // targetTurretState = // targetTurretState(targetY, drive.getState().Pose.getY(), targetX, drive.getState().Pose.getX());
        // Math.toDegrees(
        //     Math.atan2(targetY -  drive.getState().Pose.getY(), targetX -  drive.getState().Pose.getX()));

        targetTurretState = targetTurretHeading(targetX,targetY, currentRobotPosX, currentRobotPosY);
      }
      if(alliance == Alliance.Red && currentRobotPosX <= AimingConstants.redAllianceTrench && currentRobotPosY <= AimingConstants.middleLine ){
        targetX = AimingConstants.redLeftFeedingTargetX;
        targetY = AimingConstants.redLeftFeedingTargetY;
        // targetTurretState = // targetTurretState(targetY, drive.getState().Pose.getY(), targetX, drive.getState().Pose.getX());
        // Math.toDegrees(
        //     Math.atan2(targetY -  drive.getState().Pose.getY(), targetX -  drive.getState().Pose.getX()));
        targetTurretState = targetTurretHeading(targetX,targetY, currentRobotPosX, currentRobotPosY);
      }
      
    }

    return Rotation2d.fromDegrees(targetTurretState);
    
  }



// TODO Make a method for targetTurretState so it's written better
  public double targetTurretHeading(Double targetX, Double TargetY, Double currentPoseX, Double currentPoseY){
     return Math.toDegrees(
            Math.atan2(targetY - currentPoseY, targetX -  currentPoseX));
  }

  /**
   * Checks if turret is in a tolerable range of where it needs to be.
   *
   * @return boolean - true or false depending on if its there or not.
   */
  public boolean isTurretWithinTolerance() {
    return Math.abs(targetTurretState - currentTurretState) <= turretTolerance;
  }

  @Override
  public void periodic() {
    // System.out.println("Yo sim so cool");
  }
}

