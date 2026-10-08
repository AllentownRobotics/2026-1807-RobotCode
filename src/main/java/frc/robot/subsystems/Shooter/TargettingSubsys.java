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
// import frc.robot.Constants.AimingConstants;
import frc.robot.subsystems.Drive.CommandSwerveDrivetrain;
// import frc.robot.subsystems.CommandSwerveDrivetrain;
// import frc.robot.utils.FieldConstants;
import frc.utils.Kraken;
import java.util.Optional;

public class TargettingSubsys extends SubsystemBase {
  /** Creates a new Targetting subsystem. */

  private CommandSwerveDrivetrain drive;
  private double currentTurretState;
  private double targetTurretState;
  private double targettingTolerance; // put in constants
  private double targetY;
  private double targetX;
  private double robotHeading;
  private Alliance alliance;
  private double currentRobotPosX;
  private double currentRobotPosY;


  public TargettingSubsys(CommandSwerveDrivetrain drive) {
    this.drive = drive;
    targettingTolerance = 1; // degrees
  }
/**
  * Calculates a robot relative angle tracking the hub. <p>
  * Uses inverse tangent to calculate angle between current robot pose and Hub pose. 
*/
  public Rotation2d getTargetHubAngle(){
     Optional<Alliance> ally = DriverStation.getAlliance();

    if (ally.isPresent()) {
      if (ally.get() == Alliance.Red) {
        alliance = Alliance.Red;
        targetX = Constants.TargettingConstants.RED_HUB.getX();
        targetY = Constants.TargettingConstants.RED_HUB.getY();
      }
      if (ally.get() == Alliance.Blue) {
        alliance = Alliance.Blue;
        targetX = Constants.TargettingConstants.BLUE_HUB.getX();
        targetY = Constants.TargettingConstants.BLUE_HUB.getY();
      }
    }

    currentRobotPosX = drive.getState().Pose.getX();
    currentRobotPosY = drive.getState().Pose.getY();

    
  if(alliance == Alliance.Blue && currentRobotPosX <= AimingConstants.blueAllianceTrench){
      targetTurretState = targetRotationHeading(targetX,targetY, currentRobotPosX, currentRobotPosY);
    }else{
      if(alliance == Alliance.Blue && currentRobotPosX >= AimingConstants.blueAllianceTrench && currentRobotPosY <= AimingConstants.middleLine){
        targetX = AimingConstants.blueRightFeedingTargetX;
        targetY = AimingConstants.blueRightFeedingTargetY;
         targetTurretState = targetRotationHeading(targetX,targetY, currentRobotPosX, currentRobotPosY);
      } 
      if(alliance == Alliance.Blue && currentRobotPosX >= AimingConstants.blueAllianceTrench && currentRobotPosY >= AimingConstants.middleLine){
        targetX = AimingConstants.blueLeftFeedingTargetX;
        targetY = AimingConstants.blueLeftFeedingTargetY;
        targetTurretState = targetRotationHeading(targetX,targetY, currentRobotPosX, currentRobotPosY);
      }
    }
    if(alliance == Alliance.Red && currentRobotPosX >= AimingConstants.redAllianceTrench){

      targetTurretState = targetRotationHeading(targetX,targetY, currentRobotPosX, currentRobotPosY);
    }else{
      if(alliance == Alliance.Red && currentRobotPosX <= AimingConstants.redAllianceTrench && currentRobotPosY >= AimingConstants.middleLine){
        targetX = AimingConstants.redRightFeedingTargetX;
        targetY = AimingConstants.redRightFeedingTargetY;
        targetTurretState = targetRotationHeading(targetX,targetY, currentRobotPosX, currentRobotPosY);
      }
      if(alliance == Alliance.Red && currentRobotPosX <= AimingConstants.redAllianceTrench && currentRobotPosY <= AimingConstants.middleLine ){
        targetX = AimingConstants.redLeftFeedingTargetX;
        targetY = AimingConstants.redLeftFeedingTargetY;
        targetTurretState = targetRotationHeading(targetX,targetY, currentRobotPosX, currentRobotPosY);
      }
      
    }
    targetTurretState = Math.toDegrees(MathUtil.angleModulus(Math.toRadians(targetTurretState)));
    currentTurretState = drive.getState().Pose.getRotation().getDegrees();
    return Rotation2d.fromDegrees(targetTurretState);
    
  }

  /**
   * Helper function to get the degrees of rotation to turn to in order to face a target
   * @param targetX
   * @param TargetY
   * @param currentPoseX
   * @param currentPoseY
   * @return Degrees to target
   */

  public double targetRotationHeading(Double targetX, Double TargetY, Double currentPoseX, Double currentPoseY){
     return Math.toDegrees(
            Math.atan2(targetY - currentPoseY, targetX -  currentPoseX)) + 180;
  }

  /**
   * Checks if rotation is in a tolerable range of where it needs to be.
   *
   * @return boolean - true or false depending on if its there or not.
   */
  public boolean isTargettingWithinTolerance() {
    return Math.abs(targetTurretState - currentTurretState) <= targettingTolerance;
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("Is the drivetrain facing the hub?", isTargettingWithinTolerance());
    SmartDashboard.putNumber("currrent robot heading is dis", currentTurretState);
    SmartDashboard.putNumber("Target Targetting State", targetTurretState);
    SmartDashboard.putNumber("rotation check", drive.getState().Pose.getRotation().getDegrees());
  }
}

