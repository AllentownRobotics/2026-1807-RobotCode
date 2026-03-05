// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
/* 
package frc.robot.subsystems.Vision;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;

public class AutoVisionSubsys extends SubsystemBase {
  NetworkTable climbTable = NetworkTableInstance.getDefault().getTable("limelight-back");
  NetworkTable trenchTable = NetworkTableInstance.getDefault().getTable("limelight-back");

  double climbRot;
  double climbTX;
  double climbTY;
  double climbTZ;

  double trenchRot;
  double trenchTX;
  double trenchTY;
  double trenchTZ;
  
  Pose2d climbPose, trenchPose;
  boolean climbTv, trenchTv;
  
 
  // Creates a new vision. 
  public AutoVisionSubsys() {
    climbTable.getEntry("priorityid").setNumber(-1);
    trenchTable.getEntry("priorityid").setNumber(-1);
  }

  public boolean isTargetClimb(){
    return climbTv;
  }

  public boolean isTargetTrench(){
    return trenchTv;
  }

  public Optional<Pose2d> climbTargetSpace(){
    if(!isTargetClimb()){
      return Optional.empty();
    }
    
    int climbExists = climbTv ? 1 : 0;

    Pose2d newClimbPose = climbPose.times(climbExists);

    Translation2d translationPoseClimb = newClimbPose.getTranslation();
    Rotation2d rotationPoseClimb = newClimbPose.getRotation();

    return Optional.of(new Pose2d(translationPoseClimb, rotationPoseClimb));
  }

  public boolean isNearClimbTargetSpace(Pose2d pose, double yDeadzone){
    if(!isTargetClimb()){
      return false;
    }

    Pose2d climbPose2d = climbTargetSpace().get();

    double xDistance = Math.abs(pose.getTranslation().getX() - climbPose2d.getTranslation().getX());
    double yDistance = Math.abs(pose.getTranslation().getY() - climbPose2d.getTranslation().getY());
    double angle = Math.abs(pose.getRotation().getDegrees() - climbPose2d.getRotation().getDegrees());

    boolean climbXInRange = (xDistance <= VisionConstants.xDistanceDeadzone);
    boolean climbYInRange = (yDistance <= yDeadzone);
    boolean climbRotInRange = (angle <= VisionConstants.angleDeadzone);
    
    boolean isClimbInRange = climbXInRange && climbYInRange && climbRotInRange;

    SmartDashboard.putNumber("x distance from aligned", xDistance);
    SmartDashboard.putNumber("y distance from aligned", yDistance);
    SmartDashboard.putNumber("angular distance from aligned", angle);

    SmartDashboard.putBoolean("x in range", climbXInRange);
    SmartDashboard.putBoolean("y in range", climbYInRange);
    SmartDashboard.putBoolean("rot in range", climbRotInRange);
    SmartDashboard.putBoolean("target in range", isClimbInRange);

    return isClimbInRange;
  }

  public Optional<Pose2d> trenchTargetSpace(){
    if(!isTargetTrench()){
      return Optional.empty();
    }
    
    int trenchExists = trenchTv ? 1 : 0;
    Pose2d newTrenchPose = trenchPose.times(trenchExists);

    Translation2d translationPoseTrench = newTrenchPose.getTranslation();
    Rotation2d rotationPoseTrench = newTrenchPose.getRotation();

    return Optional.of(new Pose2d(translationPoseTrench, rotationPoseTrench));
  }

  public boolean isNearTrenchTargetSpace(Pose2d trenchPose){
    if(!isTargetTrench()){
      return false;
    }

    Pose2d climbPose2d = climbTargetSpace().get();

    double xDistance = Math.abs(trenchPose.getTranslation().getX() - climbPose2d.getTranslation().getX());
    double yDistance = Math.abs(trenchPose.getTranslation().getY() - climbPose2d.getTranslation().getY());
    double angle = Math.abs(trenchPose.getRotation().getDegrees() - climbPose2d.getRotation().getDegrees());

    boolean trenchXInRange = (xDistance <= VisionConstants.xDistanceDeadzone);
    boolean trenchYInRange = (yDistance <= VisionConstants.yLDistanceDeadzone);
    boolean trenchRotInRange = (angle <= VisionConstants.angleDeadzone);
    
    boolean isTrenchInRange = trenchXInRange && trenchYInRange && trenchRotInRange;

    SmartDashboard.putNumber("x distance from aligned", xDistance);
    SmartDashboard.putNumber("y distance from aligned", yDistance);
    SmartDashboard.putNumber("angular distance from aligned", angle);

    SmartDashboard.putBoolean("x in range", trenchXInRange);
    SmartDashboard.putBoolean("y in range", trenchYInRange);
    SmartDashboard.putBoolean("rot in range", trenchRotInRange);
    SmartDashboard.putBoolean("target in range", isTrenchInRange);

    return isTrenchInRange;
  }

  @Override
  public void periodic() {
    climbTv = climbTable.getEntry("tv").getInteger(0)>0;
    trenchTv = trenchTable.getEntry("tv").getInteger(0)>0;

    climbRot = climbTable.getEntry("botpose_targetspace").getDoubleArray(new double[6])[4];
    climbTX = climbTable.getEntry("botpose_targetspace").getDoubleArray(new double[6])[0];
    climbTY = climbTable.getEntry("botpose_targetspace").getDoubleArray(new double[6])[1];
    climbTZ = climbTable.getEntry("botpose_targetspace").getDoubleArray(new double[6])[2];

    climbPose = new Pose2d(new Translation2d(climbTX, climbTZ), Rotation2d.fromDegrees(climbRot));

    trenchRot = trenchTable.getEntry("botpose_targetspace").getDoubleArray(new double[6])[4];
    trenchTX = trenchTable.getEntry("botpose_targetspace").getDoubleArray(new double[6])[0];
    trenchTY = trenchTable.getEntry("botpose_targetspace").getDoubleArray(new double[6])[1];
    trenchTZ = trenchTable.getEntry("botpose_targetspace").getDoubleArray(new double[6])[2];

    trenchPose = new Pose2d(new Translation2d(trenchTX, trenchTZ), Rotation2d.fromDegrees(trenchRot));

    SmartDashboard.putNumber("climb rotation offset", climbRot);
    SmartDashboard.putNumber("climbTX", climbTX);
    SmartDashboard.putNumber("climbTY", climbTY);
    SmartDashboard.putNumber("climbTZ", climbTZ);

    SmartDashboard.putNumber("trench rotation offset", trenchRot);
    SmartDashboard.putNumber("trenchTX", trenchTX);
    SmartDashboard.putNumber("trenchTY", trenchTY);
    SmartDashboard.putNumber("trenchTZ", trenchTZ);
  }
}
*/