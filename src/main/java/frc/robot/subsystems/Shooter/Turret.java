// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Shooter;

import com.ctre.phoenix6.hardware.CANcoder;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.Drive.CommandSwerveDrivetrain;
// import frc.robot.subsystems.CommandSwerveDrivetrain;
// import frc.robot.utils.FieldConstants;
import frc.robot.utils.Kraken;
import java.util.Optional;

public class Turret extends SubsystemBase {
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

  public Turret(CommandSwerveDrivetrain drive) {
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
   * Calculates the field Relative angle to the hub using a hard coded hub constant then subtracts
   * current robot heading to turn that into a robot relative angle. In doing so allows us to set
   * the encoder to a robot relative position. If set as a default command allows you to consantly
   * track the hub using PID values.
   *
   * @return nothing cuz it a void
   */
  public void trackHUB() {

    // Alliance shift, calculates the target X and target Y of the turret for angle calculation
    Optional<Alliance> ally = DriverStation.getAlliance();
    if (ally.isPresent()) {
      if (ally.get() == Alliance.Red) {
        targetX = Constants.turretConstants.RED_HUB.getX();
        targetY = Constants.turretConstants.RED_HUB.getY();
      }
      if (ally.get() == Alliance.Blue) {
        targetX = Constants.turretConstants.BLUE_HUB.getX();
        targetY = Constants.turretConstants.BLUE_HUB.getY();
      }
    }
    // current absolute encoder reading
    currentTurretState = turretEncoder.getAbsolutePosition().getValueAsDouble() * 360;
    robotHeading = drive.getState().Pose.getRotation().getDegrees();

    /*calculates robot relative angle by taking the inverse tan between the hub and the robot, then by subtracting
     * robot heading allows you to get a robot relative angle*/
    targetTurretState =
        Math.toDegrees(
            Math.atan2(targetY -  drive.getState().Pose.getY(), targetX -  drive.getState().Pose.getX())
                - drive.getPigeon2().getRotation2d().getRadians());

    
    // only allows angles between -pi to +pi because every angle after that can be reprsensted by a
    // smaller angle in that interval
    targetTurretState = Math.toDegrees(MathUtil.angleModulus(Math.toRadians(targetTurretState)));
    // Clamps between -25 and 25 so the degrees returned can never be above 25 or below -25.
    targetTurretState = MathUtil.clamp(targetTurretState, -360, 360);
    // tells the encoder to get to that target state. divided by 360 because it wants rotations and
    // turretState returns a degree
    turretMotor.setDesiredEncoderPosition(targetTurretState / 360);

    // various smartDashboard variables to test / tune
    SmartDashboard.putNumber("Turret current state", currentTurretState);
    SmartDashboard.putNumber("Turret target state", targetTurretState);
    SmartDashboard.putNumber("posetargetX", drive.getState().Pose.getX());
    SmartDashboard.putNumber("posetargetY", drive.getState().Pose.getY());
    SmartDashboard.putNumber("targetX", targetX);
    SmartDashboard.putNumber("targetY", targetY);
    SmartDashboard.putNumber("Pidgeon reading", robotHeading);
  }

  public double getTargetTurretAngle(){
     Optional<Alliance> ally = DriverStation.getAlliance();
    if (ally.isPresent()) {
      if (ally.get() == Alliance.Red) {
        targetX = Constants.turretConstants.RED_HUB.getX();
        targetY = Constants.turretConstants.RED_HUB.getY();
      }
      if (ally.get() == Alliance.Blue) {
        targetX = Constants.turretConstants.BLUE_HUB.getX();
        targetY = Constants.turretConstants.BLUE_HUB.getY();
      }
    }
    // currentTurretState = turretEncoder.getAbsolutePosition().getValueAsDouble() * 360;
    /*calculates robot relative angle by taking the inverse tan between the hub and the robot, then by subtracting
     * robot heading allows you to get a robot relative angle*/
    targetTurretState =
        Math.toDegrees(
            Math.atan2(targetY - drive.getState().Pose.getY(), targetX - drive.getState().Pose.getX())); 

    // turretPositionError = targetTurretState - currentTurretState;
    // only allows angles between -pi to +pi because every angle after that can be reprsensted by a
    // smaller angle in that interval
    // targetTurretState = Math.toDegrees(MathUtil.angleModulus(Math.toRadians(targetTurretState)));

    return targetTurretState;
  }


  /**
   * Checks if turret is in a tolerable range of where it needs to be.
   *
   * @return boolean - true or false depending on if its there or not.
   */
  public boolean isTurretWithinTolerance() {
    if (Math.abs(targetTurretState - currentTurretState) <= turretTolerance) {
      return true;
    } else {
      return false;
    }
  }

  @Override
  public void periodic() {
    // System.out.println("Yo sim so cool");
  }
}
