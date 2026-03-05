// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.TargetingCMDs;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import java.util.Optional;

import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.FieldCentric;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.VisionConstants;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Vision.AutoVisionSubsys;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class TargetClimb extends Command {
  AutoVisionSubsys limelight;
  double leftRightOffset;
  double topBottomOffset;

  PIDController sideToSideController;
  PIDController topToBottomController;
  PIDController rotationController;
  CommandXboxController driverController;

  CommandSwerveDrivetrain drivetrain;
  SwerveRequest.RobotCentric drive;
  SwerveRequest.FieldCentric driveFieldRelative;

  double maxSpeed;
  double MaxAngularRate;

  double slowDriveSpeed;
  double slowAngularRate;

  static Pose2d previousPose = null;

  /** Creates a new TargetClimb. */
  public TargetClimb(AutoVisionSubsys limelight, CommandSwerveDrivetrain drivetrain, CommandXboxController driverController, double leftRightOffset, double topBottomOffset) {
    //Declaring 
    this.limelight = limelight;
    this.drivetrain = drivetrain;
    this.driverController = driverController;
    this.leftRightOffset = leftRightOffset;
    this.topBottomOffset = topBottomOffset;

    maxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    MaxAngularRate = 5*(RotationsPerSecond.of(1).in(RotationsPerSecond));

    slowDriveSpeed = maxSpeed * TunerConstants.slowDriveScalingConstant;
    slowAngularRate = MaxAngularRate * TunerConstants.slowDriveScalingConstant;

    drive = new SwerveRequest.RobotCentric().withDriveRequestType(SwerveModule.DriveRequestType.OpenLoopVoltage);

    driveFieldRelative = new FieldCentric().withDriveRequestType(SwerveModule.DriveRequestType.OpenLoopVoltage);

    sideToSideController = new PIDController(VisionConstants.translation_kP, VisionConstants.translation_kI, VisionConstants.translation_kD);
    topToBottomController = new PIDController(VisionConstants.translation_kP, VisionConstants.translation_kI, VisionConstants.translation_kD);
    rotationController = new PIDController(VisionConstants.translation_kP, VisionConstants.translation_kI, VisionConstants.translation_kD);

    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    sideToSideController.reset();
    topToBottomController.reset();
    rotationController.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    Optional<Pose2d> pose = limelight.climbTargetSpace();

    if(pose.isPresent()){
      previousPose = pose.get();

      double topToBottomCalculation = topToBottomController.calculate(pose.get().getY(), topBottomOffset);
      double sideToSideCalculation = sideToSideController.calculate(pose.get().getX(), leftRightOffset);
      double rotationCalculation = rotationController.calculate(pose.get().getRotation().getRadians(), 0);

      SmartDashboard.putString("is using previous pose?", "no");

      SmartDashboard.putNumber("left right translation Pose", pose.get().getX());
      SmartDashboard.putNumber("rotation Pose", pose.get().getRotation().getRadians());
      SmartDashboard.putNumber("top bottom translation pose", pose.get().getY());
      SmartDashboard.putNumber("PID top bottom translation value", topToBottomCalculation);
      SmartDashboard.putNumber("PID left right translation value", sideToSideCalculation);
      SmartDashboard.putNumber("targeting PID rotation", rotationCalculation);

       drivetrain.applyRequest(() ->
        drive
        .withVelocityX(-topToBottomCalculation) 
        .withVelocityY(sideToSideCalculation)
        .withRotationalRate(rotationCalculation)
      ).execute();
      
        } else {
          drivetrain.applyRequest(() ->
          driveFieldRelative.withVelocityX(-driverController.getLeftY() * maxSpeed) // Drive forward with negative Y (forward)
          .withVelocityY(-driverController.getLeftX() * maxSpeed) // Drive left with negative X (left)
          .withRotationalRate(-driverController.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
      ).execute();

          if(previousPose != null) {
          SmartDashboard.putString("is using previous pose?", "yes");
    }
    else {

    }
  }
  SmartDashboard.putBoolean("has pose", pose.isPresent());
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
