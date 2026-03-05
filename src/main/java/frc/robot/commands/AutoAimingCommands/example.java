// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoAimingCommands;

import java.util.Optional;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drive.CommandSwerveDrivetrain;
import frc.robot.subsystems.Shooter.Turret;
import frc.robot.Constants;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class example extends Command {
  /** Creates a new example. */

  Turret turret;
  CommandSwerveDrivetrain drivetrain;

  double targetX;
  double targetY;

  public example(Turret turret, CommandSwerveDrivetrain drivetrain) {
    this.turret = turret;
    this.drivetrain = drivetrain;

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
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
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.putNumber("Drive current pose X", drivetrain.getState().Pose.getX());
    SmartDashboard.putNumber("Drive current pose Y", drivetrain.getState().Pose.getY());
    SmartDashboard.putNumber("Drive target pose Y", targetX);
    SmartDashboard.putNumber("Drive target Pose Y", targetY);
    SmartDashboard.putNumber("target angle", turret.getTargetTurretAngle());
    SmartDashboard.putNumber("Pigeon reading", drivetrain.getState().Pose.getRotation().getDegrees());


    // SmartDashboard.putNumber("Drive current pose X", drivetrain.getState().Pose.getX());
    // SmartDashboard.putNumber("Drive current pose Y", drivetrain.getState().Pose.getY());
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
