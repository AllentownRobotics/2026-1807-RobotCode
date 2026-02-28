// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.units.Units.*;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Drive.CommandSwerveDrivetrain;
import frc.robot.subsystems.Shooter.Turret;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class autoAimHub extends Command {
  /** Creates a new autoAimHUB. */
  CommandSwerveDrivetrain drivetrain;
  CommandXboxController driverController;
  Turret turret;
  PIDController thetaController;
  double targetHubAngle;
  private final SwerveRequest.FieldCentric drive;
  private double MaxSpeed;
  private double MaxAngularRate;

  public autoAimHub(CommandSwerveDrivetrain drivetrain, CommandXboxController driverController, Turret turret) {
    this.drivetrain = drivetrain;
    this.driverController = driverController;
    this.turret = turret;

    MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity
    
    drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

    thetaController = new PIDController(1, 0, 0);
    thetaController.enableContinuousInput(-180, 180);
    addRequirements(drivetrain);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    thetaController.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    targetHubAngle = turret.getTargetTurretAngle();

    double thetaCalculation = thetaController.calculate(drivetrain.getPigeon2().getRotation2d().getDegrees(), targetHubAngle);

    drivetrain.applyRequest(() -> drive.withVelocityX(-driverController.getLeftY())
                                      .withVelocityY(-driverController.getLeftX())
                                      .withRotationalRate(-thetaCalculation)).execute();

   
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