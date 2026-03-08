// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoAimingCommands;

import static edu.wpi.first.units.Units.*;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Drive.CommandSwerveDrivetrain;
import frc.robot.subsystems.Shooter.TurretSubsys;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class TeleopAutoAimHub extends Command {
  /** Creates a new autoAimHUB. */
  CommandSwerveDrivetrain drivetrain;
  CommandXboxController driverController;
  TurretSubsys turret;
  PIDController thetaController;
  double targetHubAngle;
  private final SwerveRequest.FieldCentric drive;
  private double MaxSpeed;
  private double MaxAngularRate;
  private double rotationRate;

  public TeleopAutoAimHub(CommandSwerveDrivetrain drivetrain, CommandXboxController driverController, TurretSubsys turret) {
    this.drivetrain = drivetrain;
    this.driverController = driverController;
    this.turret = turret;
    
    MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    MaxAngularRate = RotationsPerSecond.of(1).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity
    
    drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.10) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

    // PID controller for theta or rotational movement
    thetaController = new PIDController(Constants.AimingConstants.driveHubAutoAimkP
                                       ,Constants.AimingConstants.driveHubAutoAimkI
                                       ,Constants.AimingConstants.driveHubAutoAimkD);
    // wraps from -180 to 180 so we never take the "long" route to get to a setpoint
    thetaController.enableContinuousInput(-180, 180);
    addRequirements(drivetrain);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    thetaController.reset(); // doing a whole lot of resetting
    SmartDashboard.putBoolean("Did this command start?", true);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // compensate for robot facing the right direction.
    targetHubAngle = turret.getTargetTurretAngle() + 180;
    
    // calculate the PID gains we need, feed that in for our turning rate to turn to a specific position
    double thetaCalculation = thetaController.calculate(-drivetrain.getState().Pose.getRotation().getDegrees(), -targetHubAngle);
    // variable used for tolerance
    rotationRate = thetaCalculation;

    // if statement saying if the difference between our target and current is below 5 degrees, we can stop rotating
    if(Math.abs(targetHubAngle  - drivetrain.getState().Pose.getRotation().getDegrees() - 180)  <= 7){
      rotationRate = 0;
    }
    // applies the request to be able to drive while aiming
    drivetrain.applyRequest(() -> drive.withVelocityX(-driverController.getLeftY() * MaxSpeed)
                                      .withVelocityY(-driverController.getLeftX() * MaxSpeed)
                                      .withRotationalRate(rotationRate)).execute();
    // smart dash to see current error
    SmartDashboard.putNumber("auto hub error", Math.abs(targetHubAngle  - drivetrain.getState().Pose.getRotation().getDegrees() - 180));
    
        // add Driver feedback here
    // if(Math.abs(targetHubAngle  - drivetrain.getState().Pose.getRotation().getDegrees() - 180) <= 5){
    //     // make LEDS turn green here, or any sort of bright color to signify it has been targetted. 
    // }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // make LEDS go VROOOOM
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}