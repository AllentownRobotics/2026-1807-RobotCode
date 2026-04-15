// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoAimingCommands;

import static edu.wpi.first.units.Units.*;

// import java.util.TimerTask;

// import java.security.PublicKey;
// import java.util.function.Function;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
// import com.pathplanner.lib.events.OneShotTriggerEvent;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
// import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
// import edu.wpi.first.wpilibj.Timer;
// import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
// import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
// import frc.robot.Constants;
import frc.robot.Constants.AimingConstants;
import frc.robot.Constants.LEDConstants.LEDPatterns;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.Drive.CommandSwerveDrivetrain;
import frc.robot.subsystems.Shooter.TurretSubsys;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class TeleopAutoAimHub extends Command {
  /** Creates a new autoAimHUB. */
  CommandSwerveDrivetrain drivetrain;
  CommandXboxController driverController;
  CommandXboxController operatorController;
  TurretSubsys turret;
  LEDSubsystem LEDs; 
  // PIDController thetaController;
  double targetHubAngle;
  private final SwerveRequest.FieldCentric drive;
  private double MaxSpeed;
  private double MaxAngularRate;
  private double rotationRate;
  // private TrapezoidProfile headingControl;
  private ProfiledPIDController thetaController;

  public TeleopAutoAimHub(CommandSwerveDrivetrain drivetrain, CommandXboxController driverController, TurretSubsys turret, CommandXboxController operatorController, LEDSubsystem LEDs) {
    this.drivetrain = drivetrain;
    this.driverController = driverController;
    this.turret = turret;
    this.operatorController = operatorController;
    this.LEDs = LEDs;
    
    MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    MaxAngularRate = RotationsPerSecond.of(1).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity
    
    drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

    // PID controller for theta or rotational movement
    // thetaController = new PIDController(Constants.AimingConstants.driveHubAutoAimkP
    //                                    ,Constants.AimingConstants.driveHubAutoAimkI
    //                                    ,Constants.AimingConstants.driveHubAutoAimkD);
    
    // headingControl = new TrapezoidProfile(new Constraints(MaxAngularRate, MaxAngularRate * 5));
    thetaController = new ProfiledPIDController(AimingConstants.driveHubAutoAimkP, AimingConstants.driveHubAutoAimkI, AimingConstants.driveHubAutoAimkD, new Constraints(MaxSpeed, MaxAngularRate * 5));
    
    // wraps from -180 to 180 so we never take the "long" route to get to a setpoint
    thetaController.enableContinuousInput(-Math.PI, Math.PI);
    
    addRequirements(drivetrain);
    // Use addRequirements() here to declare subsystem dependencies.
  }

// {
// Function;
// Zarif - hair = Robot wins worlds!

// then: explode 

// if collectorExtendo
// then: targetHub
// }
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    State currentState = new State(MathUtil.angleModulus(drivetrain.getState().Pose.getRotation().getRadians()), drivetrain.getState().Speeds.omegaRadiansPerSecond);
    thetaController.reset(currentState); // doing a whole lot of resetting
    SmartDashboard.putBoolean("Did this command start?", true);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // compensate for robot facing the right direction.
    targetHubAngle = turret.getTargetTurretAngle().getRadians();
    // Rotation2d.fromRotations(targetHubAngle).minus(Rotation2d.kZero);

    // State currentState = new State(MathUtil.angleModulus(drivetrain.getState().Pose.getRotation().getRadians()), drivetrain.getState().Speeds.omegaRadiansPerSecond);
    // State targetState = new State(MathUtil.angleModulus(targetHubAngle), 0);

    // State nextPosition = headingControl.calculate(.02, currentState, targetState);
    // state nextPos = thetaController.ca

    double profiledPIDControllerOutput = thetaController.calculate(drivetrain.getState().Pose.getRotation().getRadians(), targetHubAngle);
    
    // calculate the PID gains we need, feed that in for our turning rate to turn to a specific position
    // double thetaCalculation = thetaController.calculate(drivetrain.getState().Pose.getRotation().getDegrees(), nextPosition.position);
    // variable used for tolerance
    rotationRate = profiledPIDControllerOutput; 

    // if statement saying if the difference between our target and current is below 5 degrees, we can stop rotating
    if(Math.abs(targetHubAngle  - drivetrain.getState().Pose.getRotation().getDegrees() - 180)  <= 5){
      rotationRate = 0;
    }
    // applies the request to be able to drive while aiming
    drivetrain.applyRequest(() -> drive.withVelocityX(-driverController.getLeftY() * MaxSpeed)
                                      .withVelocityY(-driverController.getLeftX() * MaxSpeed)
                                      .withRotationalRate(thetaController.getSetpoint().velocity + rotationRate)).execute();
    // smart dash to see current error
    SmartDashboard.putNumber("auto hub error", Math.abs(targetHubAngle  - drivetrain.getState().Pose.getRotation().getDegrees() - 180));
    
        // add Driver feedback here
     if(Math.abs(targetHubAngle  - drivetrain.getState().Pose.getRotation().getDegrees() - 180) <= 2){
         LEDs.setPattern(Constants.LEDConstants.LEDPatterns.ALIGNED_WITH_HUB);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    LEDs.setPattern(LEDPatterns.IDLE);
    // make LEDS go VROOOOM
    // if(turret.isTurretWithinTolerance()){
    //     operatorController.setRumble(RumbleType.kBothRumble, 0.5);
    //     new WaitCommand(1).andThen(() -> operatorController.getHID().setRumble(RumbleType.kBothRumble, 0));
    // }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}