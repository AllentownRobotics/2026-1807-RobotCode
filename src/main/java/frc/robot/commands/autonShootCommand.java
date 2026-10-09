// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.Commands;
// import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.IndexerSubsys.IndexerSubsys;
import frc.robot.subsystems.Kicker.KickerSubsys;
import frc.robot.subsystems.Shooter.FlywheelSubsys;
import frc.robot.subsystems.Shooter.HoodSubsys;
import frc.robot.subsystems.Shooter.TargettingSubsys;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class autonShootCommand extends Command {
  HoodSubsys hood;
  FlywheelSubsys flywheel;
  TargettingSubsys turret;
  KickerSubsys kicker;
  IndexerSubsys indexer;
  LEDSubsystem LEDs;

  /** Creates a new shootingSequence. */
  public autonShootCommand(HoodSubsys hood, FlywheelSubsys flywheel, LEDSubsystem LEDs, KickerSubsys kicker, IndexerSubsys indexer) {
    this.hood = hood;
    this.flywheel = flywheel;
    this.LEDs = LEDs;
    this.kicker = kicker;
    this.indexer = indexer;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(hood, flywheel, LEDs, kicker, indexer);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {


    

    flywheel.setFlywheelSpeedFromDistance();
    hood.setHoodAutomaticallyFromDistance();

    if(hood.HoodWithinTolerance() && flywheel.flywheelAtSpeed()){
        kicker.kickFuel();
        indexer.setIndexerSpeedRPS();
        // LEDs.setPattern(Constants.LEDConstants.LEDPatterns.ROBOT_IS_SHOOTING);
    }    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    
    hood.setHoodAngle(1);
    flywheel.stopSpeed();
    kicker.stopKickerMotors();
    indexer.stopIndexer();
    // LEDs.setPattern(Constants.LEDConstants.LEDPatterns.IDLE);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
