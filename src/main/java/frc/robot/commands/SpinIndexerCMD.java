// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IndexerSubsys.IndexerSubsys;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class SpinIndexerCMD extends Command {
  public IndexerSubsys indexerSubsys;

  /** Creates a new SpinIndexerCMD. */
  public SpinIndexerCMD(IndexerSubsys indexerSubsys) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.indexerSubsys = indexerSubsys;

    addRequirements(indexerSubsys);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Sets motor's desired speed.
    // indexerSubsys.setIndexerDesiredSpeed(Constants.IndexerConstants.desiredIndexerSpeed); 

    indexerSubsys.setIndexerSpeed();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Continuously sets the setpoint so that the motor keeps aiming to reach the desired speed.
    // indexerSubsys.setIndexerDesiredSpeed(Constants.IndexerConstants.desiredIndexerSpeed);
    
    // Uses PID to set motor speed.
    indexerSubsys.setIndexerSpeed();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    indexerSubsys.stopIndexer(); // Stops motor when command ends.
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
