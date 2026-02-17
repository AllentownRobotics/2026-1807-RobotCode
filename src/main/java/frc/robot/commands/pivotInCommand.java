// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.pivotConsants;
import frc.robot.subsystems.GroundCollector.*;
/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class pivotInCommand extends Command {
  /** Creates a new pivotInCommand. */
  private final GroundCollector groundCollectionSubsystem;
  public pivotInCommand(GroundCollector subsystem) {
    groundCollectionSubsystem = subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    groundCollectionSubsystem.setPivotPosition(pivotConsants.startPosition);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    groundCollectionSubsystem.pivotMotorSpin();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    groundCollectionSubsystem.stopPivotMotor();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
