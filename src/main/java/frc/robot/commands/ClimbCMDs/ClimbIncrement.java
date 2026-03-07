// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ClimbCMDs;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climb.ClimbSubsys;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ClimbIncrement extends Command {
  ClimbSubsys climb;
  int increment;
  /** Creates a new ClimbIncrement. */
  public ClimbIncrement(ClimbSubsys climb, int increment) {
    this.climb = climb;
    this.increment = increment;

    addRequirements(climb);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    climb.adjustPositionIncremently(increment);
  }
}
