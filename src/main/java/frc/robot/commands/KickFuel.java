// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Kicker.Kicker;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class KickFuel extends Command {
  private Kicker kicker;
  /** Creates a new KickFuel. */
  public KickFuel(Kicker kicker) {
    this.kicker = kicker;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(kicker);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (kicker.getSensorValue()) {
      kicker.kickFuel();
    } else {
      kicker.stopMotors();
    }
  
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (kicker.getSensorValue()) {
      kicker.kickFuel();
    } else {
      kicker.stopMotors();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
