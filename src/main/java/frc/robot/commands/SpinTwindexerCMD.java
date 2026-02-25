// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.TwindexerSubsys.TwindexerSubsys;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class SpinTwindexerCMD extends Command {
  /** Creates a new SpinTwindexerCMD. */
  public TwindexerSubsys twindexerSubsys;
  public SpinTwindexerCMD(TwindexerSubsys twindexerSubsys) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.twindexerSubsys = twindexerSubsys;
    addRequirements(twindexerSubsys);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Sets motor's desired speed.
    twindexerSubsys.setTwindexerDesiredSpeed(Constants.TwindexerConstants.desiredTwindexerSpeed);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Spins motor to continuously reach desired speed using PID.
    twindexerSubsys.setTwindexerDesiredSpeed(Constants.TwindexerConstants.desiredTwindexerSpeed);
    twindexerSubsys.setTwindexerSpeed();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // Stops motor when command ends.
    twindexerSubsys.stopTwindexer();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
