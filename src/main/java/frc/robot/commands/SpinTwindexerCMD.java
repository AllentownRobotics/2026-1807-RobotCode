// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.TwindexerSubsys.TwindexerSubsys;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class SpinTwindexerCMD extends Command {
  public TwindexerSubsys twindexerSubsys;

  /** Creates a new SpinTwindexerCMD. */
  public SpinTwindexerCMD(TwindexerSubsys twindexerSubsys) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.twindexerSubsys = twindexerSubsys;

    addRequirements(twindexerSubsys);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    twindexerSubsys.setTwindexerDesiredSpeed(Constants.TwindexerConstants.desiredTwindexerSpeed); // Sets motor's desired speed.
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Continuously sets setpoint so that the motor keeps aiming to reach the desired speed.
    twindexerSubsys.setTwindexerDesiredSpeed(Constants.TwindexerConstants.desiredTwindexerSpeed);
    
    // Uses PID to set motor speed.
    twindexerSubsys.setTwindexerSpeed();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    twindexerSubsys.stopTwindexer(); // Stops motor when command ends.
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
