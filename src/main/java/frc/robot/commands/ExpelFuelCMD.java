// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Kicker.KickerSubsys;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ExpelFuelCMD extends Command {
  private KickerSubsys kicker;//Establishes the kicker
  /** Creates a new ExpelFuel. */
  public ExpelFuelCMD(KickerSubsys kicker) {
    this.kicker = kicker;//Instantiates the kicker
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(kicker);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    kicker.expelFuel();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    kicker.stopMotors();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
