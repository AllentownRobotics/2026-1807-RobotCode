// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Kicker.KickerSubsys;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class KickFuelCMD extends Command {
  private KickerSubsys kicker;//Establishes the kicker
  private final Timer timer = new Timer();
  /** Creates a new KickFuel. */
  public KickFuelCMD(KickerSubsys kicker) {
    this.kicker = kicker;//Instantiates the kicker
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(kicker);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  //TODO unsure if there should be an initialize, possibly just have an execute
  //TODO figure out how to keep the command running even is there are discrepencies in the flow of fuel
  public void execute() {
    
    // if (kicker.isFuelInKicker()) {
    //   kicker.kickFuel();
    //   timer.restart();
    // } 
    kicker.kickFuel();
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    kicker.stopKickerMotors();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
