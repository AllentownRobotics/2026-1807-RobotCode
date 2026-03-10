// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ClimbCMDs;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ClimbConstants;
import frc.robot.subsystems.Climb.ClimbSubsys;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ClimbToL1 extends Command {
  ClimbSubsys climb;
  BooleanSupplier isAtL1; 
  /** Creates a new ClimbToL1. */
  public ClimbToL1(ClimbSubsys climb) {
    this.climb = climb;

    //Checks if climb is at the L1Position which it has to reach before it reverses back
    this.isAtL1 = climb.isAtPosition(ClimbConstants.L1Position);

    addRequirements(climb);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //Uses PID to move climb to L1Position
    climb.setClimbSetpoint(ClimbConstants.L1Position);
  }

  @Override
  public void execute(){
    //If the climb has reached the L1Position, it can then be brought from there to the L1LockPosition 
    if(isAtL1.getAsBoolean()){
      climb.setClimbSetpoint(ClimbConstants.L1PositionLock);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
  //If climb reached L1positionLock then the command is over
    return climb.isAtPosition(ClimbConstants.L1PositionLock).getAsBoolean();
  }
}
