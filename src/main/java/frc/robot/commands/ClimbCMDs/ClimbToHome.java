// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ClimbCMDs;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ClimbConstants;
import frc.robot.subsystems.Climb.ClimbSubsys;
/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ClimbToHome extends Command {
  ClimbSubsys climb;
  BooleanSupplier isAtL1;
  /** Creates a new ClimbToHome. */
  public ClimbToHome(ClimbSubsys climb) {
    this.climb = climb;
    this.isAtL1 = climb.isAtPosition(ClimbConstants.L1Position);

    addRequirements(climb);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    climb.setClimbSetpoint(ClimbConstants.L1Position);
  }

  @Override 
  public void execute(){
    if(isAtL1.getAsBoolean()){
      climb.setClimbSetpoint(ClimbConstants.climbHomePosition);
    }
  }
}
