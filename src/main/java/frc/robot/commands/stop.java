// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.Constants.pivotConsants;
import frc.robot.subsystems.GroundCollector.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class stop extends Command {
  private final GroundCollector groundCollectionSubsystem;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public stop(GroundCollector subsystem) {
    groundCollectionSubsystem = subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  /**
   *sets pivot position when intaking
   */
  public void initialize() {
    // groundCollectionSubsystem.setPivotPosition(-0.21);
    // groundCollectionSubsystem.setPivotPosition(pivotConsants.pivotOutPosition);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  /**
   *spins pivot motor
   */
  public void execute() {
    // double error = groundCollectionSubsystem.getPivotPosition() / 360.0;
    
    // error = -0.21 - error;
    // if(Math.abs(error) < 0.1){
    //   groundCollectionSubsystem.drivePivotVolts(0.6);
    // }else{
    
    // groundCollectionSubsystem.setPivotPositionFeedforwards(-0.21,0.6);
    // SmartDashboard.putNumber("error", error);
    // }
    groundCollectionSubsystem.stopPivotMotor();
  }

  // Called once the command ends or is interrupted.
  @Override
  /**
   *stops pivot motor
   */
  public void end(boolean interrupted) {
     groundCollectionSubsystem.stopPivotMotor();
  }

  // Returns the intake position when the command should end.
  @Override
  public boolean isFinished() {
    return groundCollectionSubsystem.isAtPosition(-0.21);
    // return false;
  }
}