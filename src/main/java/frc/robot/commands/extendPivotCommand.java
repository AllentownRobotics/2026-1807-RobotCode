// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

<<<<<<< HEAD:src/main/java/frc/robot/commands/ExampleCommand.java
=======
import frc.robot.Constants.pivotConsants;
import frc.robot.subsystems.GroundCollector.*;
>>>>>>> main:src/main/java/frc/robot/commands/extendPivotCommand.java
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LEDSubsystem;

/** An example command that uses an example subsystem. */
<<<<<<< HEAD:src/main/java/frc/robot/commands/ExampleCommand.java
public class ExampleCommand extends Command {
  @SuppressWarnings("PMD.UnusedPrivateField")
  private final LEDSubsystem m_subsystem;
=======
public class extendPivotCommand extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final GroundCollector groundCollectionSubsystem;
>>>>>>> main:src/main/java/frc/robot/commands/extendPivotCommand.java

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
<<<<<<< HEAD:src/main/java/frc/robot/commands/ExampleCommand.java
  public ExampleCommand(LEDSubsystem subsystem) {
    m_subsystem = subsystem;
=======
  public extendPivotCommand(GroundCollector subsystem) {
    groundCollectionSubsystem = subsystem;
>>>>>>> main:src/main/java/frc/robot/commands/extendPivotCommand.java
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  /**
   *sets pivot position when intaking
   */
  public void initialize() {
    groundCollectionSubsystem.setPivotPosition(pivotConsants.homePosition);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  /**
   *spins pivot motor
   */
  public void execute() {
    groundCollectionSubsystem.pivotMotorSpin();
  }

  // Called once the command ends or is interrupted.
  @Override
  /**
   *stops pivot motor
   */
  public void end(boolean interrupted) {
     groundCollectionSubsystem.stopPivotMotor();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}