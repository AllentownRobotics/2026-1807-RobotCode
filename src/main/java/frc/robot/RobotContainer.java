// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.retractPivotCommand;
import frc.robot.commands.extendPivotCommand;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.GroundCollector.GroundCollector;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
   // The robot's subsystems and commands are defined here...
   private final GroundCollector m_GroundCollectionSubsystem = new GroundCollector();
   //private final extendPivotCommand m_GroundCollectionCommand = new extendPivotCommand(m_GroundCollectionSubsystem);
   private final CommandXboxController m_xboxController = new CommandXboxController(0);
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
   /**
    *when pressing y on controller, the pivot motor spins to intake position
    */
    m_xboxController.y().whileTrue(
      new extendPivotCommand(m_GroundCollectionSubsystem)
    );
    /**
     *when pressing x on controller, the pivot motor spins to home position
     */
    m_xboxController.x().whileTrue(
      new retractPivotCommand(m_GroundCollectionSubsystem)
    );
  }
}
