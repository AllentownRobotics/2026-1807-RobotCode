// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

// import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.LEDConstants;
import frc.robot.Constants.operatorConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.LEDSubsystem;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
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

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_operatorController =
      new CommandXboxController(operatorConstants.operatorController);//creates new operator controller

  private final LEDSubsystem LEDSubsystem = new LEDSubsystem();//makes new LEDSubsystem

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    //registers commands for every pattern
    NamedCommands.registerCommand("LEDPatternOff", new InstantCommand(()-> LEDSubsystem.setPattern(Constants.LEDConstants.LEDPatterns.OFF), LEDSubsystem));
    NamedCommands.registerCommand("LEDPatternIdle", new InstantCommand(()-> LEDSubsystem.setPattern(Constants.LEDConstants.LEDPatterns.IDLE), LEDSubsystem));
    NamedCommands.registerCommand("LEDPatternAlignedWithHub", new InstantCommand(()-> LEDSubsystem.setPattern(Constants.LEDConstants.LEDPatterns.ALIGNED_WITH_HUB), LEDSubsystem));
    NamedCommands.registerCommand("LEDPatternClimbComplete", new InstantCommand(()-> LEDSubsystem.setPattern(Constants.LEDConstants.LEDPatterns.CLIMB_COMPLETE), LEDSubsystem));
    NamedCommands.registerCommand("LED PatternClimbCompleteRed", new InstantCommand(()-> LEDSubsystem.setPattern(Constants.LEDConstants.LEDPatterns.CLIMB_COMPLETE_RED), LEDSubsystem));
    NamedCommands.registerCommand("LED PatternClimbCompleteBlue", new InstantCommand(()-> LEDSubsystem.setPattern(Constants.LEDConstants.LEDPatterns.CLIMB_COMPLETE_BLUE), LEDSubsystem));
    NamedCommands.registerCommand("LEDPatternPivotAtIntakePosition", new InstantCommand(()-> LEDSubsystem.setPattern(Constants.LEDConstants.LEDPatterns.PIVOT_AT_INTAKE_POSITION), LEDSubsystem));
    NamedCommands.registerCommand("LEDPatternPivotAtHomePosition", new InstantCommand(()-> LEDSubsystem.setPattern(Constants.LEDConstants.LEDPatterns.PIVOT_AT_HOME_POSITION), LEDSubsystem));
    NamedCommands.registerCommand("LEDPatternFuelReadyToShoot", new InstantCommand(()-> LEDSubsystem.setPattern(Constants.LEDConstants.LEDPatterns.FUEL_READY_TO_SHOOT), LEDSubsystem));
    NamedCommands.registerCommand("LEDPatternFuelInTwindexer", new InstantCommand(()-> LEDSubsystem.setPattern(Constants.LEDConstants.LEDPatterns.FUEL_IN_TWINDEXER), LEDSubsystem));
    NamedCommands.registerCommand("LEDPatternAlignedWithHumanPlayerStation", new InstantCommand(()-> LEDSubsystem.setPattern(Constants.LEDConstants.LEDPatterns.ALIGNED_WITH_HUMAN_PLAYER_STATION), LEDSubsystem));

    SmartDashboard.putData(LEDSubsystem);//puts data into smart dashboard
        
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
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    /**
     * sets pattern alignedWithHumanPlayerStation when you press the start button
     */
    m_operatorController.start().whileTrue(Commands.run(
            () -> LEDSubsystem.setPattern(
                LEDConstants.LEDPatterns.ALIGNED_WITH_HUMAN_PLAYER_STATION), LEDSubsystem));

        LEDSubsystem.setDefaultCommand(Commands.runOnce(
            () -> LEDSubsystem.setPattern(
                LEDConstants.LEDPatterns.IDLE), LEDSubsystem));//set idle as default pattern
    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
      return null;
    // An example command will be run in autonomous
  }
}
