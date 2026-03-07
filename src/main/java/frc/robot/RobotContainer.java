// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.ClimbConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.ClimbCMDs.ClimbIncrement;
import frc.robot.commands.ClimbCMDs.ClimbToHome;
import frc.robot.commands.ClimbCMDs.ClimbToL1;
import frc.robot.commands.ClimbCMDs.ClimbToL2;
import frc.robot.commands.ClimbCMDs.ClimbToL3;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.Climb.ClimbSubsys;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
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
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  private final ClimbSubsys climbSubsystem = new ClimbSubsys();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    NamedCommands.registerCommand("ClimbToHome", new ClimbToHome(climbSubsystem));
    NamedCommands.registerCommand("ClimbToL1", new ClimbToL1(climbSubsystem));
    NamedCommands.registerCommand("ClimbToL2", new ClimbToL2(climbSubsystem));
    NamedCommands.registerCommand("ClimbToL3", new ClimbToL3(climbSubsystem));
    NamedCommands.registerCommand("ClimbWaitForL1", new WaitUntilCommand(climbSubsystem.isAtPosition(ClimbConstants.L1Position)));
    NamedCommands.registerCommand("ClimbWaitForL2", new WaitUntilCommand(climbSubsystem.isAtPosition(ClimbConstants.L2Position)));
    NamedCommands.registerCommand("ClimbWaitForL3", new WaitUntilCommand(climbSubsystem.isAtPosition(ClimbConstants.L3Position)));
    NamedCommands.registerCommand("ClimbWaitForHome", new WaitUntilCommand(climbSubsystem.isAtPosition(ClimbConstants.climbHomePosition)));

    NamedCommands.registerCommand("Climb to L1", new ClimbToL1(climbSubsystem));
    NamedCommands.registerCommand("Climb to L2", new ClimbToL2(climbSubsystem));
    NamedCommands.registerCommand("Climb to L3", new ClimbToL3(climbSubsystem));

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
    //Need to do this still 
    
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    new Trigger(m_exampleSubsystem::exampleCondition)
        .onTrue(new ExampleCommand(m_exampleSubsystem));

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
    m_driverController.b().whileTrue(m_exampleSubsystem.exampleMethodCommand());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return Autos.exampleAuto(m_exampleSubsystem);
  }
}
