// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.ClimbConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.ClimbCMDs.ClimbDownManual;
import frc.robot.commands.ClimbCMDs.ClimbIncrement;
import frc.robot.commands.ClimbCMDs.ClimbToHome;
import frc.robot.commands.ClimbCMDs.ClimbToL1;
import frc.robot.commands.ClimbCMDs.ClimbToL2;
import frc.robot.commands.ClimbCMDs.ClimbToL3;
import frc.robot.commands.ClimbCMDs.ClimbUpManual;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.Climb.ClimbSubsys;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
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
  private final ClimbSubsys climbSubsystem = new ClimbSubsys();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController operatorController =
    new CommandXboxController(OperatorConstants.kOperatorControllerPort);
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    //Goes to point slightly above L1
    NamedCommands.registerCommand("ClimbToL1Start", Commands.runOnce(() -> 
      climbSubsystem.setClimbSetpoint(ClimbConstants.L1AutoStartPosition), climbSubsystem)); 

    //Reverses and latches down to complete L1
    NamedCommands.registerCommand("ClimbToL1EndPos", Commands.runOnce(() -> 
      climbSubsystem.setClimbSetpoint(ClimbConstants.L1AutoEndPosition), climbSubsystem)); 

    //Goes back to home, must go back to L1 Start first
    NamedCommands.registerCommand("ClimbToHome", Commands.runOnce(() -> 
      climbSubsystem.setClimbSetpoint(ClimbConstants.climbHomePosition), climbSubsystem)); 

    //Waiting Commands
    NamedCommands.registerCommand("ClimbWaitforL1Start", Commands.waitUntil(climbSubsystem.isAtPosition(ClimbConstants.L1AutoStartPosition)));
    NamedCommands.registerCommand("ClimbWaitforL1EndPos", Commands.waitUntil(climbSubsystem.isAtPosition(ClimbConstants.L1AutoEndPosition)));
    NamedCommands.registerCommand("ClimbWaitforHome", Commands.waitUntil(climbSubsystem.isAtPosition(ClimbConstants.climbHomePosition)));

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
  

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.

    //Sequential Command Group so the climb goes to L1, then L2, then L3
    //operatorController.a().whileTrue(new ClimbToL1(climbSubsystem).andThen(new ClimbToL2(climbSubsystem).andThen(new ClimbToL3(climbSubsystem))));
  
    operatorController.povDown().whileTrue(Commands.runOnce(() -> 
      climbSubsystem.setClimbSetpoint(ClimbConstants.L1Position), climbSubsystem))
      .onFalse(Commands.runOnce(() -> 
      climbSubsystem.setClimbSetpoint(ClimbConstants.climbHomePosition)));

    operatorController.povLeft().whileTrue(Commands.runOnce(() -> 
      climbSubsystem.setClimbSetpoint(ClimbConstants.L1Position), climbSubsystem))
      .onFalse(Commands.runOnce(() -> 
      climbSubsystem.setClimbSetpoint(ClimbConstants.L1PositionLock), climbSubsystem));

      
    //L2 and L3 Commands
    /*operatorController.povUp().whileTrue(Commands.runOnce(() -> 
      climbSubsystem.setClimbSetpoint(ClimbConstants.L2Position), climbSubsystem))
      .onFalse(Commands.runOnce(() -> 
      climbSubsystem.setClimbSetpoint(ClimbConstants.L2PositionLock), climbSubsystem));

    operatorController.povRight().whileTrue(Commands.runOnce(() -> 
      climbSubsystem.setClimbSetpoint(ClimbConstants.L3Position), climbSubsystem))
      .onFalse(Commands.runOnce(() -> 
      climbSubsystem.setClimbSetpoint(ClimbConstants.L3PositionLock), climbSubsystem));*/


    /*operatorController.rightTrigger().onFalse(Commands.runOnce(() -> 
      climbSubsystem.setClimbSetpoint(ClimbConstants.L1PositionLock), climbSubsystem)); */  

    //operatorController.rightTrigger().onTrue(new ClimbToL1(climbSubsystem)); //Test

    //Manual Commands 
    operatorController.x().whileTrue(new ClimbUpManual(climbSubsystem));
    operatorController.y().whileTrue(new ClimbDownManual(climbSubsystem));
    operatorController.a().onTrue(Commands.runOnce(() -> 
      climbSubsystem.resetEncoderPos(), climbSubsystem)); 
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return null;
  }
}
