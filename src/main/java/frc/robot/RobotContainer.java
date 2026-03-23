// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;
import frc.robot.Constants;
import frc.robot.commands.collectorMoveVoltsIn;
import frc.robot.commands.collectorMoveVoltsOut;
// import frc.robot.commands.collectorMoveVoltsIn;
// import frc.robot.commands.collectorMoveVoltsOut;
// import frc.robot.commands.retractPivotCommand;
// import frc.robot.commands.runCollectorCommands;
// import frc.robot.commands.stop;
// import frc.robot.commands.collectorVoltage;
// import frc.robot.commands.extendPivotCommand;
import frc.robot.commands.runCollectorCommands;
import frc.robot.generated.TunerConstants;
import frc.robot.Constants.ClimbConstants;
// import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.LEDConstants;
// import frc.robot.Constants.operatorConstants;
// import frc.robot.commands.ExpelFuelCMD;
// import frc.robot.commands.AutosSpinIndexerCMD;
// import frc.robot.commands.KickFuelCMD;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.Drive.CommandSwerveDrivetrain;
// import frc.robot.commands.SpinIndexerCMD;
// import frc.robot.subsystems.Kicker.KickerSubsys;
import frc.utils.ButtonBoard;
import frc.robot.subsystems.GroundCollector.GroundCollector;
// import frc.robot.subsystems.GroundCollector.GroundCollector;
// import frc.robot.subsystems.GroundCollector.newCollector;
// import frc.robot.subsystems.IndexerSubsys.IndexerSubsys;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
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
  // The robot's subsystems and commands are defined here...
  // private final KickerSubsys m_kicker = new KickerSubsys();
  // private final IndexerSubsys m_indexerSubsystem = new IndexerSubsys();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
      new CommandXboxController(Constants.kDriverControllerPort);
  // private final SpinIndexerCMD m_SpinIndexerCMD = new SpinIndexerCMD(m_indexerSubsystem);
  // private final AutosSpinIndexerCMD m_AutosSpinIndexerCMD = new AutosSpinIndexerCMD(m_indexerSubsystem);

  // private final runCollectorCommands m_RunCollectorCommands = new runCollectorCommands(m_GroundCollectionSubsystem);
  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_operatorController =
      new CommandXboxController(Constants.operatorConstants.operatorController);//creates new operator controller

  private final LEDSubsystem LEDSubsystem = new LEDSubsystem();//makes new LEDSubsystem
  // private final ExpelFuelCMD m_expelFuel = new ExpelFuelCMD(m_kicker);
  // private final KickFuelCMD m_kickFuel = new KickFuelCMD(m_kicker);

  final ButtonBoard m_buttonboard = new ButtonBoard(0);
  private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate =  RotationsPerSecond.of(1).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity
    private double slowSpeed = 0.5 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController joystick = new CommandXboxController(0);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    //registers commands for every pattern
    NamedCommands.registerCommand("LEDPatternOff", new InstantCommand(()-> LEDSubsystem.setPattern(Constants.LEDConstants.LEDPatterns.OFF), LEDSubsystem));
    NamedCommands.registerCommand("LEDPatternIdle", new InstantCommand(()-> LEDSubsystem.setPattern(Constants.LEDConstants.LEDPatterns.IDLE), LEDSubsystem));
    NamedCommands.registerCommand("LEDPatternAlignedWithHub", new InstantCommand(()-> LEDSubsystem.setPattern(Constants.LEDConstants.LEDPatterns.ALIGNED_WITH_HUB), LEDSubsystem));
    NamedCommands.registerCommand("LEDPatternClimbComplete", new InstantCommand(()-> LEDSubsystem.setPattern(Constants.LEDConstants.LEDPatterns.CLIMB_COMPLETE), LEDSubsystem));
    // NamedCommands.registerCommand("LED PatternClimbCompleteRed", new InstantCommand(()-> LEDSubsystem.setPattern(Constants.LEDConstants.LEDPatterns.CLIMB_COMPLETE_RED), LEDSubsystem));
    // NamedCommands.registerCommand("LED PatternClimbCompleteBlue", new InstantCommand(()-> LEDSubsystem.setPattern(Constants.LEDConstants.LEDPatterns.CLIMB_COMPLETE_BLUE), LEDSubsystem));
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
  //Trigger value = m_buttonboard.getButton(0);
  private void configureBindings() {

    // m_xboxController.leftBumper().onTrue(Commands.runOnce(SignalLogger::start));
    // m_xboxController.rightBumper().onTrue(Commands.runOnce(SignalLogger::stop));

    // m_xboxController.start().and(m_xboxController.a()).whileTrue(m_GroundCollectionSubsystem.sysIdQuasistatic(Direction.kForward));
    // m_xboxController.start().and(m_xboxController.b()).whileTrue(m_GroundCollectionSubsystem.sysIdQuasistatic(Direction.kReverse));
    //  m_xboxController.back().and(m_xboxController.a()).whileTrue(m_GroundCollectionSubsystem.sysIdDynamic(Direction.kForward));
    // m_xboxController.back().and(m_xboxController.b()).whileTrue(m_GroundCollectionSubsystem.sysIdDynamic(Direction.kReverse));
    
    drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-joystick.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );



    

    // m_xboxController.start().whileTrue(new collectorVoltage(m_GroundCollectionSubsystem));
   /**
    *when pressing y on controller, the pivot motor spins to intake position
    */
    // m_xboxController.y().whileTrue(
    //   new extendPivotCommand(m_GroundCollectionSubsystem)
    // );
    /**
     *when pressing x on controller, the pivot motor spins to home position
     */
    // m_xboxController.x().whileTrue(
    //   new retractPivotCommand(m_GroundCollectionSubsystem)
    // );

    // m_xboxController.start().onTrue(Commands.runOnce(() -> m_GroundCollectionSubsystem.
    //   setPivotPosition(-0.21), m_GroundCollectionSubsystem).
    //     withDeadline(Commands.waitUntil(()->m_GroundCollectionSubsystem.isAtPosition(-0.21))));

    // m_xboxController.back().onTrue(Commands.runOnce(() -> m_GroundCollectionSubsystem.
    //   setPivotPosition(-0.0), m_GroundCollectionSubsystem).
    //     withDeadline(Commands.waitUntil(()->m_GroundCollectionSubsystem.isAtPosition(-0.0))));
    // m_xboxController.a().whileTrue(m_RunCollectorCommands);
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    /**
     * sets pattern alignedWithHumanPlayerStation when you press the start button
     */
    // m_operatorController.start().whileTrue(Commands.run(
    //         () -> LEDSubsystem.setPattern(
    //             LEDConstants.LEDPatterns.ALIGNED_WITH_HUMAN_PLAYER_STATION), LEDSubsystem));
      m_xboxController.a().whileTrue(new runCollectorCommands(m_GroundCollectionSubsystem));
      // m_xboxController.leftBumper().whileTrue(Commands.run(() -> m_GroundCollectionSubsystem.drivePivotVolts(-1.5), m_GroundCollectionSubsystem));
      m_xboxController.leftBumper().whileTrue(new collectorMoveVoltsOut(m_GroundCollectionSubsystem));
      // // m_xboxController.rightBumper().whileTrue(Commands.run(() -> m_GroundCollectionSubsystem.drivePivotVolts(1.5), m_GroundCollectionSubsystem));
      m_xboxController.rightBumper().whileTrue(new collectorMoveVoltsIn(m_GroundCollectionSubsystem));

        LEDSubsystem.setDefaultCommand(Commands.runOnce(
            () -> LEDSubsystem.setPattern(
                LEDConstants.LEDPatterns.IDLE), LEDSubsystem));//set idle as default pattern
    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
    ////m_driverController.b().whileTrue(m_exampleSubsystem.exampleMethodCommand());
    

    // Final bindings TBD: test binding toggles the SpinIndexerCMD when X is pressed.
    // m_driverController.x().toggleOnTrue(m_SpinIndexerCMD);

    // Test bindings
    // m_driverController.y().toggleOnTrue(m_AutosSpinIndexerCMD);
    //m_driverController.b().whileTrue(new KickFuelCMD(m_kicker));
    //m_kicker.setDefaultCommand(m_kickFuel);
    // m_driverController.a().whileTrue(m_expelFuel);
    // m_buttonboard.b1().whileTrue(m_kickFuel);//when you press b1, it runs the method while the button is being pressed
    // m_buttonboard.getTrigger(13).whileTrue(m_expelFuel);//when they joystick is pressed down it runs the method

    operatorController.povDown().whileTrue(Commands.runOnce(() -> 
      climbSubsystem.setClimbSetpoint(ClimbConstants.L1Position), climbSubsystem))
      .onFalse(Commands.runOnce(() -> 
      climbSubsystem.setClimbSetpoint(ClimbConstants.climbHomePosition)));

    operatorController.povLeft().whileTrue(Commands.runOnce(() -> 
      climbSubsystem.setClimbSetpoint(ClimbConstants.L1Position), climbSubsystem))
      .onFalse(Commands.runOnce(() -> 
      climbSubsystem.setClimbSetpoint(ClimbConstants.L1PositionLock), climbSubsystem));


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
      return null;
    // An example command will be run in autonomous
  }
}