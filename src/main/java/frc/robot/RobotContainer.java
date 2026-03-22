// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;
import frc.robot.Constants;
import frc.robot.commands.KickFuelCMD;
import frc.robot.commands.SpinIndexerCMD;
import frc.robot.commands.collectorMoveVoltsIn;
import frc.robot.commands.collectorMoveVoltsOut;
// import frc.robot.commands.retractPivotCommand;
// import frc.robot.commands.runCollectorCommands;
// import frc.robot.commands.stop;
// import frc.robot.commands.collectorVoltage;
// import frc.robot.commands.extendPivotCommand;
import frc.robot.commands.runCollectorForward;
import frc.robot.commands.runCollectorReverse;
import frc.robot.commands.AutoAimingCommands.TeleopAutoAimHub;
import frc.robot.commands.AutoAimingCommands.TeleopAutoSetHoodAngle;
import frc.robot.commands.ClimbCMDs.ClimbDownManual;
import frc.robot.commands.ClimbCMDs.ClimbUpManual;
import frc.robot.commands.setFlywheelVelocity;
import frc.robot.commands.shootingSequence;
import frc.robot.generated.TunerConstants;
import frc.robot.Constants.ClimbConstants;
// import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.LEDConstants;
// import frc.robot.Constants.operatorConstants;
// import frc.robot.commands.ExpelFuelCMD;
// import frc.robot.commands.AutosSpinIndexerCMD;
// import frc.robot.commands.KickFuelCMD;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.Climb.ClimbSubsys;
import frc.robot.subsystems.Drive.CommandSwerveDrivetrain;
// import frc.robot.commands.SpinIndexerCMD;
// import frc.robot.subsystems.Kicker.KickerSubsys;
import frc.utils.ButtonBoard;
import frc.robot.subsystems.GroundCollector.GroundCollector;
import frc.robot.subsystems.IndexerSubsys.IndexerSubsys;
import frc.robot.subsystems.Kicker.KickerSubsys;
import frc.robot.subsystems.Shooter.FlywheelSubsys;
import frc.robot.subsystems.Shooter.HoodSubsys;
import frc.robot.subsystems.Shooter.TurretSubsys;
// import frc.robot.subsystems.GroundCollector.GroundCollector;
// import frc.robot.subsystems.GroundCollector.newCollector;
// import frc.robot.subsystems.IndexerSubsys.IndexerSubsys;
import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.commands.runCollectorReverse;
import edu.wpi.first.wpilibj2.command.RunCommand;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Telemetry;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
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
  //  private final CommandXboxController m_xboxController = new CommandXboxController(0);
   private KickerSubsys kickerSubsys = new KickerSubsys();
   private FlywheelSubsys flywheel = new FlywheelSubsys();
   private ClimbSubsys climbSubsystem = new ClimbSubsys();
  // The robot's subsystems and commands are defined here...
  // private final KickerSubsys m_kicker = new KickerSubsys();
  private final IndexerSubsys m_indexerSubsystem = new IndexerSubsys();

  // Replace with CommandPS4Controller or CommanddriverController if needed
  // private final CommandXboxController m_driverController =
  //     new CommandXboxController(Constants.kDriverControllerPort);
  // private final SpinIndexerCMD m_SpinIndexerCMD = new SpinIndexerCMD(m_indexerSubsystem);
  // private final AutosSpinIndexerCMD m_AutosSpinIndexerCMD = new AutosSpinIndexerCMD(m_indexerSubsystem);

  // private final runCollectorCommands m_RunCollectorCommands = new runCollectorCommands(m_GroundCollectionSubsystem);
  // Replace with CommandPS4Controller or CommanddriverController if needed
  private final CommandXboxController operatorController =
      new CommandXboxController(Constants.operatorConstants.operatorController);//creates new operator controller

  private final LEDSubsystem LEDSubsystem = new LEDSubsystem();//makes new LEDSubsystem
  // private final ExpelFuelCMD m_expelFuel = new ExpelFuelCMD(m_kicker);
  // private final KickFuelCMD m_kickFuel = new KickFuelCMD(m_kicker);

  final ButtonBoard m_buttonboard = new ButtonBoard(0);
  private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate =  RotationsPerSecond.of(1).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity
    private double slowSpeed = 0.5 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    private double slowAngularRate = RotationsPerSecond.of(0.5).in(RadiansPerSecond);
    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController driverController = new CommandXboxController(0);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    private final TurretSubsys turret = new TurretSubsys(drivetrain);
    // private final Flywheel flywheel = new Flywheel();
    private final HoodSubsys hood = new HoodSubsys(drivetrain);
     /* Path follower */
    private final SendableChooser<Command> autoChooser;

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
    // NamedCommands.registerCommand("LEDPatternPivotAtHomePosition", new InstantCommand(()-> LEDSubsystem.setPattern(Constants.LEDConstants.LEDPatterns.PIVOT_AT_HOME_POSITION), LEDSubsystem));
    NamedCommands.registerCommand("LEDPatternFuelReadyToShoot", new InstantCommand(()-> LEDSubsystem.setPattern(Constants.LEDConstants.LEDPatterns.FUEL_READY_TO_SHOOT), LEDSubsystem));
    // NamedCommands.registerCommand("LEDPatternFuelInTwindexer", new InstantCommand(()-> LEDSubsystem.setPattern(Constants.LEDConstants.LEDPatterns.FUEL_IN_TWINDEXER), LEDSubsystem));
    NamedCommands.registerCommand("LEDPatternAlignedWithHumanPlayerStation", new InstantCommand(()-> LEDSubsystem.setPattern(Constants.LEDConstants.LEDPatterns.ALIGNED_WITH_HUMAN_PLAYER_STATION), LEDSubsystem));

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
    

    autoChooser = AutoBuilder.buildAutoChooser("Tests");
        SmartDashboard.putData("Auto Mode", autoChooser);
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
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommanddriverController Flight
   * driverControllers}.
   */
  //Trigger value = m_buttonboard.getButton(0);
  private void configureBindings() {



    // ------------------DRIVER CONTROLS--------------------\\


    // Regular Swerve Drive 
    // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-driverController.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-driverController.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-driverController.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

      // Slow Drive

        driverController.leftBumper().whileTrue(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-driverController.getLeftY() * slowSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-driverController.getLeftX() * slowSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-driverController.getRightX() * slowAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

      // Reset Gyro

      driverController.back().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

      // idle drive 

      // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );


        // X-LOCK

    driverController.x().whileTrue(drivetrain.applyRequest(() -> brake));


        // Target HUB

    driverController.rightTrigger().whileTrue(new TeleopAutoAimHub(drivetrain, driverController, turret));






      // ------------------Operator CONTROLS--------------------\\

    // Ground Collector Out
    operatorController.leftBumper().whileTrue(new collectorMoveVoltsOut(m_GroundCollectionSubsystem));

    // Ground Collector In
    operatorController.rightBumper().whileTrue(new collectorMoveVoltsIn(m_GroundCollectionSubsystem));

    // Run Indexer command
    operatorController.leftTrigger().whileTrue(new SpinIndexerCMD(m_indexerSubsystem)); // Run Indexer

    // shoot fuel
    operatorController.rightTrigger().whileTrue(new shootingSequence(hood, flywheel, turret, LEDSubsystem, kickerSubsys, m_indexerSubsystem));

    // Set hood to home position. This should be done automatically but is here for manual override
    operatorController.y().onTrue(Commands.runOnce(() -> hood.setHoodToHome()));
    
    // Run Collector forward
    operatorController.a().whileTrue(new runCollectorForward(m_GroundCollectionSubsystem));

    // Run Collector Backward
    operatorController.x().whileTrue(new runCollectorReverse(m_GroundCollectionSubsystem));


    // Climb Up Manual
    operatorController.povRight().whileTrue(new ClimbUpManual(climbSubsystem));

    // Climb Down Manual
    operatorController.povDown().whileTrue(new ClimbDownManual(climbSubsystem));

    //Reset Climb Encoder(s)
    operatorController.povUp().onTrue(Commands.runOnce(() -> 
      climbSubsystem.resetEncoderPos(), climbSubsystem)); 

    



         




      

















































































    // m_xboxController.leftBumper().onTrue(Commands.runOnce(SignalLogger::start));
    // m_xboxController.rightBumper().onTrue(Commands.runOnce(SignalLogger::stop));

    // m_xboxController.start().and(m_xboxController.a()).whileTrue(m_GroundCollectionSubsystem.sysIdQuasistatic(Direction.kForward));
    // m_xboxController.start().and(m_xboxController.b()).whileTrue(m_GroundCollectionSubsystem.sysIdQuasistatic(Direction.kReverse));
    //  m_xboxController.back().and(m_xboxController.a()).whileTrue(m_GroundCollectionSubsystem.sysIdDynamic(Direction.kForward));
    // m_xboxController.back().and(m_xboxController.b()).whileTrue(m_GroundCollectionSubsystem.sysIdDynamic(Direction.kReverse));
    

      operatorController.povDown().whileTrue(Commands.runOnce(() -> 
      climbSubsystem.setClimbSetpoint(ClimbConstants.L1Position), climbSubsystem))
      .onFalse(Commands.runOnce(() -> 
      climbSubsystem.setClimbSetpoint(ClimbConstants.climbHomePosition)));

    operatorController.povLeft().whileTrue(Commands.runOnce(() -> 
      climbSubsystem.setClimbSetpoint(ClimbConstants.L1Position), climbSubsystem))
      .onFalse(Commands.runOnce(() -> 
      climbSubsystem.setClimbSetpoint(ClimbConstants.L1PositionLock), climbSubsystem));


    

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
      operatorController.a().whileTrue(new runCollectorForward(m_GroundCollectionSubsystem));
      // m_xboxController.leftBumper().whileTrue(Commands.run(() -> m_GroundCollectionSubsystem.drivePivotVolts(-1.5), m_GroundCollectionSubsystem));
      operatorController.leftBumper().whileTrue(new collectorMoveVoltsOut(m_GroundCollectionSubsystem));
      // m_xboxController.rightBumper().whileTrue(Commands.run(() -> m_GroundCollectionSubsystem.drivePivotVolts(1.5), m_GroundCollectionSubsystem));
      operatorController.rightBumper().whileTrue(new collectorMoveVoltsIn(m_GroundCollectionSubsystem));

        // LEDSubsystem.setDefaultCommand(Commands.runOnce(
        //     () -> LEDSubsystem.setPattern(
        //         LEDConstants.LEDPatterns.IDLE), LEDSubsystem));//set idle as default pattern
    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
    ////m_driverController.b().whileTrue(m_exampleSubsystem.exampleMethodCommand());
    

    // Final bindings TBD: test binding toggles the SpinIndexerCMD when X is pressed.
    // m_driverController.x().toggleOnTrue(m_SpinIndexerCMD);

    // Currently a test binding - this command will be used for autos.
    // m_driverController.y().toggleOnTrue(m_AutosSpinIndexerCMD);
    //m_driverController.b().whileTrue(new KickFuelCMD(m_kicker));
    // kickerSubsys.(new KickFuelCMD(new KickerSubsys()));
    operatorController.y().whileTrue(new KickFuelCMD(kickerSubsys));
    // m_driverController.a().whileTrue(m_expelFuel);
    // m_buttonboard.b1().whileTrue(m_kickFuel);//when you press b1, it runs the method while the button is being pressed
    // m_buttonboard.getTrigger(13).whileTrue(m_expelFuel);//when they driverController is pressed down it runs the method

        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-driverController.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-driverController.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-driverController.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );


        // driverController.rightBumper().whileTrue(drivetrain.applyRequest(() ->
        //         drive.withVelocityX(-driverController.getLeftY() * slowSpeed) // Drive forward with negative Y (forward)
        //             .withVelocityY(-driverController.getLeftX() * slowSpeed) // Drive left with negative X (left)
        //             .withRotationalRate(-driverController.getRightX() * MaxAngularRate)) // Drive counterclockwise with negative X (left)
        // );

        driverController.x().whileTrue(drivetrain.applyRequest(() -> brake));
        // driverController.b().whileTrue(drivetrain.applyRequest(() ->
        //     point.withModuleDirection(new Rotation2d(-driverController.getLeftY(), -driverController.getLeftX()))
        // ));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        // driverController.back().and(driverController.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        // driverController.back().and(driverController.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        // driverController.start().and(driverController.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        // driverController.start().and(driverController.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // Reset the field-centric heading on left bumper press.
        driverController.leftTrigger().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));


        driverController.rightTrigger().whileTrue(new TeleopAutoAimHub(drivetrain, driverController, turret)); 
        // driverController.povUp().whileTrue(new SmartDashValues(turret, drivetrain));
        // turret.setDefaultCommand(new targetHUB(turret));
        // hood.setDefaultCommand(new autoSetHoodAngle(hood));
        operatorController.rightTrigger().whileTrue(new setFlywheelVelocity(flywheel));
        // operatorController.x().whileTrue(new runCollectorReverse(m_GroundCollectionSubsystem));
        // driverController.povRight().whileTrue(new manualSetHoodAngle(hood));
        // driverController.povUp().whileTrue(new AutonAutoAimHub(drivetrain, turret));
        // driverController.povUp().whileTrue(new example(drivetrain));

        drivetrain.registerTelemetry(logger::telemeterize);
 

    }
  

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
    // An example command will be run in autonomous
  }
}