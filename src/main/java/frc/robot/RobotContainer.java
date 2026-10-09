// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;


import frc.robot.commands.SpinIndexerCMD;
import frc.robot.commands.autonShootCommand;

import frc.robot.commands.feedingCommand;
import frc.robot.commands.runCollectorForward;
import frc.robot.commands.runCollectorReverse;
import frc.robot.commands.setFlywheelVelocity;
import frc.robot.commands.shootingSequence;
import frc.robot.commands.AutoAimingCommands.AutonAutoAimHub;
import frc.robot.commands.AutoAimingCommands.TeleopAutoAimHub;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.Drive.CommandSwerveDrivetrain;
import frc.robot.subsystems.GroundCollector.GroundCollector;
import frc.robot.subsystems.IndexerSubsys.IndexerSubsys;
import frc.robot.subsystems.Kicker.KickerSubsys;
import frc.robot.subsystems.Shooter.FlywheelSubsys;
import frc.robot.subsystems.Shooter.HoodSubsys;
import frc.robot.subsystems.Shooter.TargettingSubsys;
import frc.robot.subsystems.Vision.VisionSubsys;
import edu.wpi.first.wpilibj2.command.Command;

import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
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
  //  private final CommandXboxController m_xboxController = new CommandXboxController(0);
   private KickerSubsys kickerSubsys = new KickerSubsys();

  //  private ClimbSubsys climbSubsystem = new ClimbSubsys();
  private final IndexerSubsys m_indexerSubsystem = new IndexerSubsys();
  private final CommandXboxController operatorController =
      new CommandXboxController(Constants.operatorConstants.operatorController);//creates new operator controller

  private final LEDSubsystem LEDSubsystem = new LEDSubsystem();//makes new LEDSubsystem

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
    private final TargettingSubsys turret = new TargettingSubsys(drivetrain);
    // private final Flywheel flywheel = new Flywheel();
    private final HoodSubsys hood = new HoodSubsys(drivetrain);
    private FlywheelSubsys flywheel = new FlywheelSubsys(drivetrain);
     /* Path follower */
     private final VisionSubsys visionSubsys = new VisionSubsys(drivetrain);
    private final SendableChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    //registers commands for every pattern
    NamedCommands.registerCommand("Coast Mode", new InstantCommand( () -> drivetrain.configNeutralMode(NeutralModeValue.Coast)));
    NamedCommands.registerCommand("CollectorDown", m_GroundCollectionSubsystem.runOnce(() -> m_GroundCollectionSubsystem.setPivotPosition(-68)));
    NamedCommands.registerCommand("StartCollector", new runCollectorForward(m_GroundCollectionSubsystem));
    NamedCommands.registerCommand("ShootCommand", new autonShootCommand(hood, flywheel, LEDSubsystem, kickerSubsys, m_indexerSubsystem).withTimeout(4));
    NamedCommands.registerCommand("collectorUp", m_GroundCollectionSubsystem.runOnce(() -> m_GroundCollectionSubsystem.setPivotPosition(0)));
    NamedCommands.registerCommand("Auto Hub Alignment", new AutonAutoAimHub(drivetrain, turret).withTimeout(2));
    

    autoChooser = AutoBuilder.buildAutoChooser("Tests");
        SmartDashboard.putData("Auto Mode", autoChooser);
    // SmartDashboard.putData(LEDSubsystem);//puts data into smart dashboard
        
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

    driverController.rightTrigger().whileTrue(new TeleopAutoAimHub(drivetrain, driverController, turret, operatorController));



      // ------------------Operator CONTROLS--------------------\\

    // Collector Commands
    operatorController.a().whileTrue(new runCollectorForward(m_GroundCollectionSubsystem));
    operatorController.x().whileTrue(new runCollectorReverse(m_GroundCollectionSubsystem));
    

    // Shooting/Fuel pathway commands
    operatorController.b().whileTrue(new setFlywheelVelocity(flywheel));
    operatorController.y().whileTrue(new SpinIndexerCMD(m_indexerSubsystem));
    operatorController.rightTrigger().whileTrue(new shootingSequence(hood, flywheel, turret, LEDSubsystem, kickerSubsys, m_indexerSubsystem));
    operatorController.leftTrigger().whileTrue(new feedingCommand(hood, flywheel, turret, LEDSubsystem, kickerSubsys, m_indexerSubsystem));
   
    // Increment hood angle commands 
    operatorController.start().onTrue(Commands.runOnce(() -> hood.incrementHoodAngle(0.1)));
    operatorController.back().onTrue(Commands.runOnce(() -> hood.incrementHoodAngle(-0.1)));
    
    // Ground collector Commands
    operatorController.rightBumper().whileTrue(m_GroundCollectionSubsystem.runOnce(() -> m_GroundCollectionSubsystem.setPivotPosition(-68)));
    operatorController.leftBumper().whileTrue(m_GroundCollectionSubsystem.runOnce(() -> m_GroundCollectionSubsystem.setPivotPosition(0)));


    drivetrain.registerTelemetry(logger::telemeterize);
    }
  

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    System.out.println("Autonomous command was recieved " + autoChooser.getSelected().getName());
    return autoChooser.getSelected();
  }
}