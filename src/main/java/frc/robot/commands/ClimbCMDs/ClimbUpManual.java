// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.commands.ClimbCMDs;

// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.Constants.ClimbConstants;
// import frc.robot.subsystems.Climb.ClimbSubsys;

// /* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
// public class ClimbUpManual extends Command {
//   ClimbSubsys climb;

//   /** Creates a new ClimbManual. */
//   public ClimbUpManual(ClimbSubsys climb) {
//     this.climb = climb;

//     addRequirements(climb);
//     // Use addRequirements() here to declare subsystem dependencies.
//   }

//   // Called when the command is initially scheduled.
//   @Override
//   public void initialize() {
//     //Sets climb to move up at whatever the speed constant is set as
//     climb.setClimbSpeed(ClimbConstants.climbSpeed);
//   }

//   // Called every time the scheduler runs while the command is scheduled.
//   @Override
//   public void execute() {}

//   // Called once the command ends or is interrupted.
//   @Override
//   public void end(boolean interrupted) {
//     //When the command is finished the motors are stopped
//     climb.setClimbSpeed(0);
//   }

//   // Returns true when the command should end.
//   @Override
//   public boolean isFinished() {
//     return false;
//   }
// }
