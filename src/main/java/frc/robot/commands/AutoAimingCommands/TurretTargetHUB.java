// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.commands.AutoAimingCommands;

// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.subsystems.Shooter.TurretSubsys;

// /* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
// public class TurretTargetHUB extends Command {

//   private TurretSubsys turret;

//   /** Creates a new targetHUB. */
//   public TurretTargetHUB(TurretSubsys turret) {
//     this.turret = turret;
//     addRequirements(turret);
//     // Use addRequirements() here to declare subsystem dependencies.
//   }

//   // Called when the command is initially scheduled.
//   @Override
//   public void initialize() {}

//   // Called every time the scheduler runs while the command is scheduled.
//   @Override
//   public void execute() {
//     turret.trackHUB(); // continuously makes our turret track the hub at 20 times / sec
//   }

//   // Called once the command ends or is interrupted.
//   @Override
//   public void end(boolean interrupted) {
    
//   }

//   // Returns true when the command should end.
//   @Override
//   public boolean isFinished() {
//     return turret.isTurretWithinTolerance(); // returns true when turret is at correct position
//   }
// }
