// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.utils;

// import edu.wpi.first.util.sendable.Sendable;
// import edu.wpi.first.util.sendable.SendableRegistry;
// import edu.wpi.first.wpilibj.PowerDistribution;
// import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;

// public class PowerMonitor extends SubsystemBase {
  
//   private final PowerDistribution pdh;
//   private double totalAmps;
//   /** Creates a new PDH. */
//   public PowerMonitor() {
//     this.pdh = new PowerDistribution(60, ModuleType.kRev);
//   }

//   public void logAllPorts() {
//     totalAmps = 0;
//     for(double amps: pdh.getAllCurrents()){
//       totalAmps += amps;
//     }
//     SmartDashboard.putData(pdh);
//     //SmartDashboard.putNumberArray("pdh ports current values", pdh.getAllCurrents());
//     SmartDashboard.putNumber("total current", totalAmps);

//     SmartDashboard.putNumber("Total Drivetrain Current", getTotalCurrentFromPorts(0, 1, 8, 9, 10, 11, 18, 19));

//     SmartDashboard.putNumber("Total Collector Current", getTotalCurrentFromPorts(17, 2));

//     SmartDashboard.putNumber("Total Climb Current", getTotalCurrentFromPorts(6, 7));

//     SmartDashboard.putNumber("Total Indexer Current", getTotalCurrentFromPorts());

//     SmartDashboard.putNumber("Total Shooter Current", getTotalCurrentFromPorts(13, 14, 15));

//     SmartDashboard.putNumber("Total Sensors Current", getTotalCurrentFromPorts(3, 4, 12));
//   }

//   public double getTotalCurrentFromPorts(int... portIds){
//     double current = 0;

//     for(int port:portIds){
//       current += pdh.getCurrent(port);
//     }

//     return current;
//   }

//   @Override
//   public void periodic() {
//     // This method will be called once per scheduler run
//     logAllPorts();
//   }
// }
