// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.subsystems.Shooter;

// import com.ctre.phoenix6.hardware.CANcoder;
// import edu.wpi.first.math.MathUtil;
// import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
// import edu.wpi.first.math.interpolation.Interpolator;
// import edu.wpi.first.math.interpolation.InverseInterpolator;
// import edu.wpi.first.wpilibj.DriverStation;
// import edu.wpi.first.wpilibj.DriverStation.Alliance;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.Constants;
// import frc.robot.subsystems.Drive.CommandSwerveDrivetrain;
// import frc.robot.utils.Kraken;
// import java.util.Optional;

// public class Hood extends SubsystemBase {
//   /** Creates a new Hood. */
//   private Kraken hoodMotor;

//   private CommandSwerveDrivetrain drive;
//   private double zeroHoodState;
//   private CANcoder hoodEncoder;
//   private double currentHoodState;
//   private double targetHoodState;
//   private double distanceToHub;
//   private double autoTargetHoodState;
//   private double hubX;
//   private double hubY;
//   private double hoodTolerance;

//   InterpolatingTreeMap<Double, Double> hoodMap;
//   // private double hoodPositionError;
//   // private double hoodPositionTolerance; // put in constants
//   // private double targetY;
//   // private double targetX;

//   public Hood(CommandSwerveDrivetrain drive) {
//     this.drive = drive;
//     hoodTolerance = 0.1; // degrees
//     hoodMap =
//         new InterpolatingTreeMap(
//             InverseInterpolator.forDouble(),
//             Interpolator.forDouble()); // makes a new interpolating table, use case is for degree
//     // calculation

//     // hoodMotor = new Kraken(101); // make constants for this
//     // hoodEncoder = new CANcoder(201); // make constants for this
//     // hoodMotor.addEncoder(hoodEncoder);

//     // hoodMotor.setBrakeMode(); // sets break mode when not in use

//     // hoodMotor.setRotorToSensorRatio(1);
//     // hoodMotor.setSensorToMechanismRatio(30); // needs to be changed, gear ratio of the mechanism
//     // turretMotor.setMotorCurrentLimits(40);     MAKE SURE TO SET THIS BEFORE TESTING
//     // PID gains for Hood, test different number to get accurately get hood to specified angle
//     // hoodMotor.setPIDValues(
//     //     Constants.hoodConstants.hoodkP,
//     //     Constants.hoodConstants.hoodkI,
//     //     Constants.hoodConstants.hoodkD,
//     //     Constants.hoodConstants.hoodkS,
//     //     Constants.hoodConstants.hoodkV,
//     //     Constants.hoodConstants.hoodkA,
//     //     Constants.hoodConstants.hoodkG);

//     SmartDashboard.putNumber("Hood target", 0);

//     // arbitrary numbers for testing change once testing
//     // "Key" in our case represents distance, value is degrees of rotation
//     hoodMap.put(1.4, 30.0);
//     hoodMap.put(2.0, 40.0);
//     hoodMap.put(2.5, 45.0);
//     hoodMap.put(1.6, 32.0);
//     hoodMap.put(2.3, 44.0);
//     hoodMap.put(2.6, 46.0);
//   }

//   /**
//    * automatically calculates which alliance you are and using that side hub calculates distance to
//    * it allowing the use of a interpolating table to automatically calculate what degree of rotation
//    * the hub must be at to make a shot.
//    */
//   public void setHoodAutomaticallyFromDistance() {

//     // calculates the hub constant
//     Optional<Alliance> ally = DriverStation.getAlliance();
//     if (ally.isPresent()) {
//       if (ally.get() == Alliance.Red) {
//         hubX = Constants.turretConstants.RED_HUB.getX();
//         hubY = Constants.turretConstants.RED_HUB.getY();
//       }
//       if (ally.get() == Alliance.Blue) {
//         hubX = Constants.turretConstants.BLUE_HUB.getX();
//         hubY = Constants.turretConstants.BLUE_HUB.getY();
//       }
//     }

//     // distance formula using the hub as x2 and current drive pose as x1
//     distanceToHub =
//         Math.sqrt(
//             Math.pow(hubX - drive.getState().Pose.getX(), 2)
//                 + Math.pow(hubY -  drive.getState().Pose.getY(), 2));
//     autoTargetHoodState =
//         hoodMap.get(
//             distanceToHub); // using distanceToHub, gets the value using that "key" from the hub
//     // table
//     autoTargetHoodState =
//         MathUtil.clamp(
//             autoTargetHoodState,
//             0,
//             90); // clamps between 0 - 90 so if it ever breaks it will never go
//     // below 0 degrees or above 90 degrees.
//     hoodMotor.setDesiredEncoderPosition(
//         autoTargetHoodState / 360); // applies that position in rotations

//     // various smartDashboard numbers to test user wanted values
//     SmartDashboard.putNumber("Distance in meters to da HUB", distanceToHub);
//     SmartDashboard.putNumber("Autonomous Target Hood State", autoTargetHoodState);
//   }

//   public void manualSetHoodAngle() {
//     targetHoodState = SmartDashboard.getNumber("Hood target", 0);
//     currentHoodState = hoodEncoder.getAbsolutePosition().getValueAsDouble() * 360;
//     targetHoodState = MathUtil.clamp(targetHoodState, 0, 45);
//     hoodMotor.setDesiredEncoderPosition(targetHoodState / 360);
//     SmartDashboard.putNumber("Where hood is tryna go", targetHoodState);
//     SmartDashboard.putNumber("where da hood at", currentHoodState);
//   }
//   /**
//    * Checks if hood is in a tolerable range of where it needs to be.
//    *
//    * @return boolean - true or false depending on if its there or not.
//    */
//   public boolean isHoodWithinTolerance() {
//     if (Math.abs(targetHoodState - (hoodEncoder.getAbsolutePosition().getValueAsDouble() * 360))
//         <= hoodTolerance) {
//       return true;
//     } else {
//       return false;
//     }
//   }

//   /** Sends hood to the home state. */
//   public void setHoodToHome() {
//     zeroHoodState = 0;
//     currentHoodState = hoodEncoder.getAbsolutePosition().getValueAsDouble() * 360;
//     // targetHoodState = MathUtil.clamp(zeroHoodState, 0, 45);
//     hoodMotor.setDesiredEncoderPosition(zeroHoodState);
//     SmartDashboard.putNumber("Is hood trying to go to 0?", targetHoodState);
//     SmartDashboard.putNumber("where da hood at", currentHoodState);
//   }

//   @Override
//   public void periodic() {
//     // This method will be called once per scheduler run

//     // SmartDashboard.putBoolean("Hood at target?", isHoodWithinTolerance());
//   }
// }
