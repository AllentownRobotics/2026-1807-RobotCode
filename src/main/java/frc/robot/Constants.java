// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class IndexerConstants {
    public static final int indexerMotorID = 30; // Motor ID for indexer

    public static final double indexerVelocity = 1; // PID setpoint/velocity in rps
    public static final int indexerCurrentLimit = 35; // Indexer current limit
    // PID values (not calibrated yet):
    public static final double kp = 0;
   public static final int kDriverControllerPort  = 0;
  public static class MotorIDs {
    public static final int indexerMotorID = 30; // Motor ID for indexer
    
  }
  public static class hoodConstants {
    public static final double hoodkP = 1;
    public static final double hoodkI = 0;
    public static final double hoodkD = 0;
    public static final double hoodkS = 0;
    public static final double hoodkV = 0;
    public static final double hoodkA = 0;
    public static final double hoodkG = 0;
  }

  public static class turretConstants {
    public static final double turretkP = 0;
    public static final double turretkI = 0;
    public static final double turretkD = 0;
    public static final double turretkS = 0;
    public static final double turretkV = 0;
    public static final double turretkA = 0;
    public static final double turretkG = 0;

    // from onshape, calculated the inches in x and y and turned that into meters for pose.
    public static final Translation2d BLUE_HUB =
        new Translation2d(
            Units.inchesToMeters(
                158.250954 + 47.998092 / 2), // X - distance from blue alliance wall
            Units.inchesToMeters(161.517500) // Y - centered on field width // 158.84
            );
    // from onshape, calculated the inches in x and y and turned that into meters for pose.
    public static final Translation2d RED_HUB =
        new Translation2d(
            Units.inchesToMeters(
                445.250954
                    + 47.998092
                        / 2), // X - mirrored for red side // 158.84          651.22 - 157.84
            Units.inchesToMeters(161.517500) // Y - same center
            );
  }

  public static class AimingConstants{
      public static final double driveHubAutoAimkP = 5;  // fine tune more
      public static final double driveHubAutoAimkI = 1; // fine tune more 
      public static final double driveHubAutoAimkD = 0;   // fine tune more
      public static final double headingTargettingTolerance = 5;
     //------------------BLUE ALLIANCE--------------------
      public static final double blueAllianceTrench = 4;
      public static final double middleLine = 3.975;
      public static final double blueRightFeedingTargetX = 2.186;
      public static final double blueRightFeedingTargetY = 1.690;
      public static final double blueLeftFeedingTargetX = 2.186;
      public static final double blueLeftFeedingTargetY = 6.065;

      // ---------------- RED ALLIANCE--------------------
      public static final double redAllianceTrench = 12;
      public static final double redRightFeedingTargetX = 14.339;
      public static final double redRightFeedingTargetY = 6.065;
      public static final double redLeftFeedingTargetX = 14.339;
      public static final double redLeftFeedingTargetY = 1.690;
    
  }
  public static class IndexerConstants {
    public static final double desiredIndexerSpeed = 0.0; // PID setpoint (aka desired motor speed)
    // PID values (not calibrated yet):
    public static final double kp = 0.0;
    public static final double ki = 0;
    public static final double kd = 0;
    public static final double kv = 0.01;
    public static final double ka = 0.01;
    public static final double kg = 0;

    // Beam break IDs
    public static final int topBeamBreakID = 40; 
    public static final int bottomBeamBreakID = 41;
  }

  public static class KickerConstansts {
    public static final int topKickerMotorID = 30;
    public static final double topKickerMotorSpeedKick = -0.8;
    public static final double topKickerMotorSpeedExpel = 0.8;
    public static final int  bottomKickerMotorID = 33;
    public static final double bottomKickerSpeed = -0.8;
  }

  public static class SensorIDs {
    public static final int sensorID = 0;
  }

  public static class TimeConstants {
    public static final double kickDurationAsSec = 3;
  }
  public static class operatorConstants {
    public static final int operatorController = 1;
  }
  public static class DriverConstants {
    public static final int driverController = 1;
  }

  public static class LEDConstants{
    public static final int blinkinID = 12;
    /**
     *sets LED patterns
     */
    public static enum LEDPatterns{
      OFF(off),
      IDLE(idle),
      ALIGNED_WITH_HUB(alignedWithHub),
      CLIMB_COMPLETE(off),
      //CLIMB_COMPLETE_RED(climbCompleteRed),
      //CLIMB_COMPLETE_BLUE(climbCompleteBlue),
      PIVOT_AT_INTAKE_POSITION(pivotAtIntakePosition),
      //PIVOT_AT_HOME_POSITION(pivotAtHomePosition),
      FUEL_READY_TO_SHOOT(fuelReadyToShoot),
      //FUEL_IN_TWINDEXER(fuelInTwindexer),
      ALIGNED_WITH_HUMAN_PLAYER_STATION(alignedWithHumanPlayerStation),
      PANIC(panic);

      public final double value;
      private LEDPatterns(double val){
        value = val;
      }
    }
    //color codes
    public static final double off = 0.99;//black
    public static final double idle = 0.61;// solid red
    public static final double alignedWithHub = 0.77;//solid green
    //public static final double climbCompleteRed = -0.85;//shot, red
    //public static final double climbCompleteBlue = -0.83;//shot, blue
    public static final double climbComplete = 0.65;//solid orange
    public static final double pivotAtIntakePosition = 0.57;//solid hot pink
    //public static final double pivotAtHomePosition = 0;//change after color is confirmed
    public static final double fuelReadyToShoot = 0.15;//strobe green(color 1)
    //public static final double fuelInTwindexer = 0;//change after color is confirmed
    public static final double alignedWithHumanPlayerStation = 0;//purple
    public static final double panic = -0.11;//strobe red
  }

 public static class pivotConsants{

    public static final int pivotMotorID = 16; //change according to motor id number - pivot motor ID number
    public static final int pivotEncoderID = 17; //change according to encoder id number - pivot encoder ID number

    public static final int intakeLimitSwitchPort = 0;//change according to limit switch port - pivot lower limit switch port number
    public static final int homeLimitSwitchPort = 8;//change according to limit switch port - pivot upper limit switch port number

    //PID values for pivot
    public static final double kP = 1.1211;//change according to pid value
    public static final double kI = 0;//change according to pid value
    public static final double kD = 0;//change according to pid value 
    public static final double kS = 0.45145;//change if needed
    public static final double kV = 0.11565;//change if needed
    public static final double kA = 2.9024;//change if needed
    public static final double kG = 0;//change if needed // 1.9482

    public static final double pivotOutPosition = -0.21;//change according to intake position - pivot intake position

    public static final double pivotInPosition = 0;//change according to position - start position of the pivot
    public static final double positionTolerance = 0;//change according to position tolerance - pivot position tolerance

    public static final double softLimitMinPosition = 0;//min position that the pivot deploy can go
    public static final double softLimitMaxPosition = 0;//max position that the pivot deploy can go
    public static final double currentLimit = 35;
  }

  public static class collectorConstants{
    public static final int collectorMotorID = 18;//change according to motor id number - collector motor ID number
    public static final double collectorP = 0.1;
    public static final double collectorI = 0;
    public static final double collectorD = 0;    
  }

  public static class controllerConstants{
    public static final int controllerPort = 1;//change according to controller number - controller ID number
  }
  public static class ClimbConstants {
    public static final int leftClimbMotorID = 57;
    public static final int rightClimbMotorID = 55;
    public static final int leftClimbCANCoderID = 58;
    public static final int rightClimbCANCoderID = 56;

    //Climb Setpoints (inches)
    public static final double L1AutoStartPosition = 4; // Must be lower than position where slidehook comes out
    public static final double L1AutoEndPosition = 2; 
    public static final double deploySliderPosition = 4; // Position where slider is deployed
    public static final double L1Position = 8.9;
    public static final double L1PositionLock = 2;
    public static final double L2Position = 7;
    public static final double L2PositionLock = 0;
    public static final double L3Position = 7;
    public static final double L3PositionLock = 0;
    public static final double positionTolerance = 0.05;
    public static final double softLimitMinPosition = 0;
    public static final double softLimitMaxPosition = 0;
    public static final double incrementMeasurement = 0;
    public static final double climbHomePosition = 0;

    // Climb Motor PID
    public static final double CLIMB_P = 35;
    public static final double CLIMB_I = 2;
    public static final double CLIMB_D = 0.25;
    public static final double CLIMB_SFF = 0; // static feedforward
    public static final double CLIMB_VFF = 0; // velocity feedforward
    public static final double CLIMB_AFF = 0; // acceleration feedforward
    public static final double CLIMB_GFF = 0; // gravity feedforward 0.296
    public static final double CLIMB_MIN_OUTPUT = -1;
    public static final double CLIMB_MAX_OUTPUT = 1;

    public static final double climbSpeed = 0.25;


    public static final double climbGearing = -9*9; 
    public static final double climbSprocketRadius = 2.16/2; //Pitch Diameter: 2.16", Outside Diameter: 2.36", Sprocket Type: AM-4791
    public static final double climbSprocketCircumference = 2 * Math.PI * climbSprocketRadius; // inches
    public static final double climbEncoderToMechanismRatio = 1; //Encoder mounted directly to sprocket shaft

  }
}