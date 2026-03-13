// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kOperatorControllerPort = 0;
  }

  public static class ClimbConstants {
    public static final int leftClimbMotorID = 57;
    public static final int rightClimbMotorID = 55;
    public static final int leftClimbCANCoderID = 58;
    public static final int rightClimbCANCoderID = 56;

    //Climb Setpoints (inches)
    public static final double L1AutoStartPosition = 8; // Must be lower than position where slidehook comes out
    public static final double L1AutoEndPosition = 4; 
    public static final double deploySliderPosition = 9; // Position where slider is deployed
    public static final double L1Position = 7;
    public static final double L1PositionLock = 0;
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
