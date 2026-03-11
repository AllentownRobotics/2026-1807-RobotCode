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
    public static final int leftClimbMotorID = 100;
    public static final int rightClimbMotorID = 90;
    public static final int leftClimbCANCoderID = 101;
    public static final int rightClimbCANCoderID = 91;

    public static final double L1Position = 0;
    public static final double L1PositionLock = 0;
    public static final double L2Position = 0;
    public static final double L2PositionLock = 0;
    public static final double L3Position = 0;
    public static final double L3PositionLock = 0;
    public static final double positionTolerance = 0;
    public static final double softLimitMinPosition = 0;
    public static final double softLimitMaxPosition = 0;
    public static final double incrementMeasurement = 0;
    public static final double climbHomePosition = 0;

    // Climb Motor PID
    public static final double CLIMB_P = 0;
    public static final double CLIMB_I = 0;
    public static final double CLIMB_D = 0;
    public static final double CLIMB_SFF = 0; // static feedforward
    public static final double CLIMB_VFF = 0; // velocity feedforward
    public static final double CLIMB_AFF = 0; // acceleration feedforward
    public static final double CLIMB_GFF = 0; // gravity feedforward 0.296
    public static final double CLIMB_MIN_OUTPUT = -1;
    public static final double CLIMB_MAX_OUTPUT = 1;

    public static final double climbSpeed = 0;

    public static final double climbGearing = 0; // inches
    public static final double climbSprocketRadius = 0;
    public static final double climbSprocketCircumference = 0; // inches
    public static final double climbEncoderToMechanismRatio = 0;

  }
}
