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
  }
  public static class pivotConsants{

    public static final int pivotMotorID = 1; //change according to motor id number - pivot motor ID number
    public static final int pivotEncoderID = 1; //change according to encoder id number - pivot encoder ID number

    public static final int lowerLimitSwitchPort = 0;//change according to limit switch port - pivot lower limit switch port number
    public static final int upperLimitSwitchPort = 8;//change according to limit switch port - pivot upper limit switch port number

    //PID values for pivot
    public static final double kP = 0.1;//change according to pid value
    public static final double kI = 0;//change according to pid value
    public static final double kD = 0;//change according to pid value 
    public static final double kS = 0;//change if needed
    public static final double kV = 0;//change if needed
    public static final double kA = 0;//change if needed
    public static final double kG = 0;//change if needed

    public static final double startPosition = 0;//change according to start position - pivot start position
    public static final double intakePosition = 0.5;//change according to intake position - pivot intake position

    public static final double homePosition = 0;//change according to position - desired position of the pivot
    public static final double positionTolerance = 0;//change according to position tolerance - pivot position tolerance

    public static final double softLimitMinPosition = 0;//min position that the pivot deploy can go
    public static final double softLimitMaxPosition = 0;//max position that the pivot deploy can go
  }

  public static class collectorConstants{
    public static final int collectorMotorID = 1;//change according to motor id number - collector motor ID number
  }

  public static class controllerConstants{
    public static final int controllerPort = 1;//change according to controller number
  }

}
