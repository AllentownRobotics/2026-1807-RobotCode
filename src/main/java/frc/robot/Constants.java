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
  public static class MotorIDs {
    public static final int twindexerMotor = 2;
  }
  public static class TwindexerConstants {
    public static final double desiredTwindexerSpeed = 1; //PID setpoint aka desired motor speed
    //PID values:
    public static final double kp = 0.1;
    public static final double ki = 0;
    public static final double kd = 0;

    public static final int fullBeambreakID = 0; 
  }
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }
}
