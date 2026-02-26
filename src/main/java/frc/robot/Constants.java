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
    public static final int kDriverControllerPort  = 0;
  }

  public static class blinkinConstants{
    public static final int blinkinID = 12;
    /**
     *sets LED patterns
     */
    public enum LEDPatterns{
      OFF(off),
      IDLE(defaultColor),
      ALIGNED_WITH_HUB(alignedWithHub),
      CLIMB_COMPLETE_RED(climbCompleteRed),
      CLIMB_COMPLETE_BLUE(climbCompleteBlue),
      PIVOT_AT_INTAKE_POSITION(pivotAtIntakePosition),
      PIVOT_AT_HOME_POSITION(pivotAtHomePosition),
      FUEL_READY_TO_SHOOT(fuelReadyToShoot),
      FUEL_IN_TWINDEXER(fuelInTwindexer),
      ALIGNED_WITH_HUMAN_PLAYER_STATION(alignedWithHumanPlayerStation);

      public final double value;
      private LEDPatterns(double val){
        value = val;
      }
    }

    public static final double off = -0.89;//change after color is confirmed
    public static final double defaultColor = 0;//change after color is confirmed
    public static final double alignedWithHub = 0;//change after color is confirmed
    public static final double climbCompleteRed = 0;//change after color is confirmed
    public static final double climbCompleteBlue = 0;//change after color is confirmed
    public static final double pivotAtIntakePosition = 0;//change after color is confirmed
    public static final double pivotAtHomePosition = 0;//change after color is confirmed
    public static final double fuelReadyToShoot = 0;//change after color is confirmed
    public static final double fuelInTwindexer = 0;//change after color is confirmed
    public static final double alignedWithHumanPlayerStation = 0;//change after color is confirmed

  }
}