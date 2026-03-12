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
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
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
    public static final double turretkP = 1;
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
}
