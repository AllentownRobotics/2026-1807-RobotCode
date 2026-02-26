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
  public static class VisionConstants {
   
    public static final double targetingClimbTranslationOffset = 0;
    public static final double targetingClimbTopBottomTranslationOffset = 0;
    public static final double targetingTrenchTranslationOffset = 0;
    public static final double targetingTrenchTopBottomTranslationOffset = 0;
    
    public static final double rotation_kP = 0; 
    public static final double rotation_kI = 0;
    public static final double rotation_kD = 0; 

    public static final double translation_kP = 0;
    public static final double translation_kI = 0; 
    public static final double translation_kD = 0; 

    public static final double ytranslation_kP = 0;
    public static final double ytranslation_kI = 0; 
    public static final double ytranslation_kD = 0; 
    
    public static final double rotationTargetingSpeed = 0;

    public static final double translationTargetingSpeed = 0;

    public static final double xDistanceDeadzone = 0;
    public static final double yLDistanceDeadzone = 0;
    public static final double angleDeadzone = 0;


    // limelight configs (center of lens) relative to center bottom of the robot (bottom of the wheels)
    // camera view: robot pose in target space

    // forward, right, and up are in meters
    // roll, pitch, and yaw are in degrees

    public static final double hopperLLForward = 0;
    public static final double hopperLLRight =  0;
    public static final double hopperLLUp = 0;
    public static final double hopperLLRoll = 0;
    public static final double hopperLLPitch = 0;
    public static final double hopperLLYaw = 0;

    public static final double frontLLForward = 0; 
    public static final double frontLLRight = 0; 
    public static final double frontLLUp = 0;
    public static final double frontLLRoll = 0;
    public static final double frontLLPitch = 0;
    public static final double frontLLYaw = 0;

    public static final double backLLForward = 0;
    public static final double backLLRight = 0;
    public static final double backLLUp = 0;
    public static final double backLLRoll = 0;
    public static final double backLLPitch = 0;
    public static final double backLLYaw = 0;

  }
}
