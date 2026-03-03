// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Vision;

import java.lang.reflect.Array;
import java.util.ArrayList;
import java.util.Optional;

import edu.wpi.first.hal.MatchInfoData;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.jni.WPIMathJNI;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.WPIUtilJNI;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class VisionSubsys extends SubsystemBase {
  Limelight[] limelights;
  ArrayList<Double> x,y,z,pitch,yaw,roll;

  CommandSwerveDrivetrain drivetrain;
  /** Creates a new Vision. */
  public VisionSubsys(CommandSwerveDrivetrain drivetrain) {
    limelights = new Limelight[]{
      new Limelight("limelight")
    };

    this.drivetrain = drivetrain;
  }


  public ArrayList<double[]> robotPose3dFieldSpace() {

    ArrayList<double[]> poses = new ArrayList<double[]>();

    for(Limelight limelight:limelights){
      if (limelight.hasTarget()) {
        poses.add(limelight.botPoseFieldSpace());
      }
    }
    return poses;
  }

  public ArrayList<double[]> robotPose2dFieldSpace() {
    ArrayList<double[]> poses = new ArrayList<double[]>();
    for(double[] pose3d: robotPose3dFieldSpace()) {
      poses.add(new double[]{pose3d[0], pose3d[1], pose3d[5], pose3d[7]});
    }
    return poses;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    for(double[] pose: robotPose2dFieldSpace()) {
      drivetrain.addVisionMeasurement(
        new Pose2d(pose[0], pose[1], Rotation2d.fromDegrees(pose[2])),
        pose[3]
        );
    }
  }

  public class Limelight extends SubsystemBase {
    NetworkTable table;
    NetworkTableEntry tx, ty, ta, robotPoseFieldSpace, targetPoseRobotSpace, tID;

    /**
     * Creates a new Limelight
     * @param name , the hosname of the limelight
     */
    public Limelight(String name) {

      table = NetworkTableInstance.getDefault().getTable(name);

      tx = table.getEntry("tx");

      ty = table.getEntry("ty");

      ta = table.getEntry("ta");

      if (Alliance.Blue == DriverStation.getAlliance().get()) {
        robotPoseFieldSpace = table.getEntry("botpose_orb_wpiblue");
      } else if (Alliance.Red == DriverStation.getAlliance().get()) {
        robotPoseFieldSpace = table.getEntry("botpose_orb_wpired");
      } else {
        robotPoseFieldSpace = table.getEntry("botpose_orb");
      }

      tID = table.getEntry("tid");

      targetPoseRobotSpace = table.getEntry("targetpose_robotspace");
    }

    /**
     * Returns wether the limelight has a target that it is tracking.
     * @return
     * True or false if it sees a target or not.
     */
    public boolean hasTarget() {
      return table.getEntry("tv").getBoolean(false);
    }

    /**
     * Returns the id of the target that is currently being tracked.<p>
     * If multiple targets are seen, returns the target specified by the limelight settings.
     * @return
     * The id of the aprilTag currently being tracked.
     */
    public int targetID() {
      return (int) tID.getInteger(0);
    }

    /**
     * Returns the x angle from the crosshair to the center of the target.<p>
     * If multiple targets are seen, returns the center specified by the limelight settings.
     * @return
     * The x angle to the center of the target.
     */
    public double targetX() {
      return tx.getDouble(0);
    }

    /**
     * Returns the y angle from the crosshair to the center of the target.<p>
     * If multiple targets are seen, returns the center specified by the limelight settings.
     * @return
     * The y angle to the center of the target.
     */
    public double targetY() {
      return ty.getDouble(0);
    }

    /**
     * Returns the total percent of the limelight's field of view that is taken up by the target currently being tracked.<p>
     * If multiple targets are seen, returns the center specified by the limelight settings.
     * @return
     * The percentage of the field of view taken up by the current target.
     */
    public double targerArea() {
      return ta.getDouble(0);
    }

    /**
     * Returns an aray containing data about the robots position on the field<p>
     * Only works when in an aprilTag pipeline.
     * @return
     * An array containing the robots position on the field, the order of the array is:<p>
     * Translation X,
     * Translation Y,
     * Translation Z,<p>
     * Rotation around X,
     * Rotation around Y,
     * Rotation around Z,<p>
     * Total latency,<p>
     * Tag count,
     * Tag span,<p>
     * Average tag distance from camera,
     * Average tag area(percentage of image)
     */
    public double[] botPoseFieldSpace() {
      return robotPoseFieldSpace.getDoubleArray(new double[0]);
    }

    /**
     * Returns the targets position in the robot's coordinate system.<p>
     * Only works when in an aprilTag pipeline.
     * @return
     * An array containing the targets position in robot space, the order of the array is:<p>
     * Translation 
     */
    public double[] targetPoseRobotSpace() {
      return targetPoseRobotSpace.getDoubleArray(new double[0]);
    }

    public double latency() {
      return targetPoseRobotSpace.getDoubleArray(new double[0])[7];
    }

  }
}
