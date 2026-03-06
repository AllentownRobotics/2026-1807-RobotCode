// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Vision;

import java.util.ArrayList;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class VisionSubsys extends SubsystemBase {
  private Limelight[] limelights;

  CommandSwerveDrivetrain drivetrain;
  /** Creates a new Vision. */
  public VisionSubsys(CommandSwerveDrivetrain drivetrain) {
    limelights = new Limelight[]{ // add all of the limelights used for april tags here
      new Limelight("limelight-test")
    };


    this.drivetrain = drivetrain;//sets the drivetrain in the subsystem to the actual drivetrain.
  }

  /**
   * Returns an Arraylist of botpose-orb-fieldspace arrays containing <p>
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
   * Average tag area(percentage of image)<p>
   * for more information, view {@link Limelight}
   */
  public ArrayList<double[]> robotPose3dFieldSpace() {

    // array of double arrays, each array is in the order of x,y,z, pitch, yaw, roll, total latency, 
    // tag count, tag span, average tag distance from camera, average tag percent of image
    ArrayList<double[]> poses = new ArrayList<double[]>(); 

    for(Limelight limelight:limelights){//loops through all limelights
      if (limelight.hasTarget()) {//if the limelight sees an april tag
        poses.add(limelight.botPoseFieldSpace());//add the array of data from that camera to the ArrayList
      }
    }

    return poses;//return the ArrayList containing the data from all cameras that see an april tag
  }

  /**
   * 
   * @return
   */
  public ArrayList<double[]> robotPose2dFieldSpace() {

    //Array of double arrays, each array is in the order of x,y, yaw, timestamp
    ArrayList<double[]> poses = new ArrayList<double[]>();

    for(double[] pose3d: robotPose3dFieldSpace()) {//loop through all arrays in the 3d pose
      //add the 2d position and timestamp to the arraylist
      poses.add(new double[]{// TODO remove magic numbers
        pose3d[0], 
        pose3d[1], 
        pose3d[4], // this should be index 5, through testing, it was found that index 4 is the yaw
        Timer.getTimestamp() - pose3d[7]/1000.0 //currwent timestamp - the total latency of the camera
      });
    }

    return poses;// return the array of 2d positions from the cameras that see april tags
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if(!robotPose2dFieldSpace().isEmpty()) { // if the array list is not empty(so if at least one of the cameras sees an AprilTag)
      for(double[] pose: robotPose2dFieldSpace()) {// loop through all positions given by all cameras that see an AprilTag
        drivetrain.addVisionMeasurement( // update the drivetrain's position on the field with each camera's value
          new Pose2d(pose[0], pose[1], Rotation2d.fromDegrees(pose[2])), // convert the x, y, and yaw values into a Pose2d
          pose[3] // use the timestamp to allow different cameras to have different latency
          );
      }

      SmartDashboard.putNumber("x", robotPose2dFieldSpace().get(0)[0]);// for testing, put each of
      SmartDashboard.putNumber("y", robotPose2dFieldSpace().get(0)[1]);// the position values to
      SmartDashboard.putNumber("yaw", robotPose2dFieldSpace().get(0)[2]);// SmartDashboard
      
    }
  }

  public class Limelight extends SubsystemBase {
    private NetworkTable table;
    private NetworkTableEntry tx, ty, ta, robotPoseFieldSpace, targetPoseRobotSpace, tID;

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
      return table.getEntry("tv").getInteger(0) == 1;
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
    public double[] targetPoseRobotSpace() {// TODO finish this javadoc comment
      return targetPoseRobotSpace.getDoubleArray(new double[0]);
    }

  }
}
