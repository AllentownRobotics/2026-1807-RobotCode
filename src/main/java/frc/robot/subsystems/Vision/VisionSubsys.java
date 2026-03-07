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

  CommandSwerveDrivetrain drivetrain;// this is irregilar to use a different subsystem inside of a subsystem, 
  // this is because the vision subsystem needs to update the drivetrain's position with the vision measurements,
  // so it needs to have access to the drivetrain. This is not a common practice but it is the most efficient way to 
  // update the drivetrain's position with vision measurements.

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
  public ArrayList<double[]> getRobotPose3dFieldSpaceEstimates() {

    // array of double arrays, each array is in the order of x,y,z, pitch, yaw, roll, total latency, 
    // tag count, tag span, average tag distance from camera, average tag percent of image
    ArrayList<double[]> poses = new ArrayList<double[]>(); 

    for(Limelight limelight : limelights){//loops through all limelights
      if (limelight.hasTarget()) {//if the limelight sees an april tag
        poses.add(limelight.getRobotPoseFieldSpaceEstimate());//add the array of data from that camera to the ArrayList
      }
    }

    return poses;//return the ArrayList containing the data from all cameras that see an april tag
  }

  /**
   * 
   * @return
   */
  public ArrayList<double[]> getRobotPose2dFieldSpaceEstimates() {// TODO finish javadoc comment

    //Array of double arrays, each array is in the order of x,y, rotation, timestamp
    ArrayList<double[]> pose2dEstimates = new ArrayList<double[]>();

    for(double[] pose3dData: getRobotPose3dFieldSpaceEstimates()) {//loop through all arrays in the 3d pose

      // Timestamp of the camera's values, current timestamp - the total latency of the camera, first converts latency from milliseconds to seconds.
      double timestamp = Timer.getTimestamp() - pose3dData[7]/1000.0;

      //add the 2d position and timestamp to the arraylist
      pose2dEstimates.add(new double[]{// TODO remove magic numbers
        pose3dData[0], 
        pose3dData[1], 
        pose3dData[4], // this should be index 5, through testing, it was found that index 4 is the yaw
        //currwent timestamp - the total latency of the camera, first converts latency from milliseconds to seconds,
        // then subtracts it from the current timestamp to get the timestamp of when the image was taken
        timestamp
      });
    }

    return pose2dEstimates;// return the array of 2d positions from the cameras that see april tags
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    ArrayList<double[]> robotPose2dFieldSpaceEstimates = getRobotPose2dFieldSpaceEstimates();

    boolean anyCameraHasPose = !robotPose2dFieldSpaceEstimates.isEmpty();// variable to store whether any cameras see an april tag, used for testing

    if(anyCameraHasPose) { /* if any cameras see an april tag, update the drivetrain's position with the camera's values, 
    if multiple cameras see an april tag, update the drivetrain's position with each camera's values, 
    this allows the drivetrain to use all of the information from all of the cameras to get a more accurate position on the field*/

      for(double[] poseEstimate: robotPose2dFieldSpaceEstimates) {// loop through all positions given by all cameras that see an AprilTag

        drivetrain.addVisionMeasurement( // update the drivetrain's position on the field with each camera's value
          new Pose2d(poseEstimate[0], poseEstimate[1], Rotation2d.fromDegrees(poseEstimate[2])), // convert the x, y, and yaw values into a Pose2d
          poseEstimate[3] // use the timestamp to allow different cameras to have different latency
          );// TODO add standard deviations for the vision measurements
      }

      /* SmartDashboard.putNumber("x", getRobotPose2dFieldSpaceEstimates().get(0)[0]);// for testing, put each of
      SmartDashboard.putNumber("y", getRobotPose2dFieldSpaceEstimates().get(0)[1]);// the position values to
      SmartDashboard.putNumber("yaw", getRobotPose2dFieldSpaceEstimates().get(0)[2]);// SmartDashboard */
      
    }
  }

  public class Limelight extends SubsystemBase {// extends subsystem base to allow for future use of limelight specific commands, 
    // not currently used as the limelight is only used in the vision subsystem, 
    // but this allows for more modular code in the future if we want to use limelight specific commands

    private NetworkTable table;
    private NetworkTableEntry targetX, targetY, targetArea, robotPoseFieldSpace, targetPoseRobotSpace, targetID, targetValid;

    /**
     * Creates a new Limelight
     * @param name , the hosname of the limelight
     */
    public Limelight(String name) {

      table = NetworkTableInstance.getDefault().getTable(name); // gets the network table for the limelight with the given name

      targetX = table.getEntry("tx");

      targetY = table.getEntry("ty");

      targetArea = table.getEntry("ta");

      targetValid = table.getEntry("tv");

      if (Alliance.Blue == DriverStation.getAlliance().get()) {// TODO potentialy change alliance.get to variable
        robotPoseFieldSpace = table.getEntry("botpose_orb_wpiblue");
      } else if (Alliance.Red == DriverStation.getAlliance().get()) {
        robotPoseFieldSpace = table.getEntry("botpose_orb_wpired");
      } else {
        robotPoseFieldSpace = table.getEntry("botpose_orb");
      }

      targetID = table.getEntry("tid");

      targetPoseRobotSpace = table.getEntry("targetpose_robotspace");
    }

    /**
     * Returns wether the limelight has a target that it is tracking.
     * @return
     * True or false if it sees a target or not.
     */
    public boolean hasTarget() {
      return targetValid.getInteger(0) == 1;// valid target is represented by 1, no target is represented by 0,
      // if the value is not 0 or 1, it will be treated as 0, the comparison is to convert it to a boolean value
    }

    /**
     * Returns the id of the target that is currently being tracked.<p>
     * If multiple targets are seen, returns the target specified by the limelight settings.
     * @return
     * The id of the aprilTag currently being tracked.
     */
    public int getTargetID() {
      return (int) targetID.getInteger(0);// casting long to int
    }

    /**
     * Returns the x angle from the crosshair to the center of the target.<p>
     * If multiple targets are seen, returns the center specified by the limelight settings.
     * @return
     * The x angle to the center of the target.
     */
    public double getTargetX() {
      return targetX.getDouble(0);
    }

    /**
     * Returns the y angle from the crosshair to the center of the target.<p>
     * If multiple targets are seen, returns the center specified by the limelight settings.
     * @return
     * The y angle to the center of the target.
     */
    public double getTargetY() {
      return targetY.getDouble(0);
    }

    /**
     * Returns the total percent of the limelight's field of view that is taken up by the target currently being tracked.<p>
     * If multiple targets are seen, returns the center specified by the limelight settings.
     * @return
     * The percentage of the field of view taken up by the current target.
     */
    public double getTargetArea() {
      return targetArea.getDouble(0);
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
    public double[] getRobotPoseFieldSpaceEstimate() {
      double[] defaultValue = new double[0];
      return robotPoseFieldSpace.getDoubleArray(defaultValue);
    }

    /**
     * Returns the targets position in the robot's coordinate system.<p>
     * Only works when in an aprilTag pipeline.
     * @return
     * An array containing the targets position in robot space, the order of the array is:<p>
     * Translation 
     */
    public double[] getTargetPoseRobotSpaceEstimate() {// TODO finish this javadoc comment
      double[] defaultValue = new double[0];
      return targetPoseRobotSpace.getDoubleArray(defaultValue);
    }

  }
}
