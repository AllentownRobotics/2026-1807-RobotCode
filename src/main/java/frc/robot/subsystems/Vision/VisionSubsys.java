// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Vision;

import java.lang.reflect.Array;
import java.time.Duration;
import java.util.ArrayList;

import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;

import edu.wpi.first.math.VecBuilder;
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
import frc.robot.subsystems.Drive.CommandSwerveDrivetrain;
import frc.robot.subsystems.Vision.LimelightHelpers.PoseEstimate;

public class VisionSubsys extends SubsystemBase {
  private Limelight[] limelights;
  private int limelightCounter;

  CommandSwerveDrivetrain drivetrain;// this is irregilar to use a different subsystem inside of a subsystem, 
  // this is because the vision subsystem needs to update the drivetrain's position with the vision measurements,
  // so it needs to have access to the drivetrain. This is not a common practice but it is the most efficient way to 
  // update the drivetrain's position with vision measurements.

  /** Creates a new Vision. */
  public VisionSubsys(CommandSwerveDrivetrain drivetrain) {
    limelights = new Limelight[]{ // add all of the limelights used for april tags here
      new Limelight("limelight-hl"),
      new Limelight("limelight-hr"),
      
      new Limelight("limelight-bl"),
      new Limelight("limelight-br")
    //   new Limelight("limelight-collect")
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
  public ArrayList<PoseEstimate> getRobotPose3dFieldSpaceEstimates() {

    // array of double arrays, each array is in the order of x,y,z, pitch, yaw, roll, total latency, 
    // tag count, tag span, average tag distance from camera, average tag percent of image
    ArrayList<PoseEstimate> poses = new ArrayList<PoseEstimate>(); 

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
  public ArrayList<PoseEstimateAndStDevs> getRobotPose2dFieldSpaceEstimates() {// TODO finish javadoc comment

    // //Array of double arrays, each array is in the order of x,y, rotation, timestamp
    // ArrayList<double[]> pose2dEstimates = new ArrayList<double[]>();

    // for(double[] pose3dData: getRobotPose3dFieldSpaceEstimates()) {//loop through all arrays in the 3d pose

    //   // Timestamp of the camera's values, current timestamp - the total latency of the camera, first converts latency from milliseconds to seconds.
    //   double timestamp = Timer.getTimestamp() - pose3dData[7]/1000.0;

    //   //add the 2d position and timestamp to the arraylist
    //   pose2dEstimates.add(new double[]{// TODO remove magic numbers
    //     pose3dData[0], 
    //     pose3dData[1], 
    //     pose3dData[4], // this should be index 5, through testing, it was found that index 4 is the yaw
    //     //currwent timestamp - the total latency of the camera, first converts latency from milliseconds to seconds,
    //     // then subtracts it from the current timestamp to get the timestamp of when the image was taken
    //     timestamp
    //   });
    // }

    // return pose2dEstimates;// return the array of 2d positions from the cameras that see april tags

    ArrayList<PoseEstimateAndStDevs> pose2dEstimates = new ArrayList<PoseEstimateAndStDevs>();
    for(Limelight limelight : limelights){//loops through all limelights
      if (limelight.hasTarget()) {//if the limelight sees an april tag
        PoseEstimateAndStDevs poseEstimate = new PoseEstimateAndStDevs(limelight.getRobotPoseFieldSpaceEstimate(), limelight.name);
        poseEstimate.addStDevs(limelight.getStdevs());
        pose2dEstimates.add(poseEstimate);
      }
      
    }
    return pose2dEstimates;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
        for(Limelight limelight : limelights){//loops through all limelights
      if (DriverStation.isEnabled()) {
        LimelightHelpers.SetIMUMode(limelight.name, 4);
      }else{
        LimelightHelpers.SetIMUMode(limelight.name, 1);
      }
      
    }

    
      ArrayList<PoseEstimateAndStDevs> robotPose2dFieldSpaceEstimates = getRobotPose2dFieldSpaceEstimates();
  
      // loop through all positions given by all cameras that see an AprilTag
      for(PoseEstimateAndStDevs poseEstimate: robotPose2dFieldSpaceEstimates) {  
        
        // boolean useEstimate = true;

        // if(DriverStation.isAutonomous()){ 
        // if(poseEstimate.getLimelightName() == "limelight-br"){
        //   useEstimate = false;
        // }
        // if(poseEstimate.getLimelightName() == "Limelight-bl"){
        //   useEstimate = false;
        // }

        //drivetrain.setVisionMeasurementStdDevs(VecBuilder.fill(0.2, 0.2, 9999999)); // n1 = x, n2 = y, n3 = rotation
      
          
          PoseEstimate estimate = poseEstimate.getPoseEstimate();
            
          boolean goodEstimate = (estimate.tagCount > 0) // we see multiple tags
                                  && (estimate.avgTagDist <= 5) // trust if distance is less than 5 meters
                                  && !((estimate.tagCount == 1)  // if 1 tag is seen with ambiguity below 0.2, trust it. If we see 1 that has higher ambiguity dont trust it, if we see 2 or more tags trust it regardless because there is no ambiguity.
                                        && (estimate.rawFiducials[0].ambiguity < 0.2)); 

          if(goodEstimate){
          drivetrain.addVisionMeasurement( // update the drivetrain's position on the field with each camera's value
            new Pose2d(poseEstimate.getPoseEstimate().pose.getX(), poseEstimate.getPoseEstimate().pose.getY(), poseEstimate.getPoseEstimate().pose.getRotation()), // convert the x, y, and yaw values into a Pose2d
            poseEstimate.getPoseEstimate().timestampSeconds
            ,VecBuilder.fill(poseEstimate.getStDevs()[0], poseEstimate.getStDevs()[1], poseEstimate.getStDevs()[5]) // use the timestamp to allow different cameras to have different latency
            );
            
            limelightCounter += limelightCounter;
            SmartDashboard.putNumber("Limelight updates", limelightCounter);
        }
    }
  }

  public class PoseEstimateAndStDevs{

    private PoseEstimate poseEstimate;
    private double[] stdevs;
    private String limelightName;
    
    public PoseEstimateAndStDevs(PoseEstimate poseEstimate, String limelightName){
      this.poseEstimate = poseEstimate;
      this.limelightName = limelightName;
    }

    public PoseEstimate getPoseEstimate(){
      return poseEstimate;
    }

    public void addStDevs(double[] stdevs){
      this.stdevs = stdevs;
    }

    public double[] getStDevs(){
      return stdevs;
    }
    
    public String getLimelightName(){
      return this.limelightName;
    }

  }

  public class Limelight extends SubsystemBase {// extends subsystem base to allow for future use of limelight specific commands, 
    // not currently used as the limelight is only used in the vision subsystem, 
    // but this allows for more modular code in the future if we want to use limelight specific commands

    private NetworkTable table;
    private NetworkTableEntry targetX, targetY, targetArea, robotPoseFieldSpace, targetPoseRobotSpace, targetID, targetValid;
    public String name;

    /**
     * Creates a new Limelight
     * @param name , the hosname of the limelight
     */
    public Limelight(String name) {

      table = NetworkTableInstance.getDefault().getTable(name); // gets the network table for the limelight with the given name
      this.name = name;


      LimelightHelpers.SetIMUMode(name, 1);
      
      targetX = table.getEntry("tx");

      targetY = table.getEntry("ty");

      targetArea = table.getEntry("ta");

      targetValid = table.getEntry("tv");

      robotPoseFieldSpace = table.getEntry("botpose_orb_wpiblue");
      
      // if (Alliance.Blue == DriverStation.getAlliance().get()) {// TODO potentialy change alliance.get to variable
      //   robotPoseFieldSpace = table.getEntry("botpose_orb_wpiblue");
      // } else if (Alliance.Red == DriverStation.getAlliance().get()) {
      //   robotPoseFieldSpace = table.getEntry("botpose_orb_wpired");
      // } else {
      //   robotPoseFieldSpace = table.getEntry("botpose_orb");
      // }

      targetID = table.getEntry("tid");

      targetPoseRobotSpace = table.getEntry("targetpose_robotspace");
    }

    public double[] getStdevs(){
      return table.getEntry("stdevs").getDoubleArray(new double[]{
        0.2,
        0.2,
        0.2,
        9999999,
        9999999,
        9999999,
        0.1,
        0.1,
        0.1,
        9999999,
        9999999,
        9999999
      });
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
    public PoseEstimate getRobotPoseFieldSpaceEstimate() {
    //   double[] defaultValue = new double[0];
    //   return robotPoseFieldSpace.getDoubleArray(defaultValue);
      PoseEstimate poseEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue(name);
      return poseEstimate;
    }

    public PoseEstimate getRobotPose2dFieldSpaceEstimateMT2() {

      SwerveDriveState driveState = drivetrain.getState();

// We commented this aprt out because we did not want the robot to switch ro
    //   if(LimelightHelpers.getTargetCount(name) > 1){
    //     LimelightHelpers.SetRobotOrientation(name, LimelightHelpers.getBotPoseEstimate_wpiBlue(name).pose.getRotation().getDegrees(), Math.toDegrees(driveState.Speeds.omegaRadiansPerSecond),0.0,0.0,0.0,0.0);
    //   } else {
        LimelightHelpers.SetRobotOrientation(name, driveState.Pose.getRotation().getDegrees(), Math.toDegrees(driveState.Speeds.omegaRadiansPerSecond),0.0,0.0,0.0,0.0);
    //   }
      PoseEstimate poseEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(name);
      return poseEstimate;
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
