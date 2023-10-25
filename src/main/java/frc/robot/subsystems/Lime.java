// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Lime extends SubsystemBase {
  /** Creates a new Lime. */

  private final NetworkTable lime3Table = NetworkTableInstance.getDefault().getTable("limelight-zero");
  private final NetworkTable lime2Table = NetworkTableInstance.getDefault().getTable("limelight-one");

  NetworkTableEntry targetPose = lime3Table.getEntry("targetpose_cameraspace");


  public Lime() {
    
  }

  @Override
  public void periodic() {

    double[] coordToTag = targetPose.getDoubleArray(new double[6]);
    double aprilTagId_value = lime3Table.getEntry("tid").getDouble(0);

    SmartDashboard.putNumber("April_ID", aprilTagId_value);
    SmartDashboard.putNumberArray("Array Cord to Target", coordToTag);
    //getValue() returns a NetworkTableValue, which is basically a typed union, as NetworkTable values can be one of several possible type
    //System.out.println(lime3Table.getValue("tx").getDouble());


    // This method will be called once per scheduler run
  }

  public PathPlannerTrajectory getPathToAprilTag(){
    double[] coordToTag = targetPose.getDoubleArray(new double[6]);
    return PathPlanner.generatePath(
      new PathConstraints(5, 5), 
      new PathPoint(new Translation2d(0, 0), Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(0)),
      new PathPoint(new Translation2d(coordToTag[0], -(coordToTag[1] + 1)), Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(0))
      //new PathPoint(new Translation2d(coordToTag[0], coordToTag[2]), Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(0))
    );
  }; 
}
