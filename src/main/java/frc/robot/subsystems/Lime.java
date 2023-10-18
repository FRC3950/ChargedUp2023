// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

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
  
  
}
