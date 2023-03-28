// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.LimelightResults;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTableEntry;

public class Limelight extends SubsystemBase {
  /** Creates a new Limelight. */

  private final NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
NetworkTableEntry tx = table.getEntry("tx");
NetworkTableEntry ty = table.getEntry("ty");
NetworkTableEntry ta = table.getEntry("ta");
  

  public Limelight() {

    //build commands within limelight sub
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    table.getEntry("<variablename>").getDoubleArray(new double[6]);

    double x = tx.getDouble(0.0);
    double y = ty.getDouble(0.0);
    double area = ta.getDouble(0.0);

    SmartDashboard.putNumber("LimelightX", x);
    SmartDashboard.putNumber("LimelightY", y);
    SmartDashboard.putNumber("LimelightArea", area);

    double[] targetPose = LimelightHelpers.getTargetPose_RobotSpace("");
for (double d : targetPose) {
  System.out.println(d);
}    
System.out.println("---------------");
    LimelightHelpers.getBotPose3d_TargetSpace("");

    

  
  }
}
