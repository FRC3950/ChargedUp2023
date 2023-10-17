// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;

public class Lime extends SubsystemBase {
  /** Creates a new Lime. */

  private final NetworkTable lime3Table = NetworkTableInstance.getDefault().getTable("limelight-zero");
  private final NetworkTable lime2Table = NetworkTableInstance.getDefault().getTable("limelight-one");

  public Lime() {
    
  }

  @Override
  public void periodic() {
    System.out.println(lime3Table.getValue("tx").getDouble());
    // This method will be called once per scheduler run
  }
}
