// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;

public class Lime extends SubsystemBase {
  /** Creates a new Lime. */
  public Lime() {

  }

  @Override
  public void periodic() {
    System.out.println(LimelightHelpers.getFiducialID("limelight"));
    // This method will be called once per scheduler run
  }
}
