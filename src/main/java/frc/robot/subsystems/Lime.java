// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

<<<<<<< HEAD
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
=======
>>>>>>> c1ca908dec4fbf371b918e36b390cf2d020948d4
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;

public class Lime extends SubsystemBase {
  /** Creates a new Lime. */
<<<<<<< HEAD

  private final NetworkTable limeTable = NetworkTableInstance.getDefault().getTable("limelight");

  public Lime() {
    
=======
  public Lime() {

>>>>>>> c1ca908dec4fbf371b918e36b390cf2d020948d4
  }

  @Override
  public void periodic() {
<<<<<<< HEAD
    System.out.println(limeTable.getValue("tx").getDouble());
=======
    System.out.println(LimelightHelpers.getFiducialID("limelight"));
>>>>>>> c1ca908dec4fbf371b918e36b390cf2d020948d4
    // This method will be called once per scheduler run
  }
}
