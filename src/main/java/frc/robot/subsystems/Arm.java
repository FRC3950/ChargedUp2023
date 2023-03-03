// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Arm extends SubsystemBase {

  private final WPI_TalonFX upperArm = new WPI_TalonFX(Constants.kArm.UpperArm);
  private final WPI_TalonFX lowerArm = new WPI_TalonFX(Constants.kArm.LowerArm);

  //Need Encoder ID  0a   1b

  //Need Limit Switch ID fal 5

  /** Creates a new Arm. */
  public Arm() {

    
  }

  
public boolean isLimitSwithEngaged(){


  return upperArm.getSensorCollection().isFwdLimitSwitchClosed()==1?true:false;
}


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
