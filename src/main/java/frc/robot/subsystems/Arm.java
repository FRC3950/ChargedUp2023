// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;



import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Arm extends SubsystemBase {

  private final WPI_TalonFX upperArm = new WPI_TalonFX(Constants.kArm.UpperArm);
  private final WPI_TalonFX lowerArm = new WPI_TalonFX(Constants.kArm.LowerArm);
  private final Encoder encoder = new Encoder(0, 1);

  //Need Encoder ID  0a   1b

  //Need Limit Switch ID fal 5

  //2 close /3 open for the arm lock

  /** Creates a new Arm. */
  public Arm() {

upperArm.follow(lowerArm);
    
  }


public void armToAngle(){

  lowerArm.config_kF(0, SmartDashboard.getNumber("lowerArm kF", 0.0));
  lowerArm.config_kP(0, SmartDashboard.getNumber("lowerArm kP", 0.0));
  lowerArm.config_kI(0, SmartDashboard.getNumber("lowerArm kI", 0.0));
  lowerArm.config_kD(0, SmartDashboard.getNumber("lowerArm kD", 0.0));

  lowerArm.set(ControlMode.MotionMagic, 0, null, 0);


lowerArm.set(ControlMode.Position, SmartDashboard.getNumber("Arm Angle in Ticks", 0));


  //lowerArm.set(TalonFXControlMode.MotionMagic, SmartDashboard.getNumber("Arm Angle in Ticks", 0), DemandType.ArbitraryFeedForward, SmartDashboard.getNumber("FeedForward", 0.05));


//Feedforwad will change based on angle - use below to reference

  // int kMeasuredPosHorizontal = 840; //Position measured when arm is horizontal
  // double kTicksPerDegree = 4096 / 360; //Sensor is 1:1 with arm rotation
  // int currentPos = _motorcontroller.getSelectedSensorPosition();
  // double degrees = (currenPos - kMeasuredPosHorizontal) / kTicksPerDegree;
  // double radians = java.lang.Math.toRadians(degrees);
  // double cosineScalar = java.lang.Math.cos(radians);
  
  // double maxGravityFF = 0.07;
  // _motorcontroller.set(ControlMode.MotionMagic, targetPos, DemandType.ArbitraryFeedForward, maxGravityFF * cosineScalar);

}
  
public boolean isLimitSwithEngaged(){
  return upperArm.getSensorCollection().isFwdLimitSwitchClosed()==1 ? true : false;
}


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("Limit Switch Engaged", isLimitSwithEngaged());
    SmartDashboard.putNumber("angle", encoder.getDistance());
    SmartDashboard.putNumber("Encoder Rate", encoder.getRate());
  }
}
