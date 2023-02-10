// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Telescope extends SubsystemBase {
  /** Creates a new Telescope. */
  private final WPI_TalonFX leader = new WPI_TalonFX(Constants.kTelescope.leader);
  private final WPI_TalonFX follower = new WPI_TalonFX(Constants.kTelescope.follower);
  private final DoubleSolenoid brake = new DoubleSolenoid(PneumaticsModuleType.REVPH, Constants.kTelescope.forward, Constants.kTelescope.reverse);
  private boolean isInInfoMode = false;

  public Telescope() {
    setEncoder(0);
    follower.follow(leader); //might need to invert follower
  }

  public void setEncoder(int count){
    leader.getSensorCollection().setIntegratedSensorPosition(count, 0);
  }

  public double getEncoder(){
    return leader.getSelectedSensorPosition();
  }

  public void setMotor(double count){
    leader.set(ControlMode.Position, count); //not sure if this is ideal
  }

  public void toggleBrake(){
    if(brake.get() != Constants.kIntake.EXTENDED){
      brake.set(Constants.kIntake.EXTENDED);
    }
    else {
      brake.set(Constants.kIntake.RETRACTED);
    }
  }

  public void setBrake(Value state){
    brake.set(state);
  }

  @Override
  public void periodic() {
    if(isInInfoMode){
      SmartDashboard.putNumber("Telescope encoder count", getEncoder());
    }
  }
}
