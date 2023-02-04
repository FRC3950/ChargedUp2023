// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Telescope extends SubsystemBase {
  /** Creates a new Telescope. */
  private final WPI_TalonFX falcon = new WPI_TalonFX(Constants.Telescope.motor);
  private final DoubleSolenoid brake = new DoubleSolenoid(PneumaticsModuleType.REVPH, Constants.Telescope.forward, Constants.Telescope.reverse);
  private boolean isInInfoMode = false;

  public Telescope() {
    setEncoder(0);
  }

  public void setEncoder(int count){
    falcon.getSensorCollection().setIntegratedSensorPosition(count, 0);
  }

  public double getEncoder(){
    return falcon.getSelectedSensorPosition();
  }

  public void setMotor(double count){
    falcon.set(ControlMode.Position, count); //not sure if this is ideal
  }

  @Override
  public void periodic() {
    if(isInInfoMode){
      SmartDashboard.putNumber("Telescope encoder count", getEncoder());
    }
  }
}
