// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.cscore.VideoSource.ConnectionStrategy;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
  /** Creates a new IntakeSubsytem. */
  private final WPI_TalonFX upper = new WPI_TalonFX(Constants.kIntake.upperID);
  private final WPI_TalonFX lower = new WPI_TalonFX(Constants.kIntake.lowerID);
  //private final WPI_TalonFX wrist = new WPI_TalonFX(Constants.kIntake.wrist);

  boolean isInInfoMode = false;
  // Will also need a sensor at some point. 

  private final DoubleSolenoid solenoid = new DoubleSolenoid(50, PneumaticsModuleType.REVPH, Constants.kIntake.forward, Constants.kIntake.reverse);

  public Intake() {
    upper.setNeutralMode(NeutralMode.Brake);
    lower.setNeutralMode(NeutralMode.Brake);
  }

  public DoubleSolenoid.Value getState(){
    return solenoid.get();
  }

  public void toggleSolenoid(){
    System.out.println(getState().toString()); //FIXME
    Value newState = (getState().equals(Value.kForward)) ? Constants.kIntake.RETRACTED : Constants.kIntake.EXTENDED;
    solenoid.set(newState);
    System.out.println(newState.toString());
  }

  public void setIntake(Value newState){
    solenoid.set(newState);
  }

  public void closeIntake(){
    solenoid.set(Value.kForward);
  }
 

  public void setIntake(double speed){
    upper.set(speed);
    lower.set(speed);
  }

  public boolean limitIsEngaged(){
    return lower.getSensorCollection().isFwdLimitSwitchClosed() == 1;
  }

  @Override
  public void periodic() {
    if(isInInfoMode){
      SmartDashboard.putBoolean("Intake solenoid", (getState().equals(Constants.kIntake.EXTENDED)));
    }
    // if(limitIsEngaged()){
    //   upper.set(0);
    //   lower.set(0);
    // }
  }
}
