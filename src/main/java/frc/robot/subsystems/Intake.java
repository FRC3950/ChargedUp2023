// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
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

  private final double kP, kI, kD, kF, kMaxSensorVelocity; //FIXME

  boolean isInInfoMode = false;
  // Will also need a sensor at some point. 

  private final DoubleSolenoid solenoid = new DoubleSolenoid(50, PneumaticsModuleType.REVPH, Constants.kIntake.forward, Constants.kIntake.reverse);

  public Intake() {
    //All MotionMagic stuff with wrist will go here if we decide to use it. 

    //FIXME
    kP = 0.0;
    kI = 0.0;
    kD = 0.0;
    kF = 0.0;


    kMaxSensorVelocity = 4096 * 2;

    //wrist.config_kP(0, kP);
    //wrist.config_kI(0, kI);
    //wrist.config_kD(0, kD);
    //wrist.config_kF(0, kF);

    //Other motion magic stuff will go here


    //wrist.configMotionAcceleration(kMaxSensorVelocity/2, 0);
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

 

  public void setIntake(double speed){
    upper.set(speed);
    lower.set(speed);
  }

  @Override
  public void periodic() {
    if(isInInfoMode){
      SmartDashboard.putBoolean("Intake solenoid", (getState().equals(Constants.kIntake.EXTENDED)));
    }
  }
}
