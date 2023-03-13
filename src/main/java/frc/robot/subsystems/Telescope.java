// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Telescope extends SubsystemBase {
  /** Creates a new Telescope. */
  private final WPI_TalonFX leader = new WPI_TalonFX(Constants.kTelescope.leader);
  private final DoubleSolenoid brake = new DoubleSolenoid(50, PneumaticsModuleType.REVPH, Constants.kTelescope.forward, Constants.kTelescope.reverse);
  private boolean isInInfoMode = true;

  public Telescope() {
    setEncoder(0);
    leader.setNeutralMode(NeutralMode.Brake);
    leader.setInverted(true);

    leader.config_kP(0, 0.012978);
    leader.configAllowableClosedloopError(0, 590);
    
  }

  //Needs Tuner to Find Extension Distance
  // 0.75 x 1023   /  (max distance)
  // Let's make sure sensor is in correct phase and + makes it extend in tuner!
  public Command extendArmToDistance_Command(double distance) {
    return new StartEndCommand(
        () -> {
          this.leader.set(ControlMode.Position, distance);},
        () -> this.leader.set(0),
        this);
  }

  public void setEncoder(int count){
    leader.getSensorCollection().setIntegratedSensorPosition(count, 0);
  }

  public double getEncoder(){
    return leader.getSelectedSensorPosition();
  }

  public void setMotor(double count){
    leader.set(ControlMode.Position, count); //not sure if this is ideal  <- Bridgwood: it is if we have the right PID :)
  }

  public void setPercent(double speed){
    leader.set(speed);
  }

  public void toggleBrake(){
    if(!brake.get().equals(Constants.kIntake.EXTENDED)){
      brake.set(Constants.kIntake.EXTENDED);
    }
    else {
      brake.set(Constants.kIntake.RETRACTED);
    }
  }

  public void setBrake(Value state){
    brake.set(state);
  }

  public Value getBrake(){
    return brake.get();
  }

  @Override
  public void periodic() {
    if(isInInfoMode){
      SmartDashboard.putNumber("Telescope: Encoder Count", getEncoder());
   

      
    }



    if(leader.getSensorCollection().isRevLimitSwitchClosed() == 1){
      setEncoder(0);
    }
    

  }
}
