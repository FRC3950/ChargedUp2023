// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Constants;

public class Wrist extends SubsystemBase {
  /** Creates a new Wrist. */
  private final WPI_TalonFX wrist = new WPI_TalonFX(Constants.kIntake.wrist);
  private double kP = 0.0152, kI = 0.0, kD = 0.0, kF = 0.0;
  private boolean isInInfoMode = true;
  public final double kWristDropPosition = 0;
  public final double kWristRestPosition = 0;
  public Wrist() {
    setWristEncoder(0);

    // if(isInInfoMode){
    //   SmartDashboard.putNumber("Wrist kP", kP);
    //   SmartDashboard.putNumber("Wrist kI", kI);
    //   SmartDashboard.putNumber("Wrist kD", kD);
    //   SmartDashboard.putNumber("Wrist kF", kF);
    // }

    wrist.setInverted(true);
    wrist.setNeutralMode(NeutralMode.Brake);
    wrist.config_kP(0, kP);
    wrist.configAllowableClosedloopError(0, 300);  //allowable error or not to keep motor goin
  }

  public Command moveWristToPosition_Command(double distance) {
    return new RunCommand(
        () -> this.wrist.set(ControlMode.Position, distance),
        this);
  }

  /**
   * 
   * @param count Should be set to Constants.kIntake.maxEncoder to extend, and some minimum value to retract, ideally 0.  <- Bridgwood: I like it! Minn value basically 0 and when it hits limit switch it will instantly get to 0 and stop 
   */
  public void setDesiredCount(double count){
    wrist.set(ControlMode.Position, count);
  }

  public void setSpeed(double speed){
    wrist.set(speed);
  }
  
  public void setWristEncoder(double count){
    wrist.getSensorCollection().setIntegratedSensorPosition(count, 0);
  }

  public double getWristEncoder(){
    return wrist.getSelectedSensorPosition();
  }

  public boolean isLimitSwithEngaged(){
    return wrist.getSensorCollection().isRevLimitSwitchClosed() == 1;
  }

  /**
   * 
   * @return double array {kP, kI, kD, kF}
   */
  // public double[] getPIDDashboardConstants(){
  //   return new double[] {kP, kI, kD, kF};
  // }

  @Override
  public void periodic() {
    if(isInInfoMode){
      // kP = SmartDashboard.getNumber("Wrist kP", kP);
      // kI = SmartDashboard.getNumber("Wrist kI", kI);
      // kD = SmartDashboard.getNumber("Wrist kD", kD);
      // kF = SmartDashboard.getNumber("Wrist kF", kF);
      SmartDashboard.putBoolean("Wrist: Limit Switch Engaged?", isLimitSwithEngaged());
      SmartDashboard.putNumber("Wrist: Encoder Count", getWristEncoder());
    }

    if(isLimitSwithEngaged()){
        setWristEncoder(0);

    }
  }
}
