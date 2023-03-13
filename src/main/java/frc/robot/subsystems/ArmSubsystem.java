// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ArmSubsystem extends SubsystemBase {

  private final WPI_TalonFX masterArm = new WPI_TalonFX(Constants.kArm.UpperArm);
  private final WPI_TalonFX slaveArm = new WPI_TalonFX(Constants.kArm.LowerArm);
  private final Encoder encoder = new Encoder(0, 1);
  private boolean isInInfoMode = true;
  private final DoubleSolenoid armLock = new DoubleSolenoid(50, PneumaticsModuleType.REVPH, Constants.kLock.closed,
      Constants.kLock.open);

  // Notes:re
  // Encoder -> 0a, 1b
  // Limit Switch Falcon (5)
  // Lock -> 2 closed, 3 open

  /** Creates a new ArmSubsystem. */
  public ArmSubsystem() {

    SmartDashboard.putNumber("Arm kF", 0.00);
    SmartDashboard.putNumber("Arm kP", 0.0); 
    SmartDashboard.putNumber("Arm kI", 0.0);

    SmartDashboard.putNumber("Arm kD", 0.0);

    SmartDashboard.putNumber("mm_Accel", 2000);
    SmartDashboard.putNumber("mm_Vel", 2000);

    SmartDashboard.putNumber("targetPosition", 0);


    encoder.reset();

    slaveArm.configClosedloopRamp(0.2);
    masterArm.configClosedloopRamp(0.2);

    slaveArm.follow(masterArm);

  }

  public void armToAngle() {

    // 2048 for full talon rev

    masterArm.config_kF(0, SmartDashboard.getNumber("Arm kF", 0.00));
    masterArm.config_kP(0, SmartDashboard.getNumber("Arm kP", 0.0)); // Start with .2497
    masterArm.config_kI(0, SmartDashboard.getNumber("Arm kI", 0.0));
    masterArm.config_kD(0, SmartDashboard.getNumber("Arm kD", 0.0));

    masterArm.configMotionAcceleration(SmartDashboard.getNumber("mm_Accel", 2000));
    masterArm.configMotionCruiseVelocity(SmartDashboard.getNumber("mm_Vel", 2000));

    masterArm.configMotionSCurveStrength(3);

    double targetPosition = SmartDashboard.getNumber("targetPosition", 0);


    masterArm.set(ControlMode.Position, targetPosition);
    //masterArm.set(ControlMode.MotionMagic, targetPosition, DemandType.ArbitraryFeedForward, 0.05);

  }

  public Command moveArmToPostionCommand() {

    return runOnce(
        () -> {
          this.armToAngle();
        });
  }

  public void armPercentCommand(double percent){
    masterArm.set(percent);
  }

  public Command zeroSensorFalcons() {
    return runOnce(
      () -> {
        masterArm.setSelectedSensorPosition(0);
        slaveArm.setSelectedSensorPosition(0);
      }
    );
  }

  public void lockArm(){
    armLock.set(Value.kForward); //FIXME
  }

  public void unlockArm(){
    armLock.set(Value.kReverse);
  }

  public DoubleSolenoid.Value getSolenoid(){
    return armLock.get();
  }

  public Command toggleArm() {
    return runOnce(
        () -> {
          armLock.toggle();
        });
  }

  public boolean isLimitSwithEngaged() {
    return masterArm.getSensorCollection().isRevLimitSwitchClosed() == 1;
  }

  public void resetEncoderCountArmMotors(){
    masterArm.setSelectedSensorPosition(0);
    slaveArm.setSelectedSensorPosition(0) ;
  }

  public double getArmEcnoderAngle(){
    return encoder.getDistance(); 
  }

  public void setMotors(double t){
    masterArm.set(t);
  }
  

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if(isInInfoMode){
      SmartDashboard.putBoolean("Limit Switch Engaged", isLimitSwithEngaged());
      SmartDashboard.putNumber("angle", encoder.getDistance());
      SmartDashboard.putNumber("Encoder Rate", encoder.getRate());
      SmartDashboard.putNumber("Limit Switch value", masterArm.getSensorCollection().isFwdLimitSwitchClosed());
      SmartDashboard.putNumber("Limit Rev Switch value", masterArm.getSensorCollection().isRevLimitSwitchClosed());
      SmartDashboard.putNumber("Encoder Distance per Pulse", encoder.getDistancePerPulse());
      SmartDashboard.putNumber("Master Interal Encoder Count", masterArm.getSelectedSensorPosition());

      SmartDashboard.putBoolean("Solenoid state", (getSolenoid().equals(Value.kForward)));
    }
    
    if(isLimitSwithEngaged()){
      encoder.reset();
    }
  }

}
