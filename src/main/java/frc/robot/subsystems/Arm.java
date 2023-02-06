// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Arm extends SubsystemBase {

  private TalonFX armMotor;




  /** Creates a new Arm. */
  public Arm() {

/* Factory default hardware to prevent unexpected behavior */
armMotor.configFactoryDefault();

/* Configure Sensor Source for Pirmary PID */
armMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, Constants.kPIDLoopIdx,
    Constants.kTimeoutMs);

/* set deadband to super small 0.001 (0.1 %).
  The default deadband is 0.04 (4 %) */
armMotor.configNeutralDeadband(0.001, Constants.kTimeoutMs);

/**
 * Configure Talon FX Output and Sensor direction accordingly Invert Motor to
 * have green LEDs when driving Talon Forward / Requesting Postiive Output Phase
 * sensor to have positive increment when driving Talon Forward (Green LED)
 */
armMotor.setSensorPhase(false);
armMotor.setInverted(false);
/*
 * Talon FX does not need sensor phase set for its integrated sensor
 * This is because it will always be correct if the selected feedback device is integrated sensor (default value)
 * and the user calls getSelectedSensor* to get the sensor's position/velocity.
 * 
 * https://phoenix-documentation.readthedocs.io/en/latest/ch14_MCSensor.html#sensor-phase
 */
    // armMotor.setSensorPhase(true);

/* Set relevant frame periods to be at least as fast as periodic rate */
armMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, Constants.kTimeoutMs);
armMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, Constants.kTimeoutMs);

/* Set the peak and nominal outputs */
armMotor.configNominalOutputForward(0, Constants.kTimeoutMs);
armMotor.configNominalOutputReverse(0, Constants.kTimeoutMs);
armMotor.configPeakOutputForward(1, Constants.kTimeoutMs);
armMotor.configPeakOutputReverse(-1, Constants.kTimeoutMs);

/* Set Motion Magic gains in slot0 - see documentation */
armMotor.selectProfileSlot(Constants.kSlotIdx, Constants.kPIDLoopIdx);
armMotor.config_kF(Constants.kSlotIdx, Constants.kGains.kF, Constants.kTimeoutMs);
armMotor.config_kP(Constants.kSlotIdx, Constants.kGains.kP, Constants.kTimeoutMs);
armMotor.config_kI(Constants.kSlotIdx, Constants.kGains.kI, Constants.kTimeoutMs);
armMotor.config_kD(Constants.kSlotIdx, Constants.kGains.kD, Constants.kTimeoutMs);

/* Set acceleration and vcruise velocity - see documentation */
armMotor.configMotionCruiseVelocity(15000, Constants.kTimeoutMs);
armMotor.configMotionAcceleration(6000, Constants.kTimeoutMs);

/* Zero the sensor once on robot boot up */
armMotor.setSelectedSensorPosition(0, Constants.kPIDLoopIdx, Constants.kTimeoutMs);




  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setArmHeight(double height){
    armMotor.set(ControlMode.MotionMagic, height,DemandType.ArbitraryFeedForward,0.1);

  }

}
