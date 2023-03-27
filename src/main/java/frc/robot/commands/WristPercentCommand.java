// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import frc.robot.subsystems.Wrist;

public class WristPercentCommand extends CommandBase {
  /** Creates a new WristPercentCommand. */
  private final Wrist s_Wrist;
  private final DoubleSupplier percent;
  private boolean isHolding = true;
  private double holdPoint = 0;

  public WristPercentCommand(Wrist s_Wrist, DoubleSupplier percent) {
    this.s_Wrist = s_Wrist;
    this.percent = percent;
    addRequirements(s_Wrist);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if(percent.getAsDouble() > 0.05 || percent.getAsDouble() < -0.05){
      s_Wrist.setSpeed(percent.getAsDouble());
      holdPoint = s_Wrist.getWristEncoder();
    }
    else {
      s_Wrist.setSpeed(0.000018 * (holdPoint - s_Wrist.getWristEncoder()) + 0.05);
    }
 
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
