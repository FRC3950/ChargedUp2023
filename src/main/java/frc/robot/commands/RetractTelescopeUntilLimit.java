// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.*;

public class RetractTelescopeUntilLimit extends CommandBase {
  /** Creates a new RetractTelescopeUntilLimit. */
  private final Telescope telescope;
  public RetractTelescopeUntilLimit(Telescope telescope) {
    this.telescope = telescope;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(telescope);
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    telescope.setBrake(Value.kReverse);
    telescope.setPercent(-.9);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("T");
    telescope.setPercent(0.0);
    telescope.setBrake(Value.kForward);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return telescope.isLimitEngaged();
  }
}
