// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.*;

public class CloseIntakeCommand extends CommandBase {
  /** Creates a new CloseIntakeCommand. */
  private final Intake intake;
  private final Telescope telescope;
  public CloseIntakeCommand(Intake intake, Telescope telescope) {
    this.intake = intake;
    this.telescope = telescope;
    addRequirements(intake, telescope);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(telescope.getEncoder() > 100000)
      intake.setIntake(DoubleSolenoid.Value.kForward);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
