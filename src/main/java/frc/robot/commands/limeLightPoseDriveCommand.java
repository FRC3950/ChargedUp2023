// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.wpilibj2.command.CommandBase;
//import frc.robot.LimelightHelpers;
import frc.robot.subsystems.Lime;

public class LimelightPoseDriveCommand extends CommandBase {
  /** Creates a new limeLightPoseDriveCommand. */

  private final Lime limelightSubsystem;
  private final SwerveAutoBuilder swerveAutoBuilder;

  private final boolean isRight;


  public LimelightPoseDriveCommand(final boolean isRight, final Lime limelightSubsystem, final SwerveAutoBuilder swerveAutoBuilder) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.isRight = isRight;
    this.limelightSubsystem = limelightSubsystem;
    this.swerveAutoBuilder = swerveAutoBuilder;

    addRequirements(limelightSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void execute() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void initialize() {
    swerveAutoBuilder.fullAuto(limelightSubsystem.getPathToAprilTag());
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
