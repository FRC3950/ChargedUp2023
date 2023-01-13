// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.*;

public class AutoBalanceCommand extends CommandBase {
  /** Creates a new AutoBalanceCommand. */
  private final Swerve drivetrain;

  
  private final double kP = 0.1; //genuinely have no idea if this is even close
  private final double deadzone = 0.5;

  private double zeroPosition = 0;
  private double speed = 0.0;
  

  public AutoBalanceCommand(Swerve drivetrain) {
    this.drivetrain = drivetrain;

    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    zeroPosition = drivetrain.getPitch();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(drivetrain.getPitch() != zeroPosition + deadzone || drivetrain.getPitch() != zeroPosition - deadzone){
      speed = kP * drivetrain.getPitch();
      drivetrain.driveVertical(speed);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.driveVertical(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
