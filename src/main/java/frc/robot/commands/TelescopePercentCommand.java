// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Telescope;

public class TelescopePercentCommand extends CommandBase {

  private final Telescope s_Telescope;
  private DoubleSupplier percent;

  public TelescopePercentCommand(Telescope s_Telescope, DoubleSupplier percent) {
    this.s_Telescope = s_Telescope;
    this.percent = percent;
    addRequirements(s_Telescope);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    s_Telescope.setBrake(Value.kForward);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //System.out.println(percent.getAsDouble());
    if(percent.getAsDouble() > 0.1 || percent.getAsDouble() < -0.1 && !s_Telescope.isLimitEngaged()){
      if(!s_Telescope.getBrake().equals(Value.kReverse)){
        s_Telescope.setBrake(Value.kReverse);
      }
      s_Telescope.setPercent(percent.getAsDouble());
    }
    else {
      s_Telescope.setBrake(Value.kForward);
      s_Telescope.setPercent(0.0);
    }
  }
  //Josh Wrote this part of the code and it will work perfectly. I think...
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    s_Telescope.setPercent(0.0);
    s_Telescope.setBrake(Value.kForward);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
