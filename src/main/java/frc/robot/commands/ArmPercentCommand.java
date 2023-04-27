// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;

public class ArmPercentCommand extends CommandBase {
  /** Creates a new ArmPercentCommand. */
  private final DoubleSupplier percent;
  private final Arm arm;
  public ArmPercentCommand(Arm arm, DoubleSupplier percent) {
    this.arm = arm;
    this.percent = percent;
    arm.lockArm();
    addRequirements(arm);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if (percent.getAsDouble() > 0.1 || percent.getAsDouble() < -0.1) {
      if (!arm.getSolenoid().equals(Value.kReverse)) {
        arm.unlockArm();

      }

      if (percent.getAsDouble() < -0.1) {
        arm.armPercentCommand(percent.getAsDouble() * 0.15);

      }

      if (percent.getAsDouble() > 0.1)
      {
        if(arm.getArmEcnoderAngle() < 300){
          arm.armPercentCommand(percent.getAsDouble());
        }
        else{
          arm.armPercentCommand(0);
        }

      } 
      
    }
    
    
    else {
      arm.lockArm();
      arm.armPercentCommand(0);
    }
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    arm.armPercentCommand(0.0);
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
