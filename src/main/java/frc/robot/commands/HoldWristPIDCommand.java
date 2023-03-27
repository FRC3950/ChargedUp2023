// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.subsystems.Wrist;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class HoldWristPIDCommand extends PIDCommand {
  /** Creates a new WristPIDCommand. */
  Wrist wrist;
  public HoldWristPIDCommand(Wrist wrist, double setpoint) {
    super(
        // The controller that the command will use
        new PIDController(0.000015, 0, 0),
        // This should return the measurement
        () -> wrist.getWristEncoder(),
        // This should return the setpoint (can also be a constant)
        () -> setpoint,
        // This uses the output
        output -> {
          wrist.setSpeed(output);
        });
      this.wrist = wrist;
    // Use addRequirements() here to declare subsystem dependencies.
    // Configure additional PID options by calling `getController` here.
   // getController().setTolerance(100); //Roughly 3.5ish degrees

  }

  @Override
  public void end(boolean interrupted){
    //wrist.setSpeed(-0.1);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return  false;

  }
}
