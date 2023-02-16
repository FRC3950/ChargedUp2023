// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants;
import frc.robot.subsystems.Telescope;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TelescopePIDCommand extends PIDCommand {
  /** Creates a new TelescopePIDCommand. */
  public TelescopePIDCommand(Telescope s_Wrist) {
    super(
        // The controller that the command will use
        new PIDController(0.35, 0, 0), //FIXME
        // This should return the measurement
        s_Wrist::getEncoder,
        // This should return the setpoint (can also be a constant)
        () -> Constants.kTelescope.encoderLimit,
        // This uses the output
        output -> {
          s_Wrist.setMotor(output);
        });
    // Use addRequirements() here to declare subsystem dependencies.
    // Configure additional PID options by calling `getController` here.
    getController().setTolerance(2000);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return getController().atSetpoint();
  }
}
