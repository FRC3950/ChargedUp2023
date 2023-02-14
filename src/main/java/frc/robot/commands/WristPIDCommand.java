// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants;
import frc.robot.subsystems.Wrist;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class WristPIDCommand extends PIDCommand {
  /** Creates a new WristPIDCommand. */
  public WristPIDCommand(Wrist s_Wrist) {
    super(
        // The controller that the command will use
        new PIDController(
        s_Wrist.getPIDDashboardConstants()[0], 
        s_Wrist.getPIDDashboardConstants()[1], 
        s_Wrist.getPIDDashboardConstants()[2]),
        // This should return the measurement
        s_Wrist::getWristEncoder,
        // This should return the setpoint (can also be a constant)
        () -> Constants.kIntake.encoderLimit, 
        // This uses the output
        output -> {
          s_Wrist.setSpeed(output + s_Wrist.getPIDDashboardConstants()[3]);
        }, s_Wrist);
    // Use addRequirements() here to declare subsystem dependencies.
    // Configure additional PID options by calling `getController` here.
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
