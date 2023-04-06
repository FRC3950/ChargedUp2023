// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.groups;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Arm;
import frc.robot.commands.*;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ArmToAngleGroup extends SequentialCommandGroup {

  /** Creates a new armtoanglegroup. */
  public ArmToAngleGroup(Arm arm, double angle) {
   

    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new InstantCommand(arm::unlockArm),
      new ArmToAnglePID(arm, angle).withTimeout(2.4),
      new InstantCommand(() -> arm.setMotors(0)),
      new WaitCommand(0.2),
      new InstantCommand(arm::lockArm)
    ); 
  }
}
