// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Telescope;
import frc.robot.subsystems.Wrist;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class scoreMid extends SequentialCommandGroup {
  /** Creates a new scoreMid. */
  public scoreMid(Wrist wrist, ArmSubsystem arm, Telescope telescope, Intake intake) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(

    new ArmToAngleGroup(arm, 275),
    new ParallelCommandGroup(
      wrist.moveWristToPosition_Command(wrist.kWristDropPosition),
      telescope.extendArmToDistance_Command(0)
    ).withTimeout(1.5),

    new InstantCommand(intake::toggleSolenoid),

    new WaitCommand(0.3)

    );
  }
}