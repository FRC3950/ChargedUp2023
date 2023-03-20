// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Telescope;
import frc.robot.subsystems.Wrist;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class IntakeStandingCommandGroup extends SequentialCommandGroup {
  /** Creates a new IntakeStandingCommandGroup. */
  public IntakeStandingCommandGroup(Wrist wrist, Arm arm, Telescope telescope, Intake intake) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ParallelCommandGroup(
        wrist.moveWristToPosition_Command(-1000).withTimeout(0.25),
        telescope.extendArmToDistance_Command(-1000).withTimeout(.25),
        new ArmToAngleGroup(arm, 120.5)
      ).withTimeout(2),

      new ParallelCommandGroup(
        wrist.moveWristToPosition_Command(48331)
      ),

      new RunCommand(() -> intake.setIntake(Value.kForward), intake)
    );
    addRequirements(wrist, arm, telescope);
  }
}
