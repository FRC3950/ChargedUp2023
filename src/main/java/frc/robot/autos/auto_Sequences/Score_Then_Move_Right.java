// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos.auto_Sequences;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.autos.runPathAuto;
import frc.robot.commands.RestMode_CommandGroup;
import frc.robot.commands.scoreMid;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Telescope;
import frc.robot.subsystems.Wrist;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Score_Then_Move_Right extends SequentialCommandGroup {
  /** Creates a new Score_Then_Move_Right. */
  public Score_Then_Move_Right(Wrist wrist, ArmSubsystem arm, Telescope telescope, Intake intake, Swerve swerve) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(

    new scoreMid(wrist, arm, telescope, intake),
    new WaitCommand(1),
    new RestMode_CommandGroup(wrist, arm, telescope),
    new WaitCommand(.5),
    new runPathAuto(swerve, Constants.PathPlannerSimpleTrajectories.advanceNorth_22inches)
    );
  }
}
