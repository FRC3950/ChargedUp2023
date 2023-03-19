// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos.AutoSequences;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.autos.runPathAuto;
import frc.robot.commands.IntakeOutCommandGroup;
import frc.robot.commands.RestModeCommandGroup;
import frc.robot.commands.ScoreHighCommandGroup;
import frc.robot.commands.ScoreMidCommandGroup;
import frc.robot.subsystems.*;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Corner2ConeAuto extends SequentialCommandGroup {
  /** Creates a new Corner2ConeAuto. */
  public Corner2ConeAuto(Wrist wrist, Arm arm, Telescope telescope, Intake intake, Swerve swerve) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ScoreHighCommandGroup(wrist, arm, telescope, intake),
      new WaitCommand(0.25),
      new ParallelCommandGroup(
        new RestModeCommandGroup(wrist, arm, telescope),
        new runPathAuto(swerve, PathPlanner.loadPath("Corner2Cone_1", new PathConstraints(3, 3)))
      ),
      new runPathAuto(swerve, PathPlanner.loadPath("Corner2Cone_2", new PathConstraints(3, 2.5))),
      new ParallelCommandGroup(
        new IntakeOutCommandGroup(wrist, arm, telescope, intake),
        new runPathAuto(swerve, PathPlanner.loadPath("Corner2Cone_3", new PathConstraints(1.5, 2.5)))
      ),
      new ParallelCommandGroup(
        new RestModeCommandGroup(wrist, arm, telescope), //consider running this in parallel w path no. 4
        new runPathAuto(swerve, PathPlanner.loadPath("Corner2Cone_4", new PathConstraints(3, 3)))
      ),
      new ScoreMidCommandGroup(wrist, arm, telescope, intake)
    );
  }
}
