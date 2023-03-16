// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos.auto_Sequences;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.autos.runPathAuto;
import frc.robot.commands.IntakeOutCommandGroup;
import frc.robot.commands.RestModeCommandGroup;
import frc.robot.commands.ScoreHighCommandGroup;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Telescope;
import frc.robot.subsystems.Wrist;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Auto2Cone extends SequentialCommandGroup {
  /** Creates a new ScoreHigh_Move_ScoreHigh. */
  public Auto2Cone(Wrist wrist, Arm arm, Telescope telescope, Intake intake, Swerve swerve) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ScoreHighCommandGroup(wrist, arm, telescope, intake),
      new WaitCommand(.25),
      // new RestModeCommandGroup(wrist, arm, telescope),
      // new WaitCommand(.5),

      new ParallelCommandGroup(
        //1.
        new runPathAuto(
          swerve, 
          PathPlanner.loadPath("Auto2ConeForward", new PathConstraints(3, 3))
        ),
        //2.
        new IntakeOutCommandGroup(wrist, arm, telescope, intake).withTimeout(2)
        //The idea behind this is that we can go from high directly to intake because the robot will be moving
      )    
    );

    
  }
}
