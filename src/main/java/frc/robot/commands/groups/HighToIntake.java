// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.groups;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Telescope;
import frc.robot.subsystems.Wrist;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class HighToIntake extends SequentialCommandGroup {
  private double armEncoder = 0, wristEncoder = 0, telescopeEncoder = 0;

  /** Creates a new RestMode_CommandGroup. */
  public HighToIntake(Wrist wrist, Arm arm, Telescope telescope, Intake intake, final boolean isAuto) {
    addCommands(
      //1. 
      new ParallelCommandGroup(
        telescope.extendArmToDistance_Command(46472).until(()->telescope.getEncoder() > 46472 -1000),
        wrist.moveWristToPosition_Command(20900)
      ).withTimeout(2)
      ,
      new InstantCommand(() -> wrist.setHoldPosition(30900)),

      //2.
      new ArmToAngleGroup(arm, 62),



  
      //3.
      (true) ? new InstantCommand(()-> intake.setIntake(0.65)) : new InstantCommand(() -> intake.setIntake(0))
    );
  }
}
