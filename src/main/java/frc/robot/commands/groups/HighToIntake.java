// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.groups;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
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
  /** Creates a new RestMode_CommandGroup. */
  public HighToIntake(Wrist wrist, Arm arm, Telescope telescope, Intake intake, final boolean isAuto) {
    addCommands(
      //1. 
      new ParallelCommandGroup(
        wrist.moveWristToPosition_Command(31500),
        telescope.extendArmToDistance_Command(61500)
      ).withTimeout(2),

      //2. 
      new ArmToAngleGroup(arm, 62),
    
      //3.
      (isAuto) ?
       new InstantCommand(()-> intake.setIntake(0.65)) :
        new InstantCommand(() -> intake.setIntake(0))
    );
  }
}
