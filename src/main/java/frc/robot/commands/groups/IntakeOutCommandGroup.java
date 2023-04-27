// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.groups;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
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
public class IntakeOutCommandGroup extends SequentialCommandGroup {
  /** Creates a new RestMode_CommandGroup. */
  public IntakeOutCommandGroup(Wrist wrist, Arm arm, Telescope telescope, Intake intake, final boolean isAuto) {
    addCommands(
      //1. 
      new ParallelDeadlineGroup(
        new ArmToAngleGroup(arm, 62),

        wrist.moveWristToPosition_Command(-1000).withTimeout(1),
        telescope.extendArmToDistance_Command(-1000).withTimeout(1)
      ).withTimeout(1.25),

      //2. 
      new ParallelCommandGroup(
        
        //0.0252 pid wrist
        wrist.moveWristToPosition_Command(30400 + 500),
        telescope.extendArmToDistance_Command(46472)
          // We Must be sure that 0 for wrist is top and limit engaged
          // And 0 for arm is retracted and limit engaged
          //The phases must be RIGHT
      ).withTimeout(1.5), //was 0.6

      new InstantCommand(() -> wrist.setHoldPosition(30900)),


      //3. 


      // new ParallelCommandGroup(
      //   new ArmToAngleGroup(arm, 61.75),
      //   wrist.moveWristToPosition_Command(30900),
      //   telescope.extendArmToDistance_Command(46472),
      //   new InstantCommand(() -> wrist.setHoldPosition(30900))
      // ).withTimeout(0.6), //do we need this? 

      //4.
      (isAuto) ? new InstantCommand(()-> intake.setIntake(0.65)) : new InstantCommand(() -> intake.setIntake(0))
    );
  }
}
