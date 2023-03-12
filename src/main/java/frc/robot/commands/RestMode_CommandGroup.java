// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.Telescope;
import frc.robot.subsystems.Wrist;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class RestMode_CommandGroup extends SequentialCommandGroup {
  /** Creates a new RestMode_CommandGroup. */
  public RestMode_CommandGroup(Wrist wrist, ArmSubsystem arm, Telescope telescope) {
    addCommands(

//1. 
      new ParallelCommandGroup(
        wrist.moveWristToPosition_Command(0),
        telescope.extendArmToDistance_Command(0)

        // We Must be sure that 0 for wrist is top and limit engaged
        // And 0 for arm is retracted and limit engaged
        //The phases must be RIGHT

        ).withTimeout(3),

//2. 
        new ParallelCommandGroup(
          new ArmToAngleGroup(null, 0),
          wrist.moveWristToPosition_Command(0),
          telescope.extendArmToDistance_Command(0)

        ).withTimeout(3)
    );
  }
}