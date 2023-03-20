// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Telescope;
import frc.robot.subsystems.Wrist;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class RestModeCommandGroup extends SequentialCommandGroup {
  /** Creates a new RestMode_CommandGroup. */
  public RestModeCommandGroup(Wrist wrist, Arm arm, Telescope telescope) {
    addCommands(
//1. 
      new ParallelCommandGroup(
        new RetractWristUntilLimit(wrist),
        new RetractTelescopeUntilLimit(telescope)
          // We Must be sure that 0 for wrist is top and limit engaged
          // And 0 for arm is retracted and limit engaged
          //The phases must be RIGHT
      ),
//2. 
      new ParallelRaceGroup(
        new ArmToAngleGroup(arm, -11).until(arm::isLimitSwithEngaged),
        new RunCommand(() -> wrist.setSpeed(-.3), wrist)
      ).withTimeout(1.25),
      new ParallelCommandGroup(
        new InstantCommand(() -> wrist.setSpeed(0)),
        new InstantCommand(() -> telescope.setPercent(0))
      )
    );
  }
}
