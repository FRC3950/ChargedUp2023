// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.groups;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.commands.CloseIntakeCommand;
import frc.robot.commands.RetractTelescopeUntilLimit;
import frc.robot.commands.RetractWristUntilLimit;
import frc.robot.subsystems.*;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SetPositionsCommandGroup extends SequentialCommandGroup {
  /** Creates a new SetPositionsCommand. */
  private double armEncoder = 0, wristEncoder = 0, telescopeEncoder = 0;
  private final boolean isAuto;

  private final Arm arm;
  private final Wrist wrist; 
  private final Telescope telescope;
  private final Intake intake;

  // double telescopeEncoderValue;


  public SetPositionsCommandGroup(Arm arm, Wrist wrist, Telescope telescope, Intake intake, double armEncoder, double wristEncoder, double telescopeEncoder, final boolean isAuto) {
    this.arm = arm;
    this.wrist = wrist;
    this.telescope = telescope;
    this.intake = intake; 

    this.isAuto = isAuto;

 
    addCommands(
  //1. Arm to Angle 
      new ArmToAngleGroup(arm, armEncoder).andThen(new PrintCommand("Finished 1")),

  //2. Telescope to Extenstion & wrist to position
      new ParallelCommandGroup(
        telescope.extendArmToDistance_Command(telescopeEncoder).until(
          () -> telescope.getEncoder() > telescopeEncoder - 999 && telescope.getEncoder() < telescopeEncoder + 999 //was 900 
          ),
        wrist.moveWristToPosition_Command(wristEncoder).until(
          () -> wrist.getWristEncoder() > wristEncoder -600 && wrist.getWristEncoder() < wristEncoder + 600
        ).withTimeout(1.6) //was 2.0
      ).withTimeout(2.2).andThen(new PrintCommand("Finished 2")), //was 2.5

  //3. Wrist Holds positions
      new InstantCommand(() -> wrist.setHoldPosition(wristEncoder)).andThen(new PrintCommand("Finished 3")),

  //4. Auto 
      (isAuto) ? 
      new InstantCommand(
        () -> intake.setIntake(-0.2))
        .andThen(new WaitCommand(0.4)) //was 0.5
        .andThen( new InstantCommand(() -> intake.setIntake(0))) 
        : new InstantCommand(() -> {})
    );
  }
  /**
   * Defaults to rest mode.
   */
  public SetPositionsCommandGroup(Arm arm, Wrist wrist, Telescope telescope, Intake intake){
    this.arm = arm;
    this.wrist = wrist;
    this.telescope = telescope;
    this.intake = intake; 

    this.isAuto = false;

    addCommands(

    //1. Intake Closes If Extended Far Enough
      new CloseIntakeCommand(intake, telescope),

    //2. Wrist and Tele retract until limits
      new ParallelCommandGroup(
        new RetractWristUntilLimit(wrist),
        new RetractTelescopeUntilLimit(telescope)
      ),

      //3. 
      new ParallelDeadlineGroup(
        new ArmToAngleGroup(arm, -8).until(arm::isLimitSwithEngaged),
        new RunCommand(() -> wrist.setSpeed(-.25), wrist),
        new RetractTelescopeUntilLimit(telescope)
      ).withTimeout(1.25),

      new InstantCommand(() -> wrist.setHoldPosition(0)),

      new ParallelCommandGroup(
   
        new InstantCommand(() -> wrist.setSpeed(0)),
        new InstantCommand(() -> telescope.setPercent(0))
      )
    );
  }
}
