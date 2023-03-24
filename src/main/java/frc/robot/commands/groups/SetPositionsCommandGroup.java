// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.groups;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.*;
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

  public SetPositionsCommandGroup(Arm arm, Wrist wrist, Telescope telescope, Intake intake, double armEncoder, double wristEncoder, double telescopeEncoder, final boolean isAuto) {
    this.arm = arm;
    this.wrist = wrist;
    this.telescope = telescope;
    this.intake = intake; 

    this.isAuto = isAuto;
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ArmToAngleGroup(arm, armEncoder),
      new ParallelCommandGroup(
        wrist.moveWristToPosition_Command(wristEncoder),
        telescope.extendArmToDistance_Command(telescopeEncoder)
      ).withTimeout(2),

      (isAuto) ? new InstantCommand(() -> intake.setIntake(-0.2)) : new InstantCommand(() -> intake.setIntake(0)),

      new WaitCommand(0.5),
      new InstantCommand(() -> intake.setIntake(0)),
      new InstantCommand(() -> intake.setIntake(Value.kForward))
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
      new ParallelCommandGroup(
        new RetractWristUntilLimit(wrist),
        new RetractTelescopeUntilLimit(telescope)
      ),

      new ParallelCommandGroup(
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
