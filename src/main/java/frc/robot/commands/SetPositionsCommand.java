// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Telescope;
import frc.robot.subsystems.Wrist;

public class SetPositionsCommand extends CommandBase {
  /** Creates a new SetPositionsCommand. */
  private double armEncoder = 0, wristEncoder = 0, telescopeEncoder = 0;
  private final boolean isAuto;

  private final Arm arm;
  private final Wrist wrist;
  private final Telescope telescope;
  private final Intake intake;

  public SetPositionsCommand(Arm arm, Wrist wrist, Telescope telescope, Intake intake, double armEncoder, double wristEncoder, double telescopeEncoder, final boolean isAuto) {
    this.arm = arm;
    this.wrist = wrist;
    this.telescope = telescope;
    this.intake = intake;

    this.armEncoder = armEncoder;
    this.wristEncoder = wristEncoder;
    this.telescopeEncoder = telescopeEncoder;

    this.isAuto = isAuto;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  /**
   * Defaults to rest mode.
   * @param arm
   * @param wrist
   * @param telescope
   * @param intake
   */
  public SetPositionsCommand(Arm arm, Wrist wrist, Telescope telescope, Intake intake){
    this.arm = arm;
    this.wrist = wrist;
    this.telescope = telescope;
    this.intake = intake;

    this.isAuto = false;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if(armEncoder == 0 && wristEncoder == 0 && telescopeEncoder == 0){
      new SequentialCommandGroup(
        new ParallelCommandGroup(
          new RetractTelescopeUntilLimit(telescope),
          new RetractWristUntilLimit(wrist)
        ),

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
    else {
      new SequentialCommandGroup(
        new ArmToAngleGroup(arm, armEncoder),
        new ParallelCommandGroup(
          wrist.moveWristToPosition_Command(wrist.kWristDropPosition),
          telescope.extendArmToDistance_Command(telescopeEncoder)
        ).withTimeout(1.5),

        new WaitCommand(0.75),
        new InstantCommand((isAuto) ? () -> intake.setIntake(-0.2) : () -> intake.setIntake(0)),

        new WaitCommand(0.5),
        new InstantCommand(() -> intake.setIntake(0)),
        new InstantCommand(() -> intake.setIntake(Value.kForward))
      );
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true; //FIXME ? 
  }
}
