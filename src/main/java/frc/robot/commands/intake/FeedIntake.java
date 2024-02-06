/* Copyright (C) 2024 Team 334. All Rights Reserved.*/
package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.IntakeSubsystem.ActuatorState;
import frc.robot.subsystems.IntakeSubsystem.FeedMode;

public class FeedIntake extends Command {
  private final IntakeSubsystem _intake;

  private final ActuatorState _actuatorState;
  private final FeedMode _feedMode;

  private boolean _runOnce; // TODO: do we need this?

  /** Creates a new FeedIntake. */
  public FeedIntake(IntakeSubsystem intake, ActuatorState actuatorState, FeedMode feedMode) {
    _intake = intake;

    _actuatorState = actuatorState;
    _feedMode = feedMode;
    _runOnce = false;

    addRequirements(_intake);
  }

  /** FeedIntake as a hold command (runs forever). */
  public FeedIntake(IntakeSubsystem intake, ActuatorState actuatorState) {
    this(intake, actuatorState, FeedMode.NONE);

    _runOnce = false;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // _intake.feed(_feedMode);
    _intake._feedMotor.set(0.3);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    _intake.actuate(_actuatorState);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // _intake.feed(FeedMode.NONE);
    _intake._feedMotor.set(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // return _runOnce && _intake.atDesiredActuatorState();
    return false;
  }
}
