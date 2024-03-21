/* Copyright (C) 2024 Team 334. All Rights Reserved.*/
package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.IntakeSubsystem.ActuatorState;
import frc.robot.subsystems.IntakeSubsystem.FeedMode;

public class FeedActuate extends Command {
  private final IntakeSubsystem _intake;

  private final ActuatorState _actuatorState;
  private final FeedMode _feedMode;

  /** Creates a new FeedActuate. */
  public FeedActuate(IntakeSubsystem intake, ActuatorState actuatorState, FeedMode feedMode) {
    _intake = intake;

    _actuatorState = actuatorState;
    _feedMode = feedMode;

    addRequirements(_intake);
  }

  /** FeedActuate to only control actuator state. */
  public FeedActuate(IntakeSubsystem intake, ActuatorState actuatorState) {
    this(intake, actuatorState, FeedMode.NONE);
  }

  /** FeedActuate to only control feed mode. */
  public FeedActuate(IntakeSubsystem intake, FeedMode feedMode) {
    this(intake, ActuatorState.NONE, feedMode);
  }

  public FeedActuate(IntakeSubsystem intake) {
    this(intake, FeedMode.NONE);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    _intake.feed(_feedMode);
    _intake.actuate(_actuatorState);

    if (_feedMode == FeedMode.OUTTAKE) _intake.resetHasNote();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    _intake.actuate(_actuatorState);

    // if (_feedMode == FeedMode.INTAKE && _intake.hasNote()) _intake.feed(FeedMode.NONE);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    _intake.actuate(ActuatorState.NONE);
    _intake.feed(FeedMode.NONE);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
