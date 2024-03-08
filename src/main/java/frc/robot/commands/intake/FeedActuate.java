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

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    _intake.feed(_feedMode);
    _intake.actuate(_actuatorState);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    _intake.actuate(_actuatorState);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
