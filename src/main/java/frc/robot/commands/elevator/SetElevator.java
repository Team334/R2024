/* Copyright (C) 2024 Team 334. All Rights Reserved.*/
package frc.robot.commands.elevator;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;

public class SetElevator extends Command {
  /**
   * Creates a new SetElevator.
   *
   * @param height
   *            The double supplier that returns the desired height of the
   *            elevator in meters.
   */
  private final ElevatorSubsystem _elevator;

  private final DoubleSupplier _height;

  public SetElevator(ElevatorSubsystem elevator, DoubleSupplier height) {
    // Use addRequirements() here to declare subsystem dependencies.
    _elevator = elevator;
    _height = height;

    addRequirements(_elevator);
  }

  /** Set elevator that sets the elevator to its lowest height. */
  public SetElevator(ElevatorSubsystem elevator) {
    this(elevator, () -> 0);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    _elevator.setElevatorHeight(_height.getAsDouble());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    _elevator.stopElevator();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return _elevator.atDesiredHeight();
  }
}
