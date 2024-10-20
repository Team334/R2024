/* Copyright (C) 2024 Team 334. All Rights Reserved.*/
package frc.robot.commands.elevator;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;

public class SetElevator extends Command {
  private final ElevatorSubsystem _elevator;
  private final DoubleSupplier _height;

  private boolean _runOnce;

  /**
   * Creates a new SetElevator.
   *
   * @param height The double supplier that returns the desired height of the elevator in meters.
   * @param runOnce Used to run this command for a changing setpoint.
   */
  public SetElevator(ElevatorSubsystem elevator, DoubleSupplier height, boolean runOnce) {
    _elevator = elevator;
    _height = height;

    _runOnce = runOnce;

    addRequirements(_elevator);
  } 

  /** Creates a new SetElevator that runs once. */
  public SetElevator(ElevatorSubsystem elevator, DoubleSupplier height) {
    // Use addRequirements() here to declare subsystem dependencies.
    this(elevator, height, true);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    _elevator.setHeight(_height.getAsDouble());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    _elevator.stopElevator();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return _runOnce && _elevator.atDesiredHeight();
    // return true;
  }
}
