/*                                  Team 334                                  */
/*               Copyright (c) 2024 Team 334. All Rights Reserved.            */

package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;
import java.util.function.DoubleSupplier;

public class SetElevator extends Command {
  /**
   * Creates a new SetElevator.
   *
   * @param height The double supplier that returns the desired height of the elevator in meters.
   */
  private final ElevatorSubsystem _elevator;

  private final DoubleSupplier _height;

  public SetElevator(ElevatorSubsystem elevator, DoubleSupplier height) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(elevator);
    _elevator = elevator;
    _height = height;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

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
