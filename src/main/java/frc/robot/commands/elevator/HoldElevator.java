/*                                  Team 334                                  */
/*               Copyright (c) 2024 Team 334. All Rights Reserved.            */

package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;

public class HoldElevator extends Command {
  private final ElevatorSubsystem _elevator;

  /** Creates a new HoldElevator. */
  public HoldElevator(ElevatorSubsystem elevator) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(elevator);
    _elevator = elevator;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    _elevator.setElevatorHeight(0); // holds the elevator in place at the bottom
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
