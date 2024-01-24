// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.elevator;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.PID;
import frc.robot.subsystems.ElevatorSubsystem;

public class SetElevator extends Command {
  /**
   * Creates a new SetElevator.
   * 
   * @param height The double supplier that returns the desired height of the elevator in meters.
   */
  private final ElevatorSubsystem _elevator;
  private final DoubleSupplier _height;
  private final PIDController _heightController = new PIDController(Constants.PID.ELEVATOR_KP, 0, 0);

  public SetElevator(ElevatorSubsystem elevator, DoubleSupplier height) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(elevator);
    _elevator = elevator;
    _height = height;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    _elevator.setMotor(_heightController.calculate(
      _elevator.getCurrentHeight(), _height.getAsDouble()
    ) + _elevator.elevatorFeed.calculate(0));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    _elevator.setMotor(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return _heightController.atSetpoint();
  }
}
