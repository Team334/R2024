// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

public class OperateShooter extends Command {
  private final ShooterSubsystem _shooter;

  private final DoubleSupplier _angleSpeed;
  private final DoubleSupplier _shooterSpeed;

  /** Creates a new OperateShooter. */
  public OperateShooter(ShooterSubsystem shooter, DoubleSupplier angleSpeed, DoubleSupplier shooterSpeed) {
    // Use addRequirements() here to declare subsystem dependencies.
    _angleSpeed = angleSpeed;
    _shooterSpeed = shooterSpeed;

    _shooter = shooter;
    addRequirements(_shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    _shooter.driveAngle(_angleSpeed.getAsDouble() * 0.1); // TODO: find speed coeff
    _shooter.spinShooter(_shooterSpeed.getAsDouble() * 0.8);
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
