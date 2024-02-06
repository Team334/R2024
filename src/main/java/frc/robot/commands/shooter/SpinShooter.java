/* Copyright (C) 2024 Team 334. All Rights Reserved.*/
package frc.robot.commands.shooter;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

/**
 * @author Elvis Osmanov
 * @author Cherine Soewingjo
 * @author Peleh Liu
 */
public class SpinShooter extends Command {
  private ShooterSubsystem _shooter;
  private DoubleSupplier _speed;

  public SpinShooter(ShooterSubsystem shooter, DoubleSupplier speed) {
    _shooter = shooter;
    _speed = speed;
    addRequirements(_shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    _shooter.spinShooter(_speed.getAsDouble());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    _shooter.stopShooter();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
