/*                                  Team 334                                  */
/* Copyright (c) 2024 Team 334. All Rights Reserved.                          */

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;
import java.util.function.DoubleSupplier;

/**
 * @author Elvis Osmanov
 * @author Harry Chen
 */
public class AngleShooter extends Command {
  /** Creates a new AngleShooter. */
  private ShooterSubsystem _shooter;

  private DoubleSupplier _angle;

  public AngleShooter(ShooterSubsystem shooter, DoubleSupplier angle) {
    // Use addRequirements() here to declare subsystem dependencies.
    _shooter = shooter;
    _angle = angle;
    addRequirements(_shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    _shooter.setAngle(_angle.getAsDouble());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    _shooter.stopAngle();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return _shooter.atDesiredAngle();
  }
}
