/* Copyright (C) 2024 Team 334. All Rights Reserved.*/
package frc.robot.commands.shooter;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

/**
 * @author Elvis Osmanov
 * @author Harry Chen
 * @author Peter Gutkovich
 */
public class SetShooter extends Command {
  /** Creates a new AngleShooter. */
  private ShooterSubsystem _shooter;

  private DoubleSupplier _angle;

  public SetShooter(ShooterSubsystem shooter, DoubleSupplier angle) {
    // Use addRequirements() here to declare subsystem dependencies.
    _shooter = shooter;
    _angle = angle;
    addRequirements(_shooter);
  }
  
  /** Sets the shooter to its lowest angle (flat). */
  public SetShooter(ShooterSubsystem shooter) {
    this(shooter, () -> 0);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    _shooter.setAngle(_angle.getAsDouble());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    _shooter.setAngle(_angle.getAsDouble());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("DONE");
    _shooter.stopAngle();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    System.out.println(_shooter.atDesiredAngle());
    return _shooter.atDesiredAngle();
  }
}
