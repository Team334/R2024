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
  private ShooterSubsystem _shooter;
  private DoubleSupplier _angle;

  private boolean _runOnce;

 /**
   * Creates a new SetShooter.
   *
   * @param angle The double supplier that returns the desired angle of the shooter in degrees.
   * @param runOnce Used to run this command for changing a setpoint.
   */
  public SetShooter(ShooterSubsystem shooter, DoubleSupplier angle, boolean runOnce) {
    _shooter = shooter;
    _angle = angle;

    _runOnce = runOnce;

    addRequirements(_shooter);
  } 

  /** Creates a new SetShooter that runs once. */
  public SetShooter(ShooterSubsystem shooter, DoubleSupplier angle) {
    // Use addRequirements() here to declare subsystem dependencies.
    this(shooter, angle, true);
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
    _shooter.stopAngle();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return _runOnce && _shooter.atDesiredAngle();
    // return true;
  }
}
