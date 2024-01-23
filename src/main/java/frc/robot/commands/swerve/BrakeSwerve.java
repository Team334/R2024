/*                                  Team 334                                  */
/* Copyright (c) 2024 Team 334. All Rights Reserved.                          */

package frc.robot.commands.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveDriveSubsystem;

/**
 * Creates a natural brake on the swerve drive by facing the modules so they form an "X" shape.
 *
 * @author Peter Gutkovich
 */
public class BrakeSwerve extends Command {
  private final SwerveDriveSubsystem _swerveDrive;

  private double _timeout = 0;
  private Timer _timer = new Timer();

  /** Creates a new BrakeSwerve. */
  public BrakeSwerve(SwerveDriveSubsystem swerveDrive) {
    _swerveDrive = swerveDrive;

    addRequirements(swerveDrive);
  }

  /**
   * Creates a new BrakeSwerve.
   *
   * @param timeout - (in seconds) Will keep the drive in brake position for this amount of time (must be >0).
   */
  public BrakeSwerve(SwerveDriveSubsystem swerveDrive, double timeout) {
    _swerveDrive = swerveDrive;
    _timeout = timeout;

    addRequirements(swerveDrive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    _timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SwerveModuleState[] states = {
      new SwerveModuleState(0, Rotation2d.fromDegrees(45)),
      new SwerveModuleState(0, Rotation2d.fromDegrees(-45)),
      new SwerveModuleState(0, Rotation2d.fromDegrees(45)),
      new SwerveModuleState(0, Rotation2d.fromDegrees(-45))
    };

    _swerveDrive.setStates(states);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    _timer.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (_timeout != 0) {
      return _timer.hasElapsed(_timeout);
    } else {
      return false;
    }
  }
}
