/* Copyright (C) 2024 Team 334. All Rights Reserved.*/
package frc.robot.commands.swerve;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveDriveSubsystem;

/**
 * Drive the swerve chassis based on teleop joystick input
 *
 * @author Peter Gutkovich
 * @author Elvis Osmanov
 */
public class TeleopDrive extends Command {
  private final SwerveDriveSubsystem _swerveDrive;

  private final DoubleSupplier _xSpeed;
  private final DoubleSupplier _ySpeed;

  private final DoubleSupplier _rotationSpeed;

  private final double _driveCoeff;

  /** Creates a new TeleopDrive. */
  public TeleopDrive(SwerveDriveSubsystem swerveDrive, DoubleSupplier xSpeed, DoubleSupplier ySpeed,
      DoubleSupplier rotationSpeed, double swerveDriveFastCoeff) {

    _swerveDrive = swerveDrive;

    _xSpeed = xSpeed;
    _ySpeed = ySpeed;

    _rotationSpeed = rotationSpeed;

    _driveCoeff = swerveDriveFastCoeff;

    // Use addRequirements() here to declare subsystem dependencies.\
    addRequirements(swerveDrive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // apply controller deadband
    // drive the swerve chassis subsystem
    _swerveDrive.driveChassis(new ChassisSpeeds(
        _xSpeed.getAsDouble() * Constants.Speeds.SWERVE_DRIVE_MAX_SPEED * _driveCoeff,
        _ySpeed.getAsDouble() * Constants.Speeds.SWERVE_DRIVE_MAX_SPEED * _driveCoeff,
        _rotationSpeed.getAsDouble() * Constants.Speeds.SWERVE_DRIVE_MAX_ANGULAR_SPEED));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
