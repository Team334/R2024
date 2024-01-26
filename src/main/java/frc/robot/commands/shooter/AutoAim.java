/*                                  Team 334                                  */
/* Copyright (c) 2024 Team 334. All Rights Reserved.                          */

package frc.robot.commands.shooter;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveDriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class AutoAim extends Command {
  private ShooterSubsystem _shooter;
  private VisionSubsystem _vision;
  private SwerveDriveSubsystem _swerve;

  private DoubleSupplier _xSpeed;
  private DoubleSupplier _ySpeed;

  private PIDController _headingController = new PIDController(
    Constants.PID.PP_ROTATION.kP,
    0,
    0
  );

  /** Creates a new AutoAim. */
  public AutoAim(
    ShooterSubsystem shooter, 
    VisionSubsystem vision, 
    SwerveDriveSubsystem swerve,
    DoubleSupplier xSpeed,
    DoubleSupplier ySpeed
  ) {
    // Use addRequirements() here to declare subsystem dependencies.
    _shooter = shooter;
    _vision = vision;
    _swerve = swerve;

    _xSpeed = xSpeed;
    _ySpeed = ySpeed;

    _headingController.setTolerance(2);

    addRequirements(_shooter, _vision, _swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double desiredShooterAngle = 0;
    double desiredSwerveHeading = 0;

    double currentSwerveHeading = _swerve.getHeading().getDegrees();

    if (_vision.isApriltagVisible()) {
      desiredShooterAngle = _vision.anglesToSpeaker()[1]; // ty
      desiredSwerveHeading = _vision.anglesToSpeaker()[0];

    } else {
      desiredShooterAngle = _swerve.anglesToSpeaker()[1];
      desiredSwerveHeading = _swerve.anglesToSpeaker()[0];
    }

    desiredSwerveHeading += currentSwerveHeading;

    _shooter.setAngle(desiredShooterAngle);

    _swerve.driveChassis(
      new ChassisSpeeds(
        _xSpeed.getAsDouble() * Constants.Speeds.SWERVE_DRIVE_MAX_SPEED * Constants.Speeds.SWERVE_DRIVE_COEFF,
        _ySpeed.getAsDouble() * Constants.Speeds.SWERVE_DRIVE_MAX_SPEED * Constants.Speeds.SWERVE_DRIVE_COEFF,
        _headingController.calculate(currentSwerveHeading, desiredSwerveHeading)
      )
    );
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
