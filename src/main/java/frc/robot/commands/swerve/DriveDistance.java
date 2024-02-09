// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.swerve;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.Speeds;
import frc.robot.subsystems.SwerveDriveSubsystem;

/** In case path planner fails. */
public class DriveDistance extends Command {
  private final SwerveDriveSubsystem _swerve;

  private final PIDController _driveController = new PIDController(0, 0, 0);
  private final double _distance;

  private double _endX;

  /** Creates a new DriveDistance. */
  public DriveDistance(SwerveDriveSubsystem swerve, double distance) {
    // Use addRequirements() here to declare subsystem dependencies.
    _swerve = swerve;
    _distance = distance;

    _driveController.setTolerance(0.1);

    addRequirements(_swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    _endX = _swerve.getPose().getX() + _distance;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    _swerve.driveChassis(new ChassisSpeeds(
      MathUtil.clamp(_driveController.calculate(_swerve.getPose().getX(), _endX), -Speeds.SWERVE_DRIVE_MAX_SPEED, Speeds.SWERVE_DRIVE_MAX_SPEED),
      0,
      0
    ));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    _swerve.driveChassis(new ChassisSpeeds());
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return _driveController.atSetpoint();
  }
}
