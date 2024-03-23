// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.swerve;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.PID;
import frc.robot.Constants.Speeds;
import frc.robot.subsystems.SwerveDriveSubsystem;

public class SetHeading extends Command {
  private SwerveDriveSubsystem _swerve;
  private DoubleSupplier _heading;
  private boolean _runOnce;

  private DoubleSupplier _xSpeed;
  private DoubleSupplier _ySpeed;

  private PIDController _headingController = new PIDController(PID.SWERVE_HEADING_KP, 0, PID.SWERVE_HEADING_KD);

  /**
   * Creates a new SetHeading.
   *
   * @param heading The double supplier that returns the desired heading of the chassis in degrees.
   * @param xSpeed X drive joystick speed.
   * @param ySpeed Y drive joystick speed.
   * @param runOnce Used to run this command for a changing setpoint.
   */
  public SetHeading(
    SwerveDriveSubsystem swerve, 
    DoubleSupplier xSpeed,
    DoubleSupplier ySpeed, 
    DoubleSupplier heading, 
    boolean runOnce
  ) {
    // Use addRequirements() here to declare subsystem dependencies.
    _swerve = swerve;
    _heading = heading;
    _runOnce = runOnce;

    _xSpeed = xSpeed;
    _ySpeed = ySpeed;

    _headingController.setTolerance(3);
    _headingController.enableContinuousInput(-180, 180);

    addRequirements(_swerve);
  }

  /** Creates a new SetHeading that runs once. */
  public SetHeading(
    SwerveDriveSubsystem swerve,
    DoubleSupplier xSpeed,
    DoubleSupplier ySpeed,
    DoubleSupplier heading
  ) {
    this(swerve, xSpeed, ySpeed, heading, true);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.putData(_headingController);

    double rotationVelocity = MathUtil.clamp(
      _headingController.calculate(_swerve.getHeading().getDegrees(), _heading.getAsDouble()),
      -Speeds.SWERVE_DRIVE_MAX_ANGULAR_SPEED,
      Speeds.SWERVE_DRIVE_MAX_ANGULAR_SPEED
    );

    _swerve.driveChassis(new ChassisSpeeds(
      _xSpeed.getAsDouble() * Speeds.SWERVE_DRIVE_MAX_SPEED * _swerve.getDriveCoeff(),
      _ySpeed.getAsDouble() * Speeds.SWERVE_DRIVE_MAX_SPEED * _swerve.getDriveCoeff(),
      rotationVelocity
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
    return _runOnce && _headingController.atSetpoint();
  }
}
