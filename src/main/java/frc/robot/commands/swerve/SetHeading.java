// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.swerve;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.Speeds;
import frc.robot.subsystems.SwerveDriveSubsystem;

public class SetHeading extends Command {
  private SwerveDriveSubsystem _swerve;
  private DoubleSupplier _heading;
  private boolean _runOnce;

  private DoubleSupplier _xSpeed;
  private DoubleSupplier _ySpeed;

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
    _swerve.setHeading(
      _xSpeed.getAsDouble() * Speeds.SWERVE_DRIVE_MAX_SPEED * _swerve.getDriveCoeff(),
      _ySpeed.getAsDouble() * Speeds.SWERVE_DRIVE_MAX_SPEED * _swerve.getDriveCoeff(),
      _heading.getAsDouble()
    );
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("AIMED");
    _swerve.driveChassis(new ChassisSpeeds());
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return _runOnce && _swerve.atDesiredHeading();
  }
}
