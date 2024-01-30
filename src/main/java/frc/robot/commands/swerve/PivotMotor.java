/*                                  Team 334                                  */
/* Copyright (c) 2024 Team 334. All Rights Reserved.                          */

package frc.robot.commands.swerve;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveDriveSubsystem;

public class PivotMotor extends Command {
  private final SwerveDriveSubsystem _swerveDrive;
  private boolean _left;

  private DoubleSupplier _xSpeed;
  private DoubleSupplier _ySpeed;
  private DoubleSupplier _rotationSpeed;

  private Translation2d _pivotPoint = new Translation2d(0, 0);

  /** Creates a new PivotMotor. */
  public PivotMotor(SwerveDriveSubsystem swerveDrive, boolean left, DoubleSupplier xSpeed, DoubleSupplier ySpeed,
      DoubleSupplier rotationSpeed) {
    // Use addRequirements() here to declare subsystem dependencies.
    _swerveDrive = swerveDrive;
    _left = left;

    _xSpeed = xSpeed;
    _ySpeed = ySpeed;
    _rotationSpeed = rotationSpeed;

    addRequirements(swerveDrive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    double currentRotation = _swerveDrive.getHeading().getDegrees();

    Translation2d _frontLeft = new Translation2d(0.292, 0.292);
    Translation2d _frontRight = new Translation2d(0.292, -0.292);
    Translation2d _backRight = new Translation2d(-0.292, -0.292);
    Translation2d _backLeft = new Translation2d(-0.292, 0.292);

    // If we are backwards
    // if (_swerveDrive._driveController.getLeftY() < 0){
    //   currentRotation += 180;
    //   _left = !_left;
    // }

    // NEW - JERRY
    int rotationOffset = _left ? 0 : 180;
    int quadrant = ((int) currentRotation + rotationOffset) / 90 % 4;

    switch (quadrant) {
        case 0:
            _pivotPoint = _left ? _frontLeft : _frontRight;
            break;
        case 1:
            _pivotPoint = _left ? _frontRight : _backRight;
            break;
        case 2:
            _pivotPoint = _left ? _backRight : _backLeft;
            break;
        case 3:
            _pivotPoint = _left ? _backLeft : _frontLeft;
            break;
    }

    _swerveDrive.pivotMotor(_pivotPoint);

    // ORIGINAL
    //
    // if (_left){
    //   if (currentRotation >= 0 && currentRotation < 90){
    //     _pivotPoint = _frontLeft;
    //   }
    //   else if (currentRotation >= 90 && currentRotation < 180){
    //     _pivotPoint = _frontRight;
    //   }
    //   else if (currentRotation >= 180 && currentRotation < 270){
    //     _pivotPoint = _backRight;
    //   }
    //   else{
    //     _pivotPoint = _backLeft;
    //   }
    // }
    // else{
    //   if (currentRotation >= 0 && currentRotation < 90){
    //     _pivotPoint = _frontRight;
    //   }
    //   else if (currentRotation >= 90 && currentRotation < 180){
    //     _pivotPoint = _backRight;
    //   }
    //   else if (currentRotation >= 180 && currentRotation < 270){
    //     _pivotPoint = _backLeft;
    //   }
    //   else{
    //     _pivotPoint = _frontLeft;
    //   }
    // }
    // _swerveDrive.pivotMotor(_pivotPoint);
    // }

    // Called every time the scheduler runs while the command is scheduled.
  }

  @Override
  public void execute() {
  _swerveDrive.driveChassis(
      new ChassisSpeeds(
        _xSpeed.getAsDouble() * Constants.Speeds.SWERVE_DRIVE_MAX_SPEED * Constants.Speeds.SWERVE_DRIVE_COEFF,
        _ySpeed.getAsDouble() * Constants.Speeds.SWERVE_DRIVE_MAX_SPEED * Constants.Speeds.SWERVE_DRIVE_COEFF,
        _rotationSpeed.getAsDouble() * Constants.Speeds.SWERVE_DRIVE_MAX_ANGULAR_SPEED
      )
    );
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    _swerveDrive.resetPivot();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
