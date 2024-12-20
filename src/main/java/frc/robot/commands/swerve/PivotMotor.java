/* Copyright (C) 2024 Team 334. All Rights Reserved.*/
package frc.robot.commands.swerve;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.SwerveDriveSubsystem;

public class PivotMotor extends Command {
  private final LEDSubsystem _leds;

  private final SwerveDriveSubsystem _swerveDrive;
  private boolean _left;

  private DoubleSupplier _forward;

  private Translation2d _pivotPoint = new Translation2d(0, 0);

  /** Creates a new PivotMotor. */
  public PivotMotor(LEDSubsystem leds, SwerveDriveSubsystem swerveDrive, boolean left, DoubleSupplier forward) {
    // Use addRequirements() here to declare subsystem dependencies.
    _leds = leds;

    _swerveDrive = swerveDrive;
    _left = left;

    _forward = forward;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    double currentRotation = MathUtil.inputModulus(_swerveDrive.getHeadingRaw().getDegrees(), 0, 360);

    Translation2d _frontLeft = new Translation2d(0.292, 0.292);
    Translation2d _frontRight = new Translation2d(0.292, -0.292);
    Translation2d _backRight = new Translation2d(-0.292, -0.292);
    Translation2d _backLeft = new Translation2d(-0.292, 0.292);

    // If we are backwards
    if (_forward.getAsDouble() < 0) {
      currentRotation += 180;
      _left = !_left;
    }

    if (currentRotation >= 330)
      currentRotation = 0;

    // // NEW - JERRY
    int quadrant = ((int) currentRotation) / 45 % 8;

    // // Debug a
    SmartDashboard.putNumber("Quadrant", quadrant);
    SmartDashboard.putBoolean("LEFT PRESSED", _left);
    SmartDashboard.putNumber("Current Rotation", currentRotation);
    SmartDashboard.putNumber("Foward", _forward.getAsDouble());

    switch (quadrant) {
      case 0 :
        _pivotPoint = _left ? _frontLeft : _frontRight;
        break;
      case 1 :
        _pivotPoint = _frontRight;
        break;
      case 2 :
        _pivotPoint = _left ? _frontRight : _backRight;
        break;
      case 3 :
        _pivotPoint = _backRight;
        break;
      case 4 :
        _pivotPoint = _left ? _backRight : _backLeft;
        break;
      case 5 :
        _pivotPoint = _backLeft;
        break;
      case 6 :
        _pivotPoint = _left ? _backLeft : _frontLeft;
        break;
      case 7 :
        _pivotPoint = _frontLeft;
        break;
    }

    _swerveDrive.pivotMotor(_pivotPoint);

    // ORIGINAL

    // if (_left){
    // if ((currentRotation >= 315 && currentRotation <= 360) || (currentRotation >=
    // 0 &&
    // currentRotation < 90)){
    // _pivotPoint = _frontLeft;
    // }
    // else if (currentRotation >= 90 && currentRotation < 180){
    // _pivotPoint = _frontRight;
    // }
    // else if (currentRotation >= 180 && currentRotation < 270){
    // _pivotPoint = _backRight;
    // }
    // else{
    // _pivotPoint = _backLeft;
    // }
    // }
    // else{
    // if ((currentRotation >= 315 && currentRotation <= 360) || (currentRotation >=
    // 0 &&
    // currentRotation < 45)){
    // _pivotPoint = _frontRight;
    // }
    // else if (currentRotation >= 45 && currentRotation < 135){
    // _pivotPoint = _backRight;
    // }
    // else if (currentRotation >= 135 && currentRotation < 225){
    // _pivotPoint = _backLeft;
    // }
    // else{
    // _pivotPoint = _frontLeft;
    // }
    // }
    // _swerveDrive.pivotMotor(_pivotPoint);

    // Called every time the scheduler runs while the command is scheduled.
  }

  @Override
  public void execute() {
    _leds.setColor(Constants.LEDColors.ORANGE);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    _left = false;
    _swerveDrive.resetPivot();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
