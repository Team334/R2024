/* Copyright (C) 2024 Team 334. All Rights Reserved.*/
package frc.robot.utils;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

/**
 * @author Peter Gutkovich
 * @author Elvis Osmanov
 */
public class SwerveModule {
  private final TalonFX _driveMotor;
  private final TalonFX _rotationMotor;

  private final PIDController _driveController;
  private final PIDController _rotationController;

  private final SimpleMotorFeedforward _driveFeedforward = new SimpleMotorFeedforward(
      Constants.FeedForward.MODULE_DRIVE_KS, Constants.FeedForward.MODULE_DRIVE_KV);

  private final CANcoder _encoder;

  private final String _name;

  /**
   * Represents a single swerve module built with talons for rotation and drive
   * control, and a cancoder for angle.
   *
   * @param name
   *            - The name of this module.
   * @param driveMotorId
   *            - CAN ID of drive motor.
   * @param rotationMotorId
   *            - CAN ID of rotation motor.
   * @param encoderId
   *            - CAN ID of cancoder.
   * @param driveP
   *            - kP for the drive controller.
   * @param rotationP
   *            - kP for the rotation controller.
   */
  public SwerveModule(String name, int driveMotorId, int rotationMotorId, int encoderId, double driveP,
      double rotationP) {
    _driveMotor = new TalonFX(driveMotorId);
    _rotationMotor = new TalonFX(rotationMotorId);

    // NO NEED FOR THIS CODE ANYMORE, FIGURED HOW TO DO OFFSETS IN PHOENIX TUNER
    // new stuff because CTRE update
    // MagnetSensorConfigs encoderConfig = new MagnetSensorConfigs();
    // encoderConfig.AbsoluteSensorRange =
    // AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
    // encoderConfig.MagnetOffset = (angleOffset / 180) / 2;

    _encoder = new CANcoder(encoderId);

    // _encoder.getConfigurator().apply(encoderConfig);

    _name = name;

    _driveController = new PIDController(driveP, 0, 0);

    _rotationController = new PIDController(rotationP, 0, 0);
    _rotationController.enableContinuousInput(-180, 180);

    SmartDashboard.putData(_driveController);
    SmartDashboard.putData(_rotationController);

    TalonFXConfig.configureFalcon(_driveMotor, false);
    TalonFXConfig.configureFalcon(_rotationMotor, true);
  }

  /** Display's this module's info on SmartDashboard. */
  public void displayInfo() {
    SmartDashboard.putNumber(_name + " Angle", getAngle());
    SmartDashboard.putNumber(_name + " Velocity", getDriveVelocity());
    SmartDashboard.putNumber(_name + " Current", _rotationMotor.getTorqueCurrent().getValueAsDouble());
  }

  /**
   * Get the talons belonging to this module as an array.
   *
   * @return [drive talon, rotation talon]
   */
  public TalonFX[] getTalons() {
    TalonFX[] fxes = new TalonFX[2];
    fxes[0] = _driveMotor;
    fxes[1] = _rotationMotor;
    return fxes;
  }

  /** Set the percentage output of the drive motor. */
  public void drive(double speed) {
    _driveMotor.set(speed);
  }

  /** Set the percentage output of the rotation motor. */
  public void rotate(double speed) {
    _rotationMotor.set(speed);
  }

  /** Get the absolute angle of the module as an int (-180 to 180 degrees). */
  public int getAngle() {
    return Double.valueOf(_encoder.getAbsolutePosition().getValueAsDouble() * 2 * 180).intValue(); // ctre update
  }

  /** Get the velocity of the drive wheel (meters per second). */
  public double getDriveVelocity() {
    double talon_rps = _driveMotor.getRotorVelocity().getValueAsDouble(); // ctre update

    // return the speed of the drive wheel itself (talon rps times gear ratio time
    // wheel size)
    // in
    // m/s
    return (talon_rps / Constants.Physical.SWERVE_DRIVE_GEAR_RATIO)
        * Constants.Physical.SWERVE_DRIVE_WHEEL_CIRCUMFERENCE;
  }

  /**
   * Get the position of this module (distance traveled by wheel + angle).
   *
   * @see SwerveModulePosition
   */
  public SwerveModulePosition getPosition() {
    double talon_rotations = _driveMotor.getPosition().getValueAsDouble();
    double distance = (talon_rotations / Constants.Physical.SWERVE_DRIVE_GEAR_RATIO)
        * Constants.Physical.SWERVE_DRIVE_WHEEL_CIRCUMFERENCE;

    return new SwerveModulePosition(distance, Rotation2d.fromDegrees(getAngle()));
  }

  /**
   * Set the state of this module. This function must be called repeatedly for the
   * state to be set.
   *
   * @see SwerveModuleState
   */
  public void setState(SwerveModuleState state) {
    // current system for setting the state of a module
    // rotation: pure pid control
    // velocity: feedforward control mainly along with pid control for small
    // disturbances

    state = SwerveModuleState.optimize(state, new Rotation2d(Math.toRadians(getAngle())));

    double speed = MathUtil.clamp(state.speedMetersPerSecond, -Constants.Speeds.SWERVE_DRIVE_MAX_SPEED,
        Constants.Speeds.SWERVE_DRIVE_MAX_SPEED);

    double rotation_pid = MathUtil.clamp(_rotationController.calculate(getAngle(), state.angle.getDegrees()),
        -0.150, 0.150);

    // double drive_feedforward = (speed / Constants.Speeds.SWERVE_DRIVE_MAX_SPEED);
    double drive_feedforward = UtilFuncs.FromVolts(_driveFeedforward.calculate(speed));
    double drive_pid = _driveController.calculate(getDriveVelocity(), speed);

    drive_pid = 0;

    rotate(rotation_pid);
    drive(drive_feedforward + drive_pid);
    // drive(0.08);
  }

  /**
   * Get the state of this module.
   *
   * @see SwerveModuleState
   */
  public SwerveModuleState getState() {
    return new SwerveModuleState(getDriveVelocity(), Rotation2d.fromDegrees(getAngle()));
  }
}
