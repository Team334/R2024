/*                                  Team 334                                  */
/* Copyright (c) 2024 Team 334. All Rights Reserved.                          */

package frc.robot.utils;

import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.Constants;
import frc.robot.Robot;

/**
 * @author Peter Gutkovich
 * @author Elvis Osmanov
 */
public class SwerveModule {
  private final TalonFX _driveMotor;
  private final TalonFX _rotationMotor;

  private final PIDController _driveController;
  private final PIDController _rotationController;

  private final CANcoder _encoder;

  /**
   * Represents a single swerve module built with talons for rotation and drive control, and a
   * cancoder for angle.
   *
   * @param driveMotorId CAN ID of drive motor.
   * @param rotationMotorId CAN ID of rotation motor.
   * @param encoderId CAN ID of cancoder.
   * @param angleOffset Angle offset to add to the absolute cancoder.
   * @param driveP kP for the drive controller.
   * @param rotationP kP for the rotation controller.
   */
  public SwerveModule(
      int driveMotorId,
      int rotationMotorId,
      int encoderId,
      double angleOffset,
      double driveP,
      double rotationP) {
    _driveMotor = new TalonFX(driveMotorId);
    _rotationMotor = new TalonFX(rotationMotorId);

    // new stuff because CTRE update
    MagnetSensorConfigs encoderConfig = new MagnetSensorConfigs();
    encoderConfig.AbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
    encoderConfig.MagnetOffset = (angleOffset / 180) / 2;

    _encoder = new CANcoder(encoderId);
    _encoder.getConfigurator().apply(encoderConfig);

    _driveController = new PIDController(driveP, 0, 0);

    _rotationController = new PIDController(rotationP, 0, 0);
    _rotationController.enableContinuousInput(-180, 180);

    TalonFXConfig.configureFalcon(_driveMotor, false);
    TalonFXConfig.configureFalcon(_rotationMotor, false);
  }

  /** Set the percentage output of the drive motor. */
  public void drive(double speed) {
    _driveMotor.set(speed);
  }

  /** Set the percentage output of the rotation motor. */
  public void rotate(double speed) {
    _rotationMotor.set(speed);
  }

  /** Get the absolute angle of the module (-180 to 180 degrees). */
  public double getAngle() {
    return _encoder.getAbsolutePosition().getValueAsDouble() * 2 * 180; // ctre update
  }

  /** Get the velocity of the drive wheel (meters per second). */
  public double getDriveVelocity() {
    double talon_rps = _driveMotor.getRotorVelocity().getValueAsDouble(); // ctre update

    // return the speed of the drive wheel itself (talon rps times gear ratio time wheel size) in
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
    double distance =
        (talon_rotations / Constants.Physical.SWERVE_DRIVE_GEAR_RATIO)
            * Constants.Physical.SWERVE_DRIVE_WHEEL_CIRCUMFERENCE;

    return new SwerveModulePosition(distance, Rotation2d.fromDegrees(getAngle()));
  }

  /**
   * Set the state of this module.
   *
   * @see SwerveModuleState
   */
  public void setState(SwerveModuleState state) {
    // current system for setting the state of a module
    // rotation: pure pid control
    // velocity: feedforward control mainly along with pid control for small disturbances

    state = SwerveModuleState.optimize(state, new Rotation2d(Math.toRadians(getAngle())));
    double speed =
        MathUtil.clamp(
            state.speedMetersPerSecond,
            -Constants.Speeds.SWERVE_DRIVE_MAX_SPEED,
            Constants.Speeds.SWERVE_DRIVE_MAX_SPEED);

    double rotation_volts =
        -MathUtil.clamp(
            _rotationController.calculate(getAngle(), state.angle.getDegrees()), -1.5, 1.5);

        // double drive_pid = _driveController.calculate(getDriveVelocity(), speed);
        double drive_pid = 0;
        double drive_output = (speed / Constants.Speeds.SWERVE_DRIVE_MAX_SPEED);
        drive_output += drive_pid;

        rotate(rotation_volts / RobotController.getBatteryVoltage());
        drive(drive_output);
    }

  /**
   * Get the state of this module.
   *
   * @see SwerveModuleState
   */
  public SwerveModuleState getState() {
    return new SwerveModuleState(getDriveVelocity(), Rotation2d.fromDegrees(getAngle()));
  }

  public TalonFX[] returnTalons() {
    TalonFX[] fxes = new TalonFX[2];
    fxes[0] = _driveMotor;
    fxes[1] = _rotationMotor;
    return fxes;
  }
}
