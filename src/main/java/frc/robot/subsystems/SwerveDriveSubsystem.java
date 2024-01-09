// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.utils.BNO055;
import frc.robot.utils.SwerveModule;

public class SwerveDriveSubsystem extends SubsystemBase {
  // TODO: Get angle offset for each module (zero each one)
  private final SwerveModule _frontLeft = new SwerveModule(Constants.CAN.DRIVE_FRONT_LEFT, Constants.CAN.ROT_FRONT_LEFT, Constants.CAN.ENC_FRONT_LEFT, Constants.Offsets.ENCODER_FRONT_LEFT, 0.015, 0.15);
  private final SwerveModule _frontRight = new SwerveModule(Constants.CAN.DRIVE_FRONT_RIGHT, Constants.CAN.ROT_FRONT_RIGHT, Constants.CAN.ENC_FRONT_RIGHT, Constants.Offsets.ENCODER_FRONT_RIGHT, 0.015, 0.17);
  private final SwerveModule _backRight = new SwerveModule(Constants.CAN.DRIVE_BACK_RIGHT, Constants.CAN.ROT_BACK_RIGHT, Constants.CAN.ENC_BACK_RIGHT, Constants.Offsets.ENCODER_BACK_RIGHT, 0.015, 0.18);
  private final SwerveModule _backLeft = new SwerveModule(Constants.CAN.DRIVE_BACK_LEFT, Constants.CAN.ROT_BACK_LEFT, Constants.CAN.ENC_BACK_LEFT, Constants.Offsets.ENCODER_BACK_LEFT, 0.015, 0.17);

  private final BNO055 _gyro = BNO055.getInstance(BNO055.opmode_t.OPERATION_MODE_IMUPLUS, BNO055.vector_type_t.VECTOR_EULER);

  private boolean _fieldOrientated = false;

  private final SwerveDrivePoseEstimator _odometry = new SwerveDrivePoseEstimator(
    Constants.Physical.SWERVE_KINEMATICS,
    getHeading(),
    new SwerveModulePosition[] {
      _frontLeft.getPosition(),
      _frontRight.getPosition(),
      _backRight.getPosition(),
      _backLeft.getPosition()
    },
    new Pose2d()
  );

  /** Creates a new SwerveDrive. */
  public SwerveDriveSubsystem() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Front Left Angle", _frontLeft.getAngle());
    SmartDashboard.putNumber("Front Right Angle", _frontRight.getAngle());
    SmartDashboard.putNumber("Back Right Angle", _backRight.getAngle());
    SmartDashboard.putNumber("Back Left Angle", _backLeft.getAngle());

    SmartDashboard.putNumber("Front Left Speed", _frontLeft.getDriveVelocity());
    SmartDashboard.putNumber("Front Right Speed", _frontRight.getDriveVelocity());
    SmartDashboard.putNumber("Back Right Speed", _backRight.getDriveVelocity());
    SmartDashboard.putNumber("Back Left Speed", _backLeft.getDriveVelocity());

    SmartDashboard.putNumber("Gyro", getHeadingRaw());

    SmartDashboard.putBoolean("Field Orientated", _fieldOrientated);
  }

  public boolean getFieldOrientated() {
    return _fieldOrientated;
  }

  public void toggleOrient() {
    _fieldOrientated = !_fieldOrientated;
  }

  /**
   * Sets the state of each SwerveModule through an array.
   */
  public void setStates(SwerveModuleState[] states) {
    SmartDashboard.putNumber("Front Left Deg", states[0].angle.getDegrees());
    SmartDashboard.putNumber("Front Left Rot Volt", _frontLeft._rotationMotor.getMotorOutputVoltage());

    _frontLeft.setState(states[0]);
    _frontRight.setState(states[1]);
    _backRight.setState(states[2]);
    _backLeft.setState(states[3]);
  }

  /**
   * Calls drive method of each SwerveModule.
   */
  public void driveTest(double speed) {
    _frontLeft.drive(speed);
    _frontRight.drive(speed);
    _backRight.drive(speed);
    _backLeft.drive(speed);
  }

  /**
   * Calls rotate method of each SwerveModule.
   */
  public void rotateTest(double speed) {
    _frontLeft.rotate(speed);
    _frontRight.rotate(speed);
    _backRight.rotate(speed);
    _backLeft.rotate(speed);
  }

  /**
   * Sets all the SwerveModules to the provided state.
   */
  public void stateTest(SwerveModuleState state) {
    _frontLeft.setState(state);
    _frontRight.setState(state);
    _backRight.setState(state);
    _backLeft.setState(state);
  }

  public void resetGyro() {
    // TODO: SET UP ODOMETRY
  }

  public double getHeadingRaw() {
    return -Math.IEEEremainder(_gyro.getHeading(), 360);
  }

  public Rotation2d getHeading() {
    return Rotation2d.fromDegrees(getHeadingRaw());
  }
}