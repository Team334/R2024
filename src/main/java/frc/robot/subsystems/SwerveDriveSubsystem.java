// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.fasterxml.jackson.databind.deser.std.ContainerDeserializerBase;
import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.utils.BNO055;
import frc.robot.utils.SwerveModule;

/**
 * @author Peter Gutkovich
 * @author Elvis Osmanov
 * @author Cherine Soewignjo
 * @author Peleh Liu
 */
public class SwerveDriveSubsystem extends SubsystemBase {
  // TODO: Get angle offset for each module (zero each one)
  private final SwerveModule _frontLeft = new SwerveModule(Constants.CAN.DRIVE_FRONT_LEFT, Constants.CAN.ROT_FRONT_LEFT,
      Constants.CAN.ENC_FRONT_LEFT, Constants.Offsets.ENCODER_FRONT_LEFT, 0.015, 0.15);
  private final SwerveModule _frontRight = new SwerveModule(Constants.CAN.DRIVE_FRONT_RIGHT,
      Constants.CAN.ROT_FRONT_RIGHT, Constants.CAN.ENC_FRONT_RIGHT, Constants.Offsets.ENCODER_FRONT_RIGHT, 0.015, 0.17);
  private final SwerveModule _backRight = new SwerveModule(Constants.CAN.DRIVE_BACK_RIGHT, Constants.CAN.ROT_BACK_RIGHT,
      Constants.CAN.ENC_BACK_RIGHT, Constants.Offsets.ENCODER_BACK_RIGHT, 0.015, 0.18);
  private final SwerveModule _backLeft = new SwerveModule(Constants.CAN.DRIVE_BACK_LEFT, Constants.CAN.ROT_BACK_LEFT,
      Constants.CAN.ENC_BACK_LEFT, Constants.Offsets.ENCODER_BACK_LEFT, 0.015, 0.17);

  private final BNO055 _gyro = BNO055.getInstance(BNO055.opmode_t.OPERATION_MODE_IMUPLUS,
      BNO055.vector_type_t.VECTOR_EULER);

  private boolean _fieldOrientated = false;

  private Pose2d _pose = new Pose2d();

  private Field2d _field = new Field2d();

  private VisionSubsystem _visionSubsystem;

  private final SwerveDrivePoseEstimator _odometry = new SwerveDrivePoseEstimator(
      Constants.Physical.SWERVE_KINEMATICS,
      getHeadingRaw(),
      new SwerveModulePosition[] {
          _frontLeft.getPosition(),
          _frontRight.getPosition(),
          _backRight.getPosition(),
          _backLeft.getPosition()
      },
      new Pose2d(),
      VecBuilder.fill(0.008, 0.008, 0.0075),
      VecBuilder.fill(0.2, .2, .75));

  public Pose2d getPose() {
    return _pose;
  }

  public ChassisSpeeds getRobotRelativeSpeeds(){
    return Constants.Physical.SWERVE_KINEMATICS.toChassisSpeeds(
      _frontLeft.getState(),
      _frontRight.getState(),
      _backRight.getState(),
      _backLeft.getState()
    );
  }

  /** Creates a new SwerveDrive. */
  // TODO: GET THIS SHITZ FIXED
  public SwerveDriveSubsystem(VisionSubsystem visionSubsystem) {
    _visionSubsystem = visionSubsystem;

    AutoBuilder.configureHolonomic(
        this::getPose,
        this::resetPose,
        this::getRobotRelativeSpeeds,

        );
  }

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

    SmartDashboard.putNumber("Gyro Raw", getHeadingRaw().getDegrees());
    SmartDashboard.putNumber("Gyro", getHeading().getDegrees());

    SmartDashboard.putBoolean("Field Orientated", _fieldOrientated);

    // Update the bot's pose
    _pose = _odometry.update(getHeadingRaw(), new SwerveModulePosition[] {
        _frontLeft.getPosition(),
        _frontRight.getPosition(),
        _backRight.getPosition(),
        _backLeft.getPosition()
    });

    if (_visionSubsystem.isApriltagVisible()) {
      _odometry.addVisionMeasurement(_visionSubsystem.getBotpose(), Timer.getFPGATimestamp());
    }

    _field.setRobotPose(_pose);
    SmartDashboard.putData("FIELD", _field);
  }

  /** Whether the drive is field oriented or not. */
  public boolean getFieldOrientated() {
    return _fieldOrientated;
  }

  /** Toggle the field orient. */
  public void toggleOrient() {
    _fieldOrientated = !_fieldOrientated;
  }

  public void driveChassis() {
    // IMPORTANT: X-axis and Y-axis are flipped (based on wpilib coord system)
    ChassisSpeeds chassisSpeeds;

    if (_fieldOrientated) {
      chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed * Constants.Speeds.SWERVE_DRIVE_MAX_SPEED, ySpeed * Constants.Speeds.SWERVE_DRIVE_MAX_SPEED, rotationSpeed * Constants.Speeds.SWERVE_DRIVE_MAX_ANGULAR_SPEED, _swerveDrive.getHeading());
    } 
    
    else {
      chassisSpeeds = new ChassisSpeeds(
        // 0,
        xSpeed * Constants.Speeds.SWERVE_DRIVE_MAX_SPEED,
        ySpeed * Constants.Speeds.SWERVE_DRIVE_MAX_SPEED,
        rotationSpeed * Constants.Speeds.SWERVE_DRIVE_MAX_ANGULAR_SPEED
      );
    }

    SwerveModuleState[] moduleStates = Constants.Physical.SWERVE_KINEMATICS.toSwerveModuleStates(chassisSpeeds);
    _swerveDrive.setStates(moduleStates);
  }

  /**
   * Sets the state of each SwerveModule through an array.
   */
  public void setStates(SwerveModuleState[] states) {
    _frontLeft.setState(states[0]);
    _frontRight.setState(states[1]);
    _backRight.setState(states[2]);
    _backLeft.setState(states[3]);
  }

  /** Resets the heading of the drive as supplied by the pose estimator. */
  public void resetGyro() {
    Pose2d new_pose = new Pose2d(
        _pose.getTranslation().getX(),
        _pose.getTranslation().getY(),
        Rotation2d.fromDegrees(0));

    resetPose(new_pose);
  }

  /** Resets the translation of the drive as supplied by the pose estimator. */
  public void resetTranslation() {
    Pose2d new_pose = new Pose2d(
        0,
        0,
        _pose.getRotation());

    resetPose(new_pose);
  }

  /** Resets the pose of the pose estimator to the supplied new pose. */
  public void resetPose(Pose2d newPose) {
    _odometry.resetPosition(getHeadingRaw(),
        new SwerveModulePosition[] {
            _frontLeft.getPosition(),
            _frontRight.getPosition(),
            _backRight.getPosition(),
            _backLeft.getPosition()
        }, newPose);
  }

  /**
   * Get heading of the drive from the odometry (pose estimator).
   */
  public Rotation2d getHeading() {
    return _odometry.getEstimatedPosition().getRotation();
  }

  /**
   * Get heading DIRECTLY from gyro as a Rotation2d.
   */
  public Rotation2d getHeadingRaw() {
    return Rotation2d.fromDegrees(-Math.IEEEremainder(_gyro.getHeading(), 360));
  }

}