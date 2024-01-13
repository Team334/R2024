// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
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
  private final SwerveModule _frontLeft = new SwerveModule(Constants.CAN.DRIVE_FRONT_LEFT, Constants.CAN.ROT_FRONT_LEFT, Constants.CAN.ENC_FRONT_LEFT, Constants.Offsets.ENCODER_FRONT_LEFT, 0.015, 0.15);
  private final SwerveModule _frontRight = new SwerveModule(Constants.CAN.DRIVE_FRONT_RIGHT, Constants.CAN.ROT_FRONT_RIGHT, Constants.CAN.ENC_FRONT_RIGHT, Constants.Offsets.ENCODER_FRONT_RIGHT, 0.015, 0.17);
  private final SwerveModule _backRight = new SwerveModule(Constants.CAN.DRIVE_BACK_RIGHT, Constants.CAN.ROT_BACK_RIGHT, Constants.CAN.ENC_BACK_RIGHT, Constants.Offsets.ENCODER_BACK_RIGHT, 0.015, 0.18);
  private final SwerveModule _backLeft = new SwerveModule(Constants.CAN.DRIVE_BACK_LEFT, Constants.CAN.ROT_BACK_LEFT, Constants.CAN.ENC_BACK_LEFT, Constants.Offsets.ENCODER_BACK_LEFT, 0.015, 0.17);

  private final BNO055 _gyro = BNO055.getInstance(BNO055.opmode_t.OPERATION_MODE_IMUPLUS, BNO055.vector_type_t.VECTOR_EULER);

  private boolean _fieldOrientated = false;

  private Pose2d m_pose = new Pose2d();

  private Field2d _field = new Field2d();

  private final SwerveDrivePoseEstimator _odometry = new SwerveDrivePoseEstimator(
    Constants.Physical.SWERVE_KINEMATICS,
    getHeadingRaw(),
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

    SmartDashboard.putNumber("Gyro Raw", getHeadingRaw().getDegrees());
    SmartDashboard.putNumber("Gyro", getHeading().getDegrees());

    SmartDashboard.putBoolean("Field Orientated", _fieldOrientated);
    
    // Update the pose
    m_pose = _odometry.update(getHeadingRaw(), new SwerveModulePosition[] {
      _frontLeft.getPosition(),
      _frontRight.getPosition(),
      _backRight.getPosition(),
      _backLeft.getPosition()
    });

    _field.setRobotPose(m_pose);
    SmartDashboard.putData("FIELD", _field);
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
    _frontLeft.setState(states[0]);
    _frontRight.setState(states[1]);
    _backRight.setState(states[2]);
    _backLeft.setState(states[3]);
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
    Pose2d new_pose = new Pose2d(
      m_pose.getTranslation().getX(),
      m_pose.getTranslation().getY(),
      Rotation2d.fromDegrees(0)
    );

    resetPose(new_pose);
    
  }

  public void resetTranslation(){
    Pose2d new_pose = new Pose2d(
      0,
      0,
      m_pose.getRotation()
    );

    resetPose(new_pose);
  }


  public void resetPose(Pose2d new_pose){
    _odometry.resetPosition(getHeadingRaw(), 
      new SwerveModulePosition[] {
        _frontLeft.getPosition(),
        _frontRight.getPosition(),
        _backRight.getPosition(),
        _backLeft.getPosition()
      }, new_pose);
  }

  
  /**
   * Get heading directly from gyro as Rotation2d
   */
  public Rotation2d getHeadingRaw() {
    return Rotation2d.fromDegrees(-Math.IEEEremainder(_gyro.getHeading(), 360));
  }

  /**
   * Get heading from the odometry (pose estimator)
   */
  public Rotation2d getHeading() {
    return _odometry.getEstimatedPosition().getRotation();
  }
}