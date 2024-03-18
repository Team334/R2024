/* Copyright (C) 2024 Team 334. All Rights Reserved.*/
package frc.robot.subsystems;

import java.sql.Driver;
import java.util.Optional;

import com.ctre.phoenix6.Orchestra;
import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.BuiltInAccelerometer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.Physical;
import frc.robot.Constants.Presets;
import frc.robot.subsystems.ShooterSubsystem.ShooterState;
import frc.robot.utils.BNO055;
import frc.robot.utils.SwerveModule;
import frc.robot.utils.UtilFuncs;

/**
 * @author Peter Gutkovich
 * @author Elvis Osmanov
 * @author Cherine Soewignjo
 * @author Peleh Liu
 */
public class SwerveDriveSubsystem extends SubsystemBase {
  // each swerve module
  private final SwerveModule _frontLeft = new SwerveModule("Front Left", Constants.CAN.DRIVE_FRONT_LEFT,
      Constants.CAN.ROT_FRONT_LEFT, Constants.CAN.ENC_FRONT_LEFT);

  private final SwerveModule _frontRight = new SwerveModule("Front Right", Constants.CAN.DRIVE_FRONT_RIGHT,
      Constants.CAN.ROT_FRONT_RIGHT, Constants.CAN.ENC_FRONT_RIGHT);

  private final SwerveModule _backRight = new SwerveModule("Back Right", Constants.CAN.DRIVE_BACK_RIGHT,
      Constants.CAN.ROT_BACK_RIGHT, Constants.CAN.ENC_BACK_RIGHT);

  private final SwerveModule _backLeft = new SwerveModule("Back Left", Constants.CAN.DRIVE_BACK_LEFT,
      Constants.CAN.ROT_BACK_LEFT, Constants.CAN.ENC_BACK_LEFT);

  SwerveModuleState[] states = new SwerveModuleState[]{
      new SwerveModuleState(_frontLeft.getDriveVelocity(), Rotation2d.fromDegrees(_frontLeft.getAngle())),
      new SwerveModuleState(_frontRight.getDriveVelocity(), Rotation2d.fromDegrees(_frontLeft.getAngle())),
      new SwerveModuleState(_backRight.getDriveVelocity(), Rotation2d.fromDegrees(_frontLeft.getAngle())),
      new SwerveModuleState(_backLeft.getDriveVelocity(), Rotation2d.fromDegrees(_frontLeft.getAngle()))};

  private final BNO055 _gyro = BNO055.getInstance(BNO055.opmode_t.OPERATION_MODE_IMUPLUS,
      BNO055.vector_type_t.VECTOR_EULER);
  
  // private final AHRS _gyro = new AHRS();

  private VisionSubsystem _visionSubsystem;

  Translation2d _pivotPoint = new Translation2d(0, 0);

  private final Orchestra _orchestra = new Orchestra();
  String song = "output.chrp";

  private DrivingSpeeds _drivingState = DrivingSpeeds.FAST;

  private Field2d _field = new Field2d();

  /** A boolean for whether the swerve is field oriented or not. */
  public boolean fieldOriented = false;

  StructArrayPublisher<SwerveModuleState> publisher = NetworkTableInstance.getDefault()
      .getStructArrayTopic("MyStates", SwerveModuleState.struct).publish();


  // Pose Estimator -> Has built in odometry and uses supplied vision measurements
  private final SwerveDrivePoseEstimator _estimator = new SwerveDrivePoseEstimator(
      Constants.Physical.SWERVE_KINEMATICS, getHeadingRaw(),
      new SwerveModulePosition[]{_frontLeft.getPosition(), _frontRight.getPosition(), _backRight.getPosition(),
          _backLeft.getPosition()},
      new Pose2d(), VecBuilder.fill(0.01, 0.01, 0.01), VecBuilder.fill(0.9, 0.9, 9999999)); // note: LL heading std was 0.9 

  // OTHER POSSIBLE STD DEV VALUES:
  // VecBuilder.fill(0.006, 0.006, 0.007), VecBuilder.fill(0.52, 0.52, 1.35)
  // VecBuilder.fill(0.006, 0.006, 0.007), VecBuilder.fill(0.5, 0.5, 1.3)

  public enum DrivingSpeeds {
    FAST,
    SLOW
  }

  /** Return the estimated pose of the swerve chassis. */
  public Pose2d getPose() {
    return _estimator.getEstimatedPosition();
  }

  /** Get the drive's chassis speeds (robot relative). */
  public ChassisSpeeds getRobotRelativeSpeeds() {
    return Constants.Physical.SWERVE_KINEMATICS.toChassisSpeeds(getStates());
  }

  /** Creates a new SwerveDriveSubsystem. */
  public SwerveDriveSubsystem(VisionSubsystem visionSubsystem) {
    _visionSubsystem = visionSubsystem;

    // resetPose(
    //   new Pose2d(2.52, 5.25, Rotation2d.fromDegrees(180))
    // ); // for testing

    // setupOrchestra();

    // pathplannerlib setup
    AutoBuilder.configureHolonomic(this::getPose, this::resetPose, this::getRobotRelativeSpeeds, this::driveChassis,
        new HolonomicPathFollowerConfig(Constants.PID.PP_TRANSLATION, Constants.PID.PP_ROTATION,
            Constants.Speeds.SWERVE_DRIVE_MAX_SPEED, Constants.Physical.SWERVE_DRIVE_BASE_RADIUS,
            new ReplanningConfig()),
        () -> {
          if (UtilFuncs.GetAlliance() == Alliance.Red) {
            return true;
          }
          return false;
        }, this);
    
    SmartDashboard.putData("Gyro", new Sendable() {
      @Override
      public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("Gyro");
        builder.addDoubleProperty("Value", () -> getHeading().getDegrees(), null);
      }
    });

    SmartDashboard.putData("Swerve Drive", new Sendable() {
      @Override
      public void initSendable(SendableBuilder builder) {
        _frontLeft.displayInfo(builder);
        _frontRight.displayInfo(builder);
        _backRight.displayInfo(builder);
        _backLeft.displayInfo(builder);
        builder.addBooleanProperty("Swerve FAST", () -> _drivingState == DrivingSpeeds.FAST, null);
      }
    });
  }

  @Override
  public void periodic() {
    publisher.set(states);

    SmartDashboard.putNumber("Gyro RAW", getHeadingRaw().getDegrees());
    SmartDashboard.putBoolean("Field Oriented", fieldOriented);
    SmartDashboard.putNumber("Match Time", DriverStation.getMatchTime());
    SmartDashboard.putNumber("Speaker Distance", speakerDistance());

    // Update the bot's pose
    _estimator.update(getHeadingRaw(), new SwerveModulePosition[]{
      _frontLeft.getPosition(),
      _frontRight.getPosition(), 
      _backRight.getPosition(), 
      _backLeft.getPosition()
    });

    Optional<Pose2d> visionBotpose = _visionSubsystem.getBotpose();
    Optional<Pose2d> resetBotpose = _visionSubsystem.resetPose();

    SmartDashboard.putBoolean("VISION VALID", visionBotpose.isPresent());

    // RESET BOTPOSE COMPLETELY
    // if (resetBotpose.isPresent() && DriverStation.isTeleopEnabled()) {
    //   resetPose(new Pose2d(resetBotpose.get().getX(), resetBotpose.get().getY(), getHeading()));
    // }

    // UPDATE BOTPOSE WITH VISION
    if (visionBotpose.isPresent()) {
      _estimator.addVisionMeasurement(visionBotpose.get(), _visionSubsystem.getLatency());
    }

    // field icon updates
    _field.setRobotPose(getPose());
    SmartDashboard.putData("FIELD", _field);
  }

  // to setup talon orchestra
  private void setupOrchestra() {
    SwerveModule[] modules = new SwerveModule[4];
    modules[0] = _frontLeft;
    modules[1] = _frontRight;
    modules[2] = _backRight;
    modules[3] = _backLeft;

    for (int i = 0; i < modules.length; i++) {
      for (int j = 0; j < 2; j++) {
        _orchestra.addInstrument(modules[i].getTalons()[j]);
      }
    }

    _orchestra.loadMusic(song);
    _orchestra.play();
  }

  /**
   * Toggle the driving speed between slow/fast.
   */
  public void toggleSpeed(){
    if (_drivingState == DrivingSpeeds.FAST) {
      _drivingState = DrivingSpeeds.SLOW;
    } else {
      _drivingState = DrivingSpeeds.FAST;
    }
  }
  
  /**
   * Get drive coeff based on toggled speed.
   */
  public double getDriveCoeff() {
    if (_drivingState == DrivingSpeeds.FAST) {
      return Constants.Speeds.SWERVE_DRIVE_FAST_COEFF;
    } else {
      return Constants.Speeds.SWERVE_DRIVE_SLOW_COEFF;
    }
  }

  /**
   * Set the chassis speed of the swerve drive.
   *
   * <p>
   * Chassis speed will be treated as field oriented if the fieldOriented class
   * attribute is set to true, otherwise it will be robot-relative.
   *
   * @see ChassisSpeeds (wpilib chassis speeds class)
   */
  public void driveChassis(ChassisSpeeds chassisSpeeds) {
    // IMPORTANT: X-axis and Y-axis are flipped (based on wpilib coord system)
    if (fieldOriented) {
      double relativeHeading = getHeading().getDegrees() + (UtilFuncs.GetAlliance() == Alliance.Red ? 180 : 0);
      chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(chassisSpeeds, Rotation2d.fromDegrees(relativeHeading));
    }

    SwerveModuleState[] moduleStates = Constants.Physical.SWERVE_KINEMATICS.toSwerveModuleStates(chassisSpeeds,
        _pivotPoint);
    setStates(moduleStates);
  }

  /**
   * Sets the state of each SwerveModule through an array.
   *
   * <p>
   * Order -> front left, front right, back right, back left
   */
  public void setStates(SwerveModuleState[] states) {
    _frontLeft.setState(states[0]);
    _frontRight.setState(states[1]);
    _backRight.setState(states[2]);
    _backLeft.setState(states[3]);
  }

  /**
   * Get the state of each SwerveModule.
   *
   * @return An array of each module state (order -> front left, front right, back
   *         right, back left)
   */
  public SwerveModuleState[] getStates() {
    SwerveModuleState[] states = {_frontLeft.getState(), _frontRight.getState(), _backRight.getState(),
        _backLeft.getState()};

    return states;
  }

  /** Resets the pose estimator's heading of the drive to 0. */
  public void resetGyro() {
    Pose2d current_pose = getPose();
    Pose2d new_pose = new Pose2d(current_pose.getTranslation().getX(), current_pose.getTranslation().getY(),
        Rotation2d.fromDegrees(0));

    resetPose(new_pose);
  }

  /** Resets pose estimator's translation of the drive to (0, 0). */
  public void resetTranslation() {
    Pose2d new_pose = new Pose2d(0, 0, getPose().getRotation());

    resetPose(new_pose);
  }

  /** Resets the pose estimator to the supplied new pose. */
  public void resetPose(Pose2d newPose) {
    _estimator.resetPosition(getHeadingRaw(), new SwerveModulePosition[]{_frontLeft.getPosition(),
        _frontRight.getPosition(), _backRight.getPosition(), _backLeft.getPosition()}, newPose);
  }

  /** Get heading of the drive from the pose estimator. */
  public Rotation2d getHeading() {
    return getPose().getRotation();
  }

  /** Get heading DIRECTLY from the BNO055 gyro as a Rotation2d. */
  public Rotation2d getHeadingRaw() {
    return Rotation2d.fromDegrees(-Math.IEEEremainder(_gyro.getHeading(), 360));
    // return Rotation2d.fromDegrees(-Math.IEEEremainder(_gyro.getAngle(), 360));
  }

  /**
   * Get the distance between the chassis and the speaker for shooting the note.
   * 
   * (could be modified to shoot while moving)
   */
  public double speakerShotDistance() {
    return speakerDistance();
  }

  /**
   * Get the distance of the chassis from the speaker shot point.
   * 
   * @return Norm of the distance vector.
   */
  public double speakerDistance() {
    // Pose3d speakerPose = UtilFuncs.GetAlliance() == Alliance.Red ? FieldConstants.SPEAKER_POSE_RED : FieldConstants.SPEAKER_POSE_BLUE;
    Pose3d speakerPose = UtilFuncs.GetSpeakerPose();

    Translation2d speakerTranslation = new Translation2d(speakerPose.getX(), speakerPose.getY());
    Translation2d botTranslation = getPose().getTranslation();

    Translation2d distanceVec = speakerTranslation.minus(botTranslation);

    return distanceVec.getNorm();
  }

  /**
   * Get the setpoint x and y angles as well as elevater height for auto-aim.
   * 
   * @return [xSpeakerAngle, ySpeakerAngle, elevatorHeight]
   */
  public double[] speakerSetpoints() {
    double xSpeakerAngle;
    double ySpeakerAngle;

    // Pose3d speakerPose = UtilFuncs.GetAlliance() == Alliance.Red ? FieldConstants.SPEAKER_POSE_RED : FieldConstants.SPEAKER_POSE_BLUE;
    Pose3d speakerPose = UtilFuncs.GetSpeakerPose();

    Translation2d speakerTranslation = new Translation2d(speakerPose.getX(), speakerPose.getY());
    Translation2d botTranslation = getPose().getTranslation();

    Translation2d distanceVec = speakerTranslation.minus(botTranslation);

    double elevatorHeight = Physical.ELEVATOR_MAX_SHOOT_HEIGHT + (distanceVec.getNorm() * Presets.ELEVATOR_HEIGHT_RATE); // TODO: get values and test

    xSpeakerAngle = MathUtil.inputModulus(distanceVec.getAngle().getDegrees(), -180, 180);

    double zDifference = speakerPose.getZ() - elevatorHeight;
    ySpeakerAngle = Math.toDegrees(Math.atan(zDifference / distanceVec.getNorm()));

    double[] offsets = {xSpeakerAngle, ySpeakerAngle, elevatorHeight - Physical.ELEVATOR_LOWEST_HEIGHT};

    return offsets;
  }

  public void pivotMotor(Translation2d pivotPoint) {
    _pivotPoint = pivotPoint;
  }

  public void resetPivot() {
    _pivotPoint = new Translation2d(0, 0);
  }
}
