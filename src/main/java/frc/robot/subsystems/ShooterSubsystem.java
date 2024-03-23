/* Copyright (C) 2024 Team 334. All Rights Reserved.*/
package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;

import javax.sound.sampled.Port;

import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Encoders;
import frc.robot.Constants.FeedForward;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.PID;
import frc.robot.Constants.Physical;
import frc.robot.Constants.Ports;
import frc.robot.Constants.Presets;
import frc.robot.Constants.Speeds;
import frc.robot.utils.UtilFuncs;
import frc.robot.utils.configs.NeoConfig;
import frc.robot.utils.configs.TalonFXConfig;

/**
 * @author Elvis Osmanov
 * @author Peleh Liu
 * @author Cherine Soewingjo
 * @author Peter Gutkovich
 */
public class ShooterSubsystem extends SubsystemBase {
  private final CANSparkMax _leftMotor = new CANSparkMax(Constants.CAN.SHOOTER_LEFT, MotorType.kBrushless);
  private final CANSparkMax _rightMotor = new CANSparkMax(Constants.CAN.SHOOTER_RIGHT, MotorType.kBrushless);

  private final TalonFX _angleMotor = new TalonFX(Constants.CAN.SHOOTER_ANGLE);
  
  private final RelativeEncoder _leftEncoder = _leftMotor.getEncoder();
  // private final DutyCycleEncoder _angleEncoderAbsolute = new DutyCycleEncoder(Ports.ANGLE_ENCODER);

  private final ArmFeedforward _angleFeed = new ArmFeedforward(0, FeedForward.SHOOTER_ANGLE_KG, 0);
  private final PIDController _angleController = new PIDController(PID.SHOOTER_ANGLE_KP, 0, 0);

  private final Debouncer _beamDebouncer = new Debouncer(0.3, DebounceType.kRising);
  private final Encoder _incremental = new Encoder(1, 2, false, Encoder.EncodingType.k2X);

  private double _shooterTrim = 0;

  private boolean _holdNote = false;

  private final boolean ABSOLUTE_RESET = false;

  /** Represents the state of the shooter's flywheels (speaker shoot, amp, nothing). */
  public enum ShooterState {
    SHOOT,
    AMP,
    INTAKE,
    IDLE,
    NONE
  }

  /** Creates a new ShooterSubsystem. */
  public ShooterSubsystem() {
    NeoConfig.configureNeo(_leftMotor, true);
    NeoConfig.configureFollowerNeo(_rightMotor, _leftMotor, true);

    TalonFXConfig.configureFalcon(_angleMotor, true);

    _incremental.setDistancePerPulse((360.0 / Physical.SHOOTER_ENCODER_ANGLE_GEAR_RATIO)/8192.0);
    _incremental.reset();

    // for resetting absolute angle
    // _angleEncoder.reset();
    // SmartDashboard.putNumber("ENC OFFSET", _angleEncoder.getPositionOffset());

    // _angleEncoderAbsolute.setDutyCycleRange(1.0/1024.0, 1023.0/1024.0);
    // _angleEncoderAbsolute.setDistancePerRotation(360 / Physical.SHOOTER_ENCODER_ANGLE_GEAR_RATIO);
    resetAngle();

    // soft limits
    SoftwareLimitSwitchConfigs softLimits = new SoftwareLimitSwitchConfigs();

    softLimits.ForwardSoftLimitThreshold = 69 * Constants.Physical.SHOOTER_ANGLE_GEAR_RATIO / 360;
    softLimits.ReverseSoftLimitThreshold = -25 * Constants.Physical.SHOOTER_ANGLE_GEAR_RATIO / 360;

    softLimits.ForwardSoftLimitEnable = true;
    softLimits.ReverseSoftLimitEnable = true;

    _angleMotor.getConfigurator().apply(softLimits);

    _angleController.setTolerance(1);

    SmartDashboard.putNumber("SHOOTER TRIM", _shooterTrim);
  }

  @Override
  public void periodic() {
    boolean beamBroken = _beamDebouncer.calculate(true); // TODO: beam break input here
    _holdNote = beamBroken ? true : _holdNote;

    // This method will be called once per scheduler run
    SmartDashboard.putNumber("SHOOTER SETPOINT", _angleController.getSetpoint());
    SmartDashboard.putNumber("SHOOTER ANGLE", getAngle());
    // SmartDashboard.putNumber("SHOOTER ABSOLUTE ENCODER", _angleEncoderAbsolute.getDistance() - 80);
    SmartDashboard.putNumber("SHOOTER PERCENT OUTPUT", _leftMotor.get());
    SmartDashboard.putNumber("SHOOTER INCREMENTAL", _incremental.getDistance());
    SmartDashboard.putBoolean("SHOOTER REVVED", false);

    _shooterTrim = SmartDashboard.getNumber("SHOOTER TRIM", _shooterTrim);
  }

  // for resetting the shooter's angle
  private void resetAngle() {
    double resetAngle;
  
    if (ABSOLUTE_RESET) {
      // resetAngle = _angleEncoderAbsolute.getDistance() - 80;
    } else {
      resetAngle = 0;
    }

    _angleMotor.setPosition(resetAngle * Physical.SHOOTER_ANGLE_GEAR_RATIO / 360);
  }

  /**
   * Get the calculated angle needed to aim at the speaker.
   */
  public double speakerAngle() {
    double distance = UtilFuncs.ShotVector().getNorm();
    double angle = Presets.SHOOTER_DISTANCE_ANGLE.get(distance);

    return angle + _shooterTrim;
  }

  /**
   * Reset the shooter's hold note, allowing it to shoot freely with a note.
   */
  public void resetHoldNote() {
    _holdNote = false;
  }

  /**
   * Boolean for whether the shooter is preventing the note from coming out.
   */
  public boolean holdNote() { 
    return _holdNote;
  }

  /** Returns true if the shooter is at the last desired angle setpoint. */
  public boolean atDesiredAngle() {
    return _angleController.atSetpoint();
  }
 
  /** Set the angle of the shooter in degrees. MUST be called repeatedly. */
  public void setAngle(double angleDegrees) {
    double pid = MathUtil.clamp(
      _angleController.calculate(getAngle(), angleDegrees),
      -Constants.Speeds.SHOOTER_ANGLE_MAX_SPEED,
      Constants.Speeds.SHOOTER_ANGLE_MAX_SPEED
    );

    driveAngle(pid);
  }

  /** Get the angle of the shooter in degrees. */
  public double getAngle() {
    return _angleMotor.getPosition().getValueAsDouble() / Constants.Physical.SHOOTER_ANGLE_GEAR_RATIO * 360;
  }

  /** Returns the angular velocity of the motor. (deg/sec) */
  public double getAngularVelocity(){
    return _angleMotor.getVelocity().getValueAsDouble() / Constants.Physical.SHOOTER_ANGLE_GEAR_RATIO * 360;
  }

  /** Get velocity of the shooter flywheel in encoder val. */
  public double getVelocity() {
    return _leftEncoder.getVelocity();
  }

  /**
   * Drives the angle motors at the desired percent output (feedforward is
   * included).
   */
  public void driveAngle(double speed) {
    double ff = UtilFuncs.FromVolts(_angleFeed.calculate(Math.toRadians(getAngle()), 0));
    System.out.println(ff + speed);
    _angleMotor.set(ff + speed);
    // _angleMotor.set(0);
  }

  /** Stops the shooter's angular movement. */
  public void stopAngle() {
    driveAngle(0);
  }

  /** Sets the state of the shooter. */
  public void setShooterState(ShooterState state) {
    switch (state) {
      case SHOOT:
        if (UtilFuncs.ShotVector().getNorm() > FieldConstants.SHOOTER_SLOW_THRESHOLD) { 
          spinShooter(Speeds.SHOOTER_FAST_SPIN_SPEED); 
        } else {
          spinShooter(Speeds.SHOOTER_SLOW_SPIN_SPEED);
        }
        break;
    
      case AMP:
        spinShooter(Speeds.SHOOTER_AMP_SPEED);
        break;

      case INTAKE:
        spinShooter(Speeds.SHOOTER_INTAKE_SPEED);

      case IDLE:
        spinShooter(0);

      case NONE:
        stopShooter();
        break;

      default:
        break;
    }
  }

  /** Spins the shooter at the specified percent output. */
  public void spinShooter(double speed) {
    _leftMotor.set(speed);
  }


  /** Stops spinning the shooter. */
  public void stopShooter() {
    spinShooter(0);
  }
}
