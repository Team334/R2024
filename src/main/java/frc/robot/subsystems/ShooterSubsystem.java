/* Copyright (C) 2024 Team 334. All Rights Reserved.*/
package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.FeedForward;
import frc.robot.Constants.PID;
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

  private final ArmFeedforward _angleFeed = new ArmFeedforward(0, 0, 0);
  private final PIDController _angleController = new PIDController(PID.SHOOTER_ANGLE_KP, 0, 0);

  /** Represents the state of the shooter's flywheels (speaker shoot, amp, nothing). */
  public enum ShooterState {
    SHOOT,
    AMP,
    INTAKE,
    NONE
  }

  /** Creates a new ShooterSubsystem. */
  public ShooterSubsystem() {
    NeoConfig.configureNeo(_leftMotor, true);
    NeoConfig.configureFollowerNeo(_rightMotor, _leftMotor, true);

    TalonFXConfig.configureFalcon(_angleMotor, true);
    _angleMotor.setPosition(0  * Constants.Physical.SHOOTER_ANGLE_GEAR_RATIO / 360);
    // _angleMotor.setPosition(0);

    // soft limits
    SoftwareLimitSwitchConfigs softLimits = new SoftwareLimitSwitchConfigs();

    softLimits.ForwardSoftLimitThreshold = 69 * Constants.Physical.SHOOTER_ANGLE_GEAR_RATIO / 360;
    softLimits.ReverseSoftLimitThreshold = -25 * Constants.Physical.SHOOTER_ANGLE_GEAR_RATIO / 360;

    softLimits.ForwardSoftLimitEnable = true;
    softLimits.ReverseSoftLimitEnable = true;

    _angleMotor.getConfigurator().apply(softLimits);

    _angleController.setTolerance(0.2);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("SHOOTER ANGLE", getAngle());
    SmartDashboard.putNumber("SHOOTER PERCENT OUTPUT", _leftMotor.get());
  }

  /** Returns true if the shooter is at the last desired angle setpoint. */
  public boolean atDesiredAngle() {
    return _angleController.atSetpoint();
  }
 
  /** Set the angle of the shooter in degrees. MUST be called repeatedly. */
  public void setAngle(double angleDegrees) {
    double pid = MathUtil.clamp(
      _angleController.calculate(getAngle(), angleDegrees), // test
      -Constants.Speeds.SHOOTER_ANGLE_MAX_SPEED,
      Constants.Speeds.SHOOTER_ANGLE_MAX_SPEED
    );

    driveAngle(pid);
  }

  /** Get the angle of the shooter in degrees. */
  public double getAngle() {
    return _angleMotor.getPosition().getValueAsDouble() / Constants.Physical.SHOOTER_ANGLE_GEAR_RATIO * 360;
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
    _angleMotor.set(UtilFuncs.FromVolts(_angleFeed.calculate(Math.toRadians(getAngle()), 0)) + speed);
    // _angleMotor.set(speed);
  }

  /** Stops the shooter's angular movement. */
  public void stopAngle() {
    driveAngle(0);
  }

  /** Sets the state of the shooter. */
  public void setShooterState(ShooterState state) {
    switch (state) {
      case SHOOT:
        if (UtilFuncs.ShootFast()) { 
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
