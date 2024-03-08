/* Copyright (C) 2024 Team 334. All Rights Reserved.*/
package frc.robot.subsystems;

import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Encoders;
import frc.robot.Constants.Physical;
import frc.robot.utils.UtilFuncs;
import frc.robot.utils.configs.TalonFXConfig;

/** @author Peter Gutkovich */
public class ElevatorSubsystem extends SubsystemBase {
  private final TalonFX _leftMotor = new TalonFX(Constants.CAN.ELEVATOR_LEFT);
  private final TalonFX _rightMotor = new TalonFX(Constants.CAN.ELEVATOR_RIGHT);

  private final ElevatorFeedforward _elevatorFeed = new ElevatorFeedforward(0, 0, 0);
  private final ElevatorFeedforward _climbFeed = new ElevatorFeedforward(0, 0, 0); // TODO: Get this value

  private final PIDController _heightController = new PIDController(Constants.PID.ELEVATOR_KP, 0, 0);

  private boolean _usingClimberFeed = false;

  /** Creates a new ElevatorSubsystem . */
  public ElevatorSubsystem() {
    TalonFXConfig.configureFalcon(_leftMotor, true);
    TalonFXConfig.configureFollowerFalcon(_rightMotor, _leftMotor, true);

    SoftwareLimitSwitchConfigs softLimits = new SoftwareLimitSwitchConfigs();

    softLimits.ForwardSoftLimitThreshold = 0.45 * Physical.ELEVATOR_GEAR_RATIO / Physical.ELEVATOR_DISTANCE_PER_ROTATION;
    softLimits.ReverseSoftLimitThreshold = 0;

    softLimits.ForwardSoftLimitEnable = true;
    softLimits.ReverseSoftLimitEnable = true;

    _leftMotor.getConfigurator().apply(softLimits);

    _heightController.setTolerance(0.01);
    
    SmartDashboard.putData("ELEVATOR PID", _heightController);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // harry chen code maybe fix

    SmartDashboard.putNumber("ELEVATOR HEIGHT METERS", getElevatorHeight());
  }

  /**
   * Whether using the climb feed forward or not (different gravity constant).
   * 
   * @param useFeed If true, use climb feed, if false, use elevator feed.
   */
  public void useClimbFeed(boolean useFeed) {
    _usingClimberFeed = !useFeed;
  }
  
  /** Returns true if the elevator is at the last desired height setpoint. */
  public boolean atDesiredHeight() {
    return _heightController.atSetpoint();
  }

  /** Sets the height of the elevator in meters. MUST be called repeatedly. */
  public void setElevatorHeight(double height) {
    // System.out.println(height);

    double out = MathUtil.clamp(
      _heightController.calculate(getElevatorHeight(), height),
      -Constants.Speeds.ELEVATOR_MAX_SPEED,
      Constants.Speeds.ELEVATOR_MAX_SPEED
    );

    System.out.println(out);

    driveElevator(out);
  }

  /** Get the height of the elevator in meters. */
  public double getElevatorHeight() {
    return _leftMotor.getPosition().getValueAsDouble() / Physical.ELEVATOR_GEAR_RATIO * Physical.ELEVATOR_DISTANCE_PER_ROTATION;
  }

  /**
   * Drives the elevator at a desired percent output (feedforward is included).
   */
  public void driveElevator(double speed) {
    double ff = 0;

    if (_usingClimberFeed)
      ff = _climbFeed.calculate(0);
    else {
      ff = _elevatorFeed.calculate(0);
    }

    ff = UtilFuncs.FromVolts(ff);

    _leftMotor.set(ff + speed);
  }

  /** Stops elevator movement. */
  public void stopElevator() {
    // System.out.println("Stopped");
    driveElevator(0);
  }
}
