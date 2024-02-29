/* Copyright (C) 2024 Team 334. All Rights Reserved.*/
package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkBase.SoftLimitDirection;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.utils.UtilFuncs;
import frc.robot.utils.configs.NeoConfig;

/** @author Peter Gutkovich */
public class ElevatorSubsystem extends SubsystemBase {
  private final CANSparkMax _leftMotor = new CANSparkMax(Constants.CAN.ELEVATOR_LEFT, MotorType.kBrushless);
  private final CANSparkMax _rightMotor = new CANSparkMax(Constants.CAN.ELEVATOR_RIGHT, MotorType.kBrushless);

  private final RelativeEncoder _leftEncoder;

  private final ElevatorFeedforward _elevatorFeed = new ElevatorFeedforward(0, 0, 0);
  private final ElevatorFeedforward _climbFeed = new ElevatorFeedforward(0, 0, 0); // TODO: Get this value

  private final PIDController _heightController = new PIDController(Constants.PID.ELEVATOR_KP, 0, 0);

  private boolean _usingClimberFeed = false;

  /** Creates a new ElevatorSubsystem . */
  public ElevatorSubsystem() {
    NeoConfig.configureNeo(_leftMotor, true);
    NeoConfig.configureFollowerNeo(_rightMotor, _leftMotor, true);

    _leftEncoder = _leftMotor.getEncoder();
    _leftEncoder.setPosition(0);

    _leftMotor.setSoftLimit(SoftLimitDirection.kForward, 85);
    _leftMotor.setSoftLimit(SoftLimitDirection.kReverse, 0);

    _leftMotor.enableSoftLimit(SoftLimitDirection.kForward, false);
    _leftMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);

    _heightController.setTolerance(0.5);

    SmartDashboard.putData("ELEVATOR PID", _heightController);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // harry chen code maybe fix

    SmartDashboard.putNumber("ELEVATOR ENCODER HEIGHT", getElevatorHeight());
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

  /** Sets the height of the elevator in encoder val. MUST be called repeatedly. */
  public void setElevatorHeight(double height) {
    double out = MathUtil.clamp(
      _heightController.calculate(getElevatorHeight(), height),
      -Constants.Speeds.ELEVATOR_MAX_SPEED,
      Constants.Speeds.ELEVATOR_MAX_SPEED
    );

    driveElevator(out);
  }

  /** Get the height of the elevator encoder val. */
  public double getElevatorHeight() {
    return _leftEncoder.getPosition();
  }

  /**
   * Drives the elevator at a desired percent output (feedforward is included).
   */
  public void driveElevator(double speed) {

    // double out = 0;

    // if (_usingClimberFeed)
    //   out = _climbFeed.calculate(0);
    // else {
    //   out = _elevatorFeed.calculate(0);
    // }

    _leftMotor.set(speed);
  }

  /** Stops elevator movement. */
  public void stopElevator() {
    // System.out.println("Stopped");
    driveElevator(0);
  }

  public void changeElevatorFeed() {
    _usingClimberFeed = !_usingClimberFeed;
  }
}
