/* Copyright (C) 2024 Team 334. All Rights Reserved.*/
package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.SoftLimitDirection;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.PID;
import frc.robot.Constants.Speeds;
import frc.robot.utils.configs.NeoConfig;

/**
 * @author Peter Gutkovich
 */
public class IntakeSubsystem extends SubsystemBase {
  private final CANSparkMax _feedMotor, _actuatorMotor;
  private final PIDController _actuatorController = new PIDController(PID.INTAKE_ACTUATE_KP, 0, 0);

  private final RelativeEncoder _actuatorEncoder;

  private boolean _hasNoteAuton = false;
  
  /** How to feed (in or out). */
  public enum FeedMode {
    INTAKE, OUTTAKE, NONE
  }

  /** The actuator state (out or stowed). */
  public enum ActuatorState {
    STOWED, OUT, NONE
  }


  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {
    _feedMotor = new CANSparkMax(Constants.CAN.INTAKE_FEED, MotorType.kBrushless);
    _actuatorMotor = new CANSparkMax(Constants.CAN.INTAKE_ACTUATOR, MotorType.kBrushless);

    _actuatorEncoder = _actuatorMotor.getEncoder();
    _actuatorEncoder.setPosition(0);

    _actuatorController.setTolerance(0.5);

    NeoConfig.configureNeo(_feedMotor, false);
    NeoConfig.configureNeo(_actuatorMotor, false);

    _feedMotor.setSmartCurrentLimit(80);

    _actuatorMotor.setSoftLimit(SoftLimitDirection.kForward, Constants.Encoders.INTAKE_OUT);
    _actuatorMotor.setSoftLimit(SoftLimitDirection.kReverse, Constants.Encoders.INTAKE_STOWED);

    _actuatorMotor.enableSoftLimit(SoftLimitDirection.kForward, true);
    _actuatorMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);
  }

  /**
   * Whether the intake has a squished note, according to the detector in auton.
   */
  public boolean hasNoteAuton() {
    return _hasNoteAuton;
  }

  /**
   * Set whether the intake has a squished note, done by the auton detector.
   */
  public void setHasNoteAuton(boolean hasNoteAuton) {
    _hasNoteAuton = hasNoteAuton;
  }

  /**
   * Returns true if the actuator is at the last desired state.
   */
  public boolean atDesiredActuatorState() {
    return _actuatorController.atSetpoint();
  }

  /**
   * Get the actuator's encoder position.
   */
  public double getActuator() {
    return _actuatorEncoder.getPosition();
  }

  /**
   * Actuate the intake at a given speed. Speed gets clamped.
   */
  public void actuate(double speed) {
    _actuatorMotor.set(MathUtil.clamp(
      speed,
      -Speeds.INTAKE_ACTUATE_MAX_SPEED,
      Speeds.INTAKE_ACTUATE_MAX_SPEED
    ));
  }

  /**
   * Set the actuator's state. MUST BE CALLED REPEATEDLY.
   *
   * @param actuatorState
   *            The state to set the actuator to.
   */
  public void actuate(ActuatorState actuatorState) {    
    switch (actuatorState) {
      case STOWED :
        actuate(_actuatorController.calculate(getActuator(), Constants.Encoders.INTAKE_STOWED));
        break;

      case OUT :
        actuate(_actuatorController.calculate(getActuator(), Constants.Encoders.INTAKE_OUT));
        break;

      case NONE:
        actuate(0);
        break;
      
      default:
        break;
    }
  }

  /**
   * Feed in/out of the intake.
   *
   * @param feedMode
   *            How to feed.
   */
  public void feed(FeedMode feedMode) {
    switch (feedMode) {
      case INTAKE :
        _feedMotor.set(Constants.Speeds.INTAKE_FEED_SPEED);
        break;

      case OUTTAKE :
        _feedMotor.set(Constants.Speeds.OUTTAKE_FEED_SPEED);
        break;

      case NONE :
        _feedMotor.set(0);
        break;

      default :
        break;
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("ACTUATOR ENCODER", getActuator());
    SmartDashboard.putNumber("ACTUATOR PERCENT OUTPUT", _actuatorMotor.get());
    SmartDashboard.putNumber("FEED CURRENT OUTPUT", _feedMotor.getOutputCurrent());
  }
}
