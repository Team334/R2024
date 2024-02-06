/* Copyright (C) 2024 Team 334. All Rights Reserved.*/
package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.utils.configs.NeoConfig;

/**
 * @author Peter Gutkovich
 */
public class IntakeSubsystem extends SubsystemBase {
  public final CANSparkMax _feedMotor, _actuatorMotor;
  private final PIDController _actuatorController = new PIDController(0, 0, 0);

  private final RelativeEncoder _actuatorEncoder;

  /** How to feed (in or out). */
  public enum FeedMode {
    INTAKE, OUTTAKE, NONE
  }

  /** The actuator state (out or stowed). */
  public enum ActuatorState {
    STOWED, OUT
  }

  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {
    _feedMotor = new CANSparkMax(Constants.CAN.INTAKE_FEED, MotorType.kBrushless);
    _actuatorMotor = new CANSparkMax(Constants.CAN.INTAKE_ACTUATOR, MotorType.kBrushless);

    _actuatorEncoder = _actuatorMotor.getEncoder();

    NeoConfig.configureNeo(_feedMotor, false);
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
   * Set the actuator's state. MUST BE CALLED REPEATEDLY.
   *
   * @param actuatorState
   *            The state to set the actuator to.
   */
  public void actuate(ActuatorState actuatorState) {
    switch (actuatorState) {
      case STOWED :
        _actuatorMotor.set(_actuatorController.calculate(getActuator(), Constants.Encoders.INTAKE_STOWED));
        break;

      case OUT :
        _actuatorMotor.set(_actuatorController.calculate(getActuator(), Constants.Encoders.INTAKE_OUT));
        break;

      default :
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
        _feedMotor.set(-Constants.Speeds.INTAKE_FEED_SPEED);
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
  }
}
