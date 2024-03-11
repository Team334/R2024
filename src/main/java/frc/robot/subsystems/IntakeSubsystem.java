/* Copyright (C) 2024 Team 334. All Rights Reserved.*/
package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.SoftLimitDirection;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
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
  // private final ProfiledPIDController _actuatorController = new ProfiledPIDController(
  //   0.05,
  //   0,
  //   0,
  //   new TrapezoidProfile.Constraints(22, 50)
  // );

  private final RelativeEncoder _actuatorEncoder;
  private final RelativeEncoder _feedEncoder;

  /** How to feed (in or out). */
  public enum FeedMode {
    INTAKE, OUTTAKE, NONE
  }

  /** The actuator state (out or stowed). */
  public enum ActuatorState {
    STOWED, OUT, NONE
  }

  private FeedMode _feedMode = FeedMode.NONE;
  private ActuatorState _actuatorState = ActuatorState.NONE;

  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {
    _feedMotor = new CANSparkMax(Constants.CAN.INTAKE_FEED, MotorType.kBrushless);
    _actuatorMotor = new CANSparkMax(Constants.CAN.INTAKE_ACTUATOR, MotorType.kBrushless);

    _actuatorEncoder = _actuatorMotor.getEncoder();
    _actuatorEncoder.setPosition(0);

    _feedEncoder = _feedMotor.getEncoder();

    _actuatorController.setTolerance(0.5);

    NeoConfig.configureNeo(_feedMotor, false);
    NeoConfig.configureNeo(_actuatorMotor, false);

    _actuatorMotor.setSoftLimit(SoftLimitDirection.kForward, Constants.Encoders.INTAKE_OUT);
    _actuatorMotor.setSoftLimit(SoftLimitDirection.kReverse, Constants.Encoders.INTAKE_STOWED);

    _actuatorMotor.enableSoftLimit(SoftLimitDirection.kForward, true);
    _actuatorMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);
  }

  // /** 
  //  * Creates a safety if the intake is moving against a note.
  //  * 
  //  * @return True if the intake is moving against a note, else False.
  //  */

  // // TODO: why not working?
  // public boolean noteSafety() {
  //   SmartDashboard.putNumber("FEED OUTPUT", Math.abs(_feedMotor.get()));
  //   SmartDashboard.putNumber("FEED VEL", Math.abs(_feedEncoder.getVelocity()));

  //   if (Math.abs(_feedMotor.get()) > 0 && Math.abs(_feedEncoder.getVelocity()) < 2) {
  //     return true;
  //   }

  //   return false;
  // }

  public boolean isFeedMode(FeedMode feedMode) {
    return _feedMode == feedMode;
  }

  public boolean isActuatorState(ActuatorState actuatorState) {
    return _actuatorState == actuatorState;
  }
  
  /**
   * Disables the reverse soft limit of the actuator.
   */
  public void disableReverseSoftLimit() {
    _actuatorMotor.enableSoftLimit(SoftLimitDirection.kReverse, false);
  }

  /**
   * Resets the reverse soft limit (and encoder) of the actuator.
   */
  public void resetReverseSoftLimit() {
    _actuatorEncoder.setPosition(0);
    _actuatorMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);
  }

  /**
   * Returns true if the actuator is at the last desired state.
   */
  public boolean atDesiredActuatorState() {
    if (_actuatorController.atSetpoint()) System.out.println("s" + _actuatorController.getSetpoint());

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
    _actuatorState = actuatorState;

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

  // public void setAngle(double angle){
  //   double out = 0;

  //    out = MathUtil.clamp(
  //         _actuatorController.calculate(getActuator(), angle),
  //         -Constants.Speeds.INTAKE_ACTUATE_MAX_SPEED,
  //         Constants.Speeds.INTAKE_ACTUATE_MAX_SPEED);

  //   _actuatorMotor.set(out);
  // }

  /**
   * Feed in/out of the intake.
   *
   * @param feedMode
   *            How to feed.
   */
  public void feed(FeedMode feedMode) {
    _feedMode = feedMode;

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
    SmartDashboard.putNumber("ACTUATOR ENCODER", _actuatorEncoder.getPosition());
    SmartDashboard.putData("ACTUATOR PID", _actuatorController);
    SmartDashboard.putNumber("ACTUATOR OUT", _actuatorMotor.get());

    // SmartDashboard.putBoolean("NOTE SAFETY", noteSafety());

    // if (noteSafety()) { feed(FeedMode.NONE); }
  }
}
