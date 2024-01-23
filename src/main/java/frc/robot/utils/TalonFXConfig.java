/*                                  Team 334                                  */
/* Copyright (c) 2024 Team 334. All Rights Reserved.                          */

package frc.robot.utils;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.Constants;

/**
 * @author Jerry Zheng
 * @author Peter Gutkovich
 */

/** For configuring Falcons. */
public class TalonFXConfig {
  /**
   * Basic Falcon config, sets Falcon to factory defaults, sets encoder to 0, and sets Falcon
   * deadband and sets Falcon to Coast neutral mode.
   *
   * @param falcon - The Falcon to config.
   *
   * @return The configuration object applied to the Falcon.
   */
  public static TalonFXConfiguration configureFalcon(TalonFX falcon, boolean invert) {
    // TODO: will prob need to add the code to zero encoder
    TalonFXConfiguration config = new TalonFXConfiguration();

    falcon.getConfigurator().DefaultTimeoutSeconds = Constants.CAN.CAN_TIMEOUT;
    falcon.getConfigurator().refresh(config);

    config.MotorOutput.DutyCycleNeutralDeadband = 0.01;
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    config.MotorOutput.Inverted =
        invert ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;

    config.SoftwareLimitSwitch.ForwardSoftLimitEnable = false;
    config.SoftwareLimitSwitch.ReverseSoftLimitEnable = false;

    // config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;

    falcon.getConfigurator().apply(config);

    return config;
  }

  /**
   * Configure a follower of a master Falcon motor.
   *
   * @param falcon - The follower motor to config.
   * @param master - The master motor.
   * @param opposeMaster - Boolean for whether the follower motor inverted to the master.
   */
  public static void configureFollowerFalcon(
      TalonFX falcon, TalonFX master, boolean opposeMaster) {
    configureFalcon(falcon, false);
    falcon.setControl(new Follower(master.getDeviceID(), opposeMaster));
  }
}
