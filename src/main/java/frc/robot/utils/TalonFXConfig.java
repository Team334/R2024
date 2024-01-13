package frc.robot.utils;


import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.Constants;

/**
 * For configuring Falcons.
 */
public class TalonFXConfig {
    /**
     * Basic Falcon config, sets Falcon to factory defaults, sets encoder to 0, 
     * and sets Falcon deadband and sets Falcon to Brake neutral mode.
     * 
     * @param falcon - The Falcon to config.
     */
    public static TalonFXConfiguration configureFalcon(TalonFX falcon, boolean invert) {
        TalonFXConfiguration config = new TalonFXConfiguration();

        falcon.getConfigurator().DefaultTimeoutSeconds = Constants.CAN.CAN_TIMEOUT;
        falcon.getConfigurator().apply(config); // FACTORY RESET
        
        
        config.MotorOutput.DutyCycleNeutralDeadband = 0.01;
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        config.MotorOutput.Inverted = invert ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;

        config.SoftwareLimitSwitch.ForwardSoftLimitEnable = false;
        config.SoftwareLimitSwitch.ReverseSoftLimitEnable = false;

        config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;

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
    public static void configureDriveFollowerFalcon(TalonFX falcon, TalonFX master, boolean opposeMaster) {
        configureFalcon(falcon, false);
        falcon.setControl(new Follower(master.getDeviceID(), opposeMaster));
    }
}