package frc.robot.utils;


import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;

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
    public static void configureFalcon(TalonFX falcon) {
        TalonFXConfiguration config = new TalonFXConfiguration();

        // TODO: ⬇ GOTTA FIX THIS FOR THE NEW CTRE UPDATE ⬇⬇

        // falcon.configFactoryDefault(Constants.CAN.CAN_TIMEOUT);
        // config.configNeutralDeadband(0.01, Constants.CAN.CAN_TIMEOUT);
        // config.setNeutralMode(NeutralMode.Brake);

        // config.configForwardSoftLimitEnable(false);
        // config.configReverseSoftLimitEnable(false);

        // config.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
        // config.(0);
    }

    /**
     * Configure a master Falcon motor.
     * 
     * @param falcon - The Falcon to config.
     * @param invert - Boolean for whether the Falcon should be inverted or not.
     */
    public static void configureDriveMasterFalcon(TalonFX falcon, boolean invert) {
        // ⬇⬇ GOTTA FIX THIS FOR THE NEW CTRE UPDATE ⬇⬇

        configureFalcon(falcon);
        // falcon.setInverted(invert ? TalonFXInvertType.CounterClockwise : TalonFXInvertType.Clockwise);
        // falcon.setNeutralMode(NeutralMode.Coast);
    }

    /**
     * Configure a slave of a master Falcon motor.
     * 
     * @param falcon - The slave motor to config.
     * @param master - The master motor.
     * @param invert - Boolean for whether the slave move inverted to the master.
     */
    public static void configureDriveFollowerFalcon(TalonFX falcon, TalonFX master, boolean invert) {
        // ⬇⬇ GOTTA FIX THIS FOR THE NEW CTRE UPDATE ⬇⬇

        configureFalcon(falcon);

        // falcon.set(TalonFXControlMode.Follower, master.getDeviceID());
        // falcon.setInverted(invert ? TalonFXInvertType.OpposeMaster : TalonFXInvertType.FollowMaster);
        // falcon.setNeutralMode(NeutralMode.Coast);
    }
}