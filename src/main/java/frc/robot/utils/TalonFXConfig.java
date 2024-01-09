package frc.robot.utils;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

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
        falcon.configFactoryDefault(Constants.CAN.CAN_TIMEOUT);
        falcon.configNeutralDeadband(0.01, Constants.CAN.CAN_TIMEOUT);
        falcon.setNeutralMode(NeutralMode.Brake);

        falcon.configForwardSoftLimitEnable(false);
        falcon.configReverseSoftLimitEnable(false);

        falcon.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
        falcon.setSelectedSensorPosition(0);
    }

    /**
     * Configure a master Falcon motor.
     * 
     * @param falcon - The Falcon to config.
     * @param invert - Boolean for whether the Falcon should be inverted or not.
     */
    public static void configureDriveMasterFalcon(TalonFX falcon, boolean invert) {
        configureFalcon(falcon);
        falcon.setInverted(invert ? TalonFXInvertType.CounterClockwise : TalonFXInvertType.Clockwise);
        falcon.setNeutralMode(NeutralMode.Coast);
    }

    /**
     * Configure a slave of a master Falcon motor.
     * 
     * @param falcon - The slave motor to config.
     * @param master - The master motor.
     * @param invert - Boolean for whether the slave move inverted to the master.
     */
    public static void configureDriveFollowerFalcon(TalonFX falcon, TalonFX master, boolean invert) {
        configureFalcon(falcon);
        falcon.set(TalonFXControlMode.Follower, master.getDeviceID());
        falcon.setInverted(invert ? TalonFXInvertType.OpposeMaster : TalonFXInvertType.FollowMaster);
        falcon.setNeutralMode(NeutralMode.Coast);
    }
}