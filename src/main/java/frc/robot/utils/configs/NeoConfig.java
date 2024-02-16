/* Copyright (C) 2024 Team 334. All Rights Reserved.*/
package frc.robot.utils.configs;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkMax;

/**
 * @author Ze Rui Zheng
 * @author Elvis Osmanov
 */

/** For configuring Neos (with CANSparkMax). */
public class NeoConfig {
  /**
   * Basic Neo (with CANSparkMax) config, sets Falcon to factory defaults, sets
   * encoder to 0, and sets Neo to Brake neutral mode.
   *
   * @param neo
   *            - The CANSparkMax (with Neo) to configure.
   * @param invert
   *            - Whether to invert the motor or not.
   */
  public static void configureNeo(CANSparkMax neo, boolean invert) {
    neo.restoreFactoryDefaults();
    
    neo.setCANTimeout(10);

    neo.setIdleMode(IdleMode.kBrake);

    neo.enableSoftLimit(SoftLimitDirection.kForward, false);
    neo.enableSoftLimit(SoftLimitDirection.kReverse, false);

    neo.setInverted(invert);
  }

  /**
   * Configure a follower of a master Neo motor.
   *
   * @param follower
   *            - The Neo (with CANSparkMax) to config.
   * @param master
   *            - The master motor.
   * @param opposeMaster
   *            - Whether to oppose the master or not.
   */
  public static void configureFollowerNeo(CANSparkMax follower, CANSparkMax master, boolean opposeMaster) {
    configureNeo(follower, false);
    follower.follow(master, opposeMaster);
  }
}
