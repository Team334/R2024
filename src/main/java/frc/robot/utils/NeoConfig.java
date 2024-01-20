/*                                  Team 334                                  */
/* Copyright (c) 2024 Team 334. All Rights Reserved.                          */

package frc.robot.utils;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkBase.SoftLimitDirection;


/**
 * @author Ze Rui Zheng
 * @author Elvis Osmanov
 */
public class NeoConfig {
    public static void configureNeo(CANSparkMax neo, boolean invert) {
        neo.setIdleMode(IdleMode.kCoast);
        neo.enableSoftLimit(SoftLimitDirection.kForward, false);
        neo.enableSoftLimit(SoftLimitDirection.kReverse, false);
        neo.setInverted(invert);
    }
    public static void configureFollowerNeo(CANSparkMax neo, CANSparkMax master, boolean opposeMaster) {
        configureNeo(neo, false);
        neo.follow(master);
    }
}
