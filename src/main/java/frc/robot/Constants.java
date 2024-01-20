/*                                  Team 334                                  */
/* Copyright (c) 2024 Team 334. All Rights Reserved.                          */

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class CAN {
    public static final int DRIVE_FRONT_LEFT = 1;
    public static final int ROT_FRONT_LEFT = 2;

    public static final int DRIVE_FRONT_RIGHT = 3;
    public static final int ROT_FRONT_RIGHT = 4;

    public static final int DRIVE_BACK_RIGHT = 5;
    public static final int ROT_BACK_RIGHT = 6;

    public static final int DRIVE_BACK_LEFT = 7;
    public static final int ROT_BACK_LEFT = 8;

    public static final int ENC_FRONT_LEFT = 9;
    public static final int ENC_FRONT_RIGHT = 10;
    public static final int ENC_BACK_LEFT = 12;
    public static final int ENC_BACK_RIGHT = 11;

    public static final int SHOOTER_LEFT = 13;
    public static final int SHOOTER_RIGHT = 14;

    public static final int CAN_TIMEOUT = 10;
  }

  public static class Speeds {
    public static final double SWERVE_DRIVE_COEFF = 1;

    public static final double SWERVE_PID_KP = 0.012;

    public static final double SWERVE_DRIVE_MAX_SPEED = 4.67; // TODO: Get this value
    public static final double SWERVE_DRIVE_MAX_ANGULAR_SPEED = Math.PI * 1; // Todo: Get this value

    public static final double SHOOTER_MAX_SPEED = 1;
  }

  public static class Physical {
    // GEAR RATIOS ARE: DRIVEN GEAR TEETH / DRIVING GEAR TEETH

    public static final double SWERVE_DRIVE_BASE_RADIUS = 0.43;

    public static final double SWERVE_DRIVE_GEAR_RATIO = 6.75;
    public static final double SWERVE_DRIVE_WHEEL_RADIUS = 0.05;
    public static final double SWERVE_DRIVE_WHEEL_CIRCUMFERENCE = 2 * Math.PI * SWERVE_DRIVE_WHEEL_RADIUS;

    public static final double SHOOTER_GEAR_RATIO = 1.45;
    public static final double SHOOTER_FLYWHEEL_RADIUS = 1; // TODO: FIND RADIUS
    public static final double SHOOTER_FLYWHEEL_CIRCUMFERENCE = 2 * Math.PI * SHOOTER_FLYWHEEL_RADIUS;


    public static final double TALON_TICKS_PER_REVOLUTION = 2048;

    public static final SwerveDriveKinematics SWERVE_KINEMATICS = new SwerveDriveKinematics(
      new Translation2d(0.292, 0.292),
      new Translation2d(0.292, -0.292),
      new Translation2d(-0.292, -0.292),
      new Translation2d(-0.292, 0.292)
    );

    public static final double SHOOTER_PID_KP = 0;
  }

  public static class Offsets {
    public static final double ENCODER_FRONT_LEFT = -93;
    public static final double ENCODER_FRONT_RIGHT = -58;
    public static final double ENCODER_BACK_RIGHT = 10;
    public static final double ENCODER_BACK_LEFT = 43;
  }

  public static class Ports {
    public static final int DRIVER_CONTROLLER = 0;
  }
}
