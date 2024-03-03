/* Copyright (C) 2024 Team 334. All Rights Reserved.*/
package frc.robot;

import com.pathplanner.lib.util.PIDConstants;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final Alliance SAFE_ALLIANCE = Alliance.Red;

  public static class CAN {
    public static final int DRIVE_FRONT_LEFT = 5;
    public static final int ROT_FRONT_LEFT = 6;

    public static final int DRIVE_FRONT_RIGHT = 7;
    public static final int ROT_FRONT_RIGHT = 8;

    public static final int DRIVE_BACK_RIGHT = 1;
    public static final int ROT_BACK_RIGHT = 2;

    public static final int DRIVE_BACK_LEFT = 3;
    public static final int ROT_BACK_LEFT = 4;

    public static final int ENC_FRONT_LEFT = 11;
    public static final int ENC_FRONT_RIGHT = 12;
    public static final int ENC_BACK_LEFT = 10;
    public static final int ENC_BACK_RIGHT = 9;

    public static final int SHOOTER_LEFT = 13;
    public static final int SHOOTER_RIGHT = 14;
    public static final int SHOOTER_ANGLE = 15;

    public static final int ELEVATOR_LEFT = 16; // TODO: Get right can ID's
    public static final int ELEVATOR_RIGHT = 17;

    public static final int INTAKE_ACTUATOR = 18;
    public static final int INTAKE_FEED = 19;

    public static final int CAN_TIMEOUT = 10;
  }

  public static class Speeds {
    public static final double SWERVE_DRIVE_COEFF = .4;

    public static final double SWERVE_DRIVE_MAX_SPEED = 4.67; // meters per second
    public static final double SWERVE_DRIVE_MAX_ANGULAR_SPEED = Math.PI * 1; // TODO: Get this value

    public static final double SHOOTER_SPIN_SPEED = 1; // TODO: Get this
    public static final double SHOOTER_AMP_SPEED = 0.2;

    public static final double SHOOTER_ANGLE_MAX_SPEED = 0.3;
    public static final double ELEVATOR_MAX_SPEED = 0.2;

    public static final double INTAKE_FEED_SPEED = 0.25; // TODO: Get this
    public static final double OUTTAKE_FEED_SPEED = -0.4;

    public static final double INTAKE_ACTUATE_MAX_SPEED = 0.4;
  }

  public static class Physical {
    // GEAR RATIOS ARE: DRIVEN GEAR TEETH / DRIVING GEAR TEETH

    public static final double SWERVE_DRIVE_BASE_RADIUS = 0.43;

    public static final double SWERVE_DRIVE_GEAR_RATIO = 6.75;
    public static final double SWERVE_DRIVE_WHEEL_RADIUS = 0.05;
    public static final double SWERVE_DRIVE_WHEEL_CIRCUMFERENCE = 2 * Math.PI * SWERVE_DRIVE_WHEEL_RADIUS;

    public static final double SHOOTER_ANGLE_GEAR_RATIO = 112;
    public static final double SHOOTER_HEIGHT_STOWED = 0.2; // TODO: Get this value

    public static final SwerveDriveKinematics SWERVE_KINEMATICS = new SwerveDriveKinematics(
        new Translation2d(0.292, 0.292), new Translation2d(0.292, -0.292), new Translation2d(-0.292, -0.292),
        new Translation2d(-0.292, 0.292));
  }

  public static class Encoders {
    public static final int INTAKE_STOWED = 0;
    public static final int INTAKE_OUT = 15;
  }

  public static class FeedForward {
    public static final double ELEVATOR_KG = 0.0;

    public static final double MODULE_DRIVE_KS = 0.32;
    public static final double MODULE_DRIVE_KV = 2.15;

    public static final double SHOOTER_ANGLE_KG = 0.001;
  }

  public static class PID {
    // TODO: Tune everything

    public static final double MODULE_DRIVE_KP = 0.05;
    public static final double MODULE_ROTATION_KP = 0.009;

    public static final double SHOOTER_ANGLE_KP = 0.08;

    public static final double INTAKE_ACTUATE_KP = 0.025;

    // these are all the same, so the two constants above are used instead
    // public static final double FRONT_LEFT_DRIVE_KP = 0.05;
    // public static final double FRONT_RIGHT_DRIVE_KP = 0.05;
    // public static final double BACK_RIGHT_DRIVE_KP = 0.05;
    // public static final double BACK_LEFT_DRIVE_KP = 0.05;

    // public static final double FRONT_LEFT_ROTATE_KP = 0.009;
    // public static final double FRONT_RIGHT_ROTATE_KP = 0.009;
    // public static final double BACK_RIGHT_ROTATE_KP = 0.009;
    // public static final double BACK_LEFT_ROTATE_KP = 0.009;

    public static final PIDConstants PP_TRANSLATION = new PIDConstants(3.69, 0, 0);
    public static final PIDConstants PP_ROTATION = new PIDConstants(1.21993, 0, 0);

    public static final double SHOOTER_FLYWHEEL_KP = 0;

    public static final double ELEVATOR_KP = 0.01;

    public static final double SWERVE_HEADING_KP = 0.045;
    public static final double SWERVE_HEADING_KD = 0.001;
  }

  public static class Offsets {
    // these aren't used anymore because cancoders can be zeroed in phoenix tuner
    // public static final double ENCODER_FRONT_LEFT = -93;
    // public static final double ENCODER_FRONT_RIGHT = -58;
    // public static final double ENCODER_BACK_RIGHT = 10;
    // public static final double ENCODER_BACK_LEFT = 43;

    // public static final double APRILTAG_SPEAKER_OFFSET = 0.565; // <- Below, but in Meters
    // 200(approx height of spk opening) - 132(Height of AprTag) + 11.5(Center of
    // AprTag) <- CM
  }

  public static class Ports {
    public static final int DRIVER_CONTROLLER = 0;
    public static final int OPERATOR_CONTROLLER = 1;
    public static final int LEDS = 0; // pwm port
  }

  // static field constants
  public static class FieldConstants {
    public static final double SPEAKER_HEIGHT = 2;

    public static final int SPEAKER_TAG_BLUE = 7;
    public static final int SPEAKER_TAG_RED = 4;

    public static final Pose3d SPEAKER_POSE_BLUE = new Pose3d(0.25, 5.5, SPEAKER_HEIGHT, new Rotation3d(0, 0, 180));
    public static final Pose3d SPEAKER_POSE_RED = new Pose3d(16.3, 5.5, SPEAKER_HEIGHT, new Rotation3d(0, 0, 0));
  }

  public static class LEDColors {
    public static final int[] NOTHING = {0, 0, 0};
    public static final int[] GREEN = {0, 255, 0};
    public static final int[] ORANGE = {255, 175, 0};
    public static final int[] YELLOW = {255, 255, 0};
    public static final int[] BLUE = {0, 0, 255};
    public static final int[] RED = {255, 0, 0};
  }
}
