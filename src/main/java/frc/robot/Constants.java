/* Copyright (C) 2024 Team 334. All Rights Reserved.*/
package frc.robot;

import com.pathplanner.lib.util.PIDConstants;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
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
    public static final int ENC_BACK_LEFT = 11;
    public static final int ENC_BACK_RIGHT = 12;

    public static final int SHOOTER_LEFT = 14;
    public static final int SHOOTER_RIGHT = 13;
    public static final int SHOOTER_ANGLE = 15; // make sure settings are right. something got messed up when trying to fix intake sparkmax

    public static final int ELEVATOR_LEFT = 16; 
    public static final int ELEVATOR_RIGHT = 17;

    public static final int INTAKE_ACTUATOR = 18; // double check this because we are switching out for a new sparkmax
    public static final int INTAKE_FEED = 19;

    public static final int CAN_TIMEOUT = 10;
  }

  public static class Speeds {
    public static final double SWERVE_DRIVE_SLOW_COEFF = .6; // Default driving speed
    public static final double SWERVE_DRIVE_FAST_COEFF = .95;

    // public static final double SWERVE_DRIVE_MAX_SPEED = 4;
    public static final double SWERVE_DRIVE_MAX_ANGULAR_SPEED = Math.PI * 2.5; // TODO: Get this value

    public static final double SWERVE_DRIVE_MAX_SPEED = 4.6;

    public static final double SHOOTER_FAST_SPIN_SPEED = 1;
    public static final double SHOOTER_SLOW_SPIN_SPEED = 0.8;
    public static final double SHOOTER_AMP_SPEED = 1;
    public static final double SHOOTER_AMP_SLOW_SPEED = 0.5;
    public static final double SHOOTER_INTAKE_SPEED = -0.15;
    public static final double SHOOTER_IDLE_SPEED = 0.3;

    public static final double SHOOTER_ANGLE_MAX_SPEED = 0.3;
    public static final double ELEVATOR_MAX_SPEED = 1;

    public static final double INTAKE_FEED_SPEED = 0.6;
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
    public static final double SHOOTER_ENCODER_ANGLE_GEAR_RATIO = 2.8;

    public static final double ELEVATOR_GEAR_RATIO = 15;
    public static final double ELEVATOR_DISTANCE_PER_ROTATION = .09;

    public static final SwerveDriveKinematics SWERVE_KINEMATICS = new SwerveDriveKinematics(
      new Translation2d(0.292, 0.292), 
      new Translation2d(0.292, -0.292), 
      new Translation2d(-0.292, -0.292),
      new Translation2d(-0.292, 0.292)
    );
  }

  public static class Encoders {
    public static final int INTAKE_STOWED = 0;
    public static final int INTAKE_OUT = 16;
  }

  public static class FeedForward {
    public static final double ELEVATOR_KG = 0.0;
    public static final double ELEVATOR_KS = 0.3;

    public static final double MODULE_DRIVE_KS = 0.3;
    public static final double MODULE_DRIVE_KV = 2.6;

    public static final double SHOOTER_ANGLE_KG = 0.001;
  }

  public static class PID {
    public static final double MODULE_DRIVE_KP = 0.015;
    public static final double MODULE_ROTATION_KP = 0.009;

    public static final double SHOOTER_ANGLE_KP = 0.04;

    public static final double INTAKE_ACTUATE_KP = 0.08;

    public static final PIDConstants PP_TRANSLATION = new PIDConstants(3.69, 0, 0);
    public static final PIDConstants PP_ROTATION = new PIDConstants(1.21993, 0, 0);

    public static final double SHOOTER_FLYWHEEL_KP = 0;

    public static final double ELEVATOR_KP = 5.2;
    public static final double ELEVATOR_KD = 0.2;

    public static final double NOTE_HEADING_KP = 0.1;

    public static final double SWERVE_HEADING_KP = 0.15;
    public static final double SWERVE_HEADING_KD = 0.01;
  }


  public static class Presets {
    public static final double CLOSE_SHOOTER_ANGLE = 50.12;
    public static final double CLOSE_ELEVATOR_HEIGHT = 0.06;

    public static final double ACTUATE_SHOOTER_ANGLE = 52;

    public static final double ELEVATOR_HEIGHT_RATE = -0.025;

    public static final double SHOOTER_AMP_HANDOFF = 50;
    public static final double ELEVATOR_AMP_HANDOFF = 0.045;

    public static final InterpolatingDoubleTreeMap SHOOTER_DISTANCE_ANGLE = new InterpolatingDoubleTreeMap();
    
    static {
      SHOOTER_DISTANCE_ANGLE.put(0.735, 57.50);
      SHOOTER_DISTANCE_ANGLE.put(1.755, 43.78);
      SHOOTER_DISTANCE_ANGLE.put(1.823, 43.44);
      SHOOTER_DISTANCE_ANGLE.put(2.543, 38.15);
      SHOOTER_DISTANCE_ANGLE.put(3.149, 35.13);
      SHOOTER_DISTANCE_ANGLE.put(3.500, 28.27);
      SHOOTER_DISTANCE_ANGLE.put(3.807, 33.56);
    };

    public static final InterpolatingDoubleTreeMap ELEVATOR_DISTANCE_HEIGHT = new InterpolatingDoubleTreeMap();

    static {
      ELEVATOR_DISTANCE_HEIGHT.put(0.735, 0.078);
      ELEVATOR_DISTANCE_HEIGHT.put(1.755, 0.061);
      ELEVATOR_DISTANCE_HEIGHT.put(1.823, 0.031);
      ELEVATOR_DISTANCE_HEIGHT.put(2.543, 0.024);
      ELEVATOR_DISTANCE_HEIGHT.put(3.149, 0.029);
      ELEVATOR_DISTANCE_HEIGHT.put(3.500, 0.010);
      ELEVATOR_DISTANCE_HEIGHT.put(3.807, 0.009);
    }
  }

  public static class Ports {
    public static final int DRIVER_CONTROLLER = 0;
    public static final int OPERATOR_CONTROLLER = 1;
    public static final int LEDS = 1; // pwm port
  }

  // static field constants
  public static class FieldConstants {
    // z: 2.2456 (USE THIS)
    public static final double SPEAKER_HEIGHT = 2.13;

    public static final double SHOOTER_SLOW_THRESHOLD = 2;

    // public static final double TAG_DISTANCE_THRESHOLD = 3.5;
    // public static final double SINGLE_TAG_DISTANCE_THRESHOLD = 1.5;

    public static final double FAR_TAG_DISTANCE_THRESHOLD = 3.5;
    public static final double CLOSE_TAG_DISTANCE_THRESHOLD = 2.8;

    public static final int SPEAKER_TAG_BLUE = 7;
    public static final int SPEAKER_TAG_RED = 4;

    public static final int SPEAKER_TAG_BLUE_OFF = 8;
    public static final int SPEAKER_TAG_RED_OFF = 3;

    public static final double SPEAKER_TAG_OFFSET = Units.inchesToMeters(33);

    // FIELD WIDTH: 651.223 INCHES

    // x: .2472 y: 5.5946 (USE THIS)
    public static final Pose3d SPEAKER_POSE_BLUE = new Pose3d(0.17, 5.3, SPEAKER_HEIGHT, new Rotation3d(0, 0, 180));
    public static final Pose3d SPEAKER_POSE_RED = new Pose3d(16.37, 5.3, SPEAKER_HEIGHT, new Rotation3d(0, 0, 0)); // TODO: fix X here
  }

  public static class LEDColors {
    public static final int[] NOTHING = {0, 0, 0};
    public static final int[] GREEN = {0, 255, 0};
    public static final int[] ORANGE = {255, 165, 0};
    public static final int[] YELLOW = {255, 255, 0};
    public static final int[] BLUE = {0, 0, 255};
    public static final int[] RED = {255, 0, 0};
  }
}
