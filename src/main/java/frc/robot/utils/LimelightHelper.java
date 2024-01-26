/*                                  Team 334                                  */
/* Copyright (c) 2024 Team 334. All Rights Reserved.                          */

package frc.robot.utils;

/** Singleton class that helps retrieve limelight info from network tables. */
public class LimelightHelper {
  private static LimelightHelper _instance;

  public static final LimelightHelper getInstance() {
    if (_instance == null) {
      _instance = new LimelightHelper();
    }

    return _instance;
  }

  private LimelightHelper() {}
}
