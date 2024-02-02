/* Copyright (C) 2024 Team 334. All Rights Reserved.*/
package frc.robot.utils.helpers;

import java.util.Optional;

import edu.wpi.first.wpilibj.DriverStation.Alliance;

/** Singleton class to help manage alliance stuff. */
public class AllianceHelper {
  private static AllianceHelper _instance;

  /** Get the single instance of the AllianceHelper. */
  public static AllianceHelper getInstance() {
    if (_instance == null) {
      _instance = new AllianceHelper();
    }

    return _instance;
  }

  private Alliance _matchAlliance;

  private AllianceHelper() {
  }

  /**
   * Update the match's alliance from the FMS.
   *
   * @param alliance
   *            An optional alliance to update with. If empty, the old alliance is
   *            kept.
   */
  public void updateAlliance(Optional<Alliance> alliance) {
    if (alliance.isPresent()) {
      _matchAlliance = alliance.get();
    }
  }

  /**
   * Get the match's alliance.
   */
  public Alliance getAlliance() {
    return _matchAlliance;
  }
}
