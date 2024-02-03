/* Copyright (C) 2024 Team 334. All Rights Reserved.*/
package frc.robot.utils.helpers;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

/** Singleton class that helps retrieve limelight info from network tables. */
public class LimelightHelper {
  private static LimelightHelper _instance;

  /** Get the single instance of the LimelightHelper. */
  public static final LimelightHelper getInstance() {
    if (_instance == null) {
      _instance = new LimelightHelper();
    }

    return _instance;
  }

  private final NetworkTableInstance _inst = NetworkTableInstance.getDefault();
  private final NetworkTable _limelight = _inst.getTable("limelight");

  private final ObjectMapper _objectMapper = new ObjectMapper();

  private LimelightHelper() {
  } // private constructor makes this a singleton

  /**
   * Returns a NetworkTableEntry from the limelight network table.
   *
   * @param name
   *            The name of the entry.
   */
  public NetworkTableEntry getEntry(String name) {
    return _limelight.getEntry(name);
  }

  /**
   * Returns a JsonNode array containing found tags and their info.
   *
   * @see JsonNode
   */
  public JsonNode getTags() {
    String jsonString = getEntry("json").getString("");

    JsonNode tags;

    try {
      tags = _objectMapper.readTree(jsonString).get("Results").get("Fiducial");
    } catch (Exception e) {
      throw new Error("Cannot Read JSON From Limelight.");
    }

    return tags;
  }

  /**
   * Returns a JsonNode containing info of a tag.
   *
   * @param ID
   *            The ID of the tag to look for.
   * @see JsonNode
   */
  public JsonNode getTag(int ID) {
    JsonNode tags = getTags();

    for (JsonNode tag : tags) {
      if (tag.get("fID").asInt() == ID) {
        return tag;
      }
    }

    return null;
  }
}