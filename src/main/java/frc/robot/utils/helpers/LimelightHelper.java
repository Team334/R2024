/* Copyright (C) 2024 Team 334. All Rights Reserved.*/
package frc.robot.utils.helpers;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;
import com.fasterxml.jackson.databind.node.ArrayNode;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

/** A class that helps retrieve limelight info from network tables. */
public class LimelightHelper {
  private final NetworkTableInstance _inst = NetworkTableInstance.getDefault();
  private final NetworkTable _limelight;

  private final ObjectMapper _objectMapper = new ObjectMapper();

  public LimelightHelper(String name) {
    _limelight = _inst.getTable(name);
  }

  /**
   * Returns a NetworkTableEntry from the limelight network table.
   *
   * @param name
   *            The name of the entry.
   */
  public NetworkTableEntry getEntry(String name) {
    return _limelight.getEntry(name);
  }

  /** Return json from limelight. */
  public JsonNode getJson() {
    String jsonString = getEntry("json").getString("");
    try {
      return _objectMapper.readTree(jsonString);
    } catch (Exception e) {
      throw new Error("Cannot Read JSON From Limelight.");
    }
  }

  /** 
   * Returns the neural targets from the limelight.
   * 
   * @see ArrayNode
   */
  public ArrayNode getNeuralTargets() {
    ArrayNode targets = (ArrayNode) getJson().get("Results").get("Detector");
    return targets;
  }

  /**
   * Returns the AprilTag targets from the limelight.
   *
   * @see ArrayNode
   */
  public ArrayNode getTagTargets() {
    ArrayNode tags = (ArrayNode) getJson().get("Results").get("Fiducial");

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
    JsonNode tags = getTagTargets();

    for (JsonNode tag : tags) {
      if (tag.get("fID").asInt() == ID) {
        return tag;
      }
    }

    return null;
  }
}
