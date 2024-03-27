// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import java.util.Comparator;

import com.fasterxml.jackson.databind.JsonNode;

public class NoteSort implements Comparator<JsonNode> {
    @Override
    public int compare(JsonNode noteA, JsonNode noteB)
    {
        double ta_A = noteA.get("ta").asDouble(0);
        double ta_B = noteB.get("ta").asDouble(0);

        return (int) Math.signum(ta_A - ta_B);
    }
}
