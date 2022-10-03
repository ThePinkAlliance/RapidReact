// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.debugInfo;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

/** Add your docs here. */
public class DebugInfo {
  private static ShuffleboardTab tab = Shuffleboard.getTab("debug");
  private static NetworkTable table = NetworkTableInstance.getDefault().getTable("robot-debug");

  public static void send(String title, Object value) {
    boolean matchFound = tab.getComponents().stream().anyMatch((e) -> e.getTitle() == title);

    if (matchFound) {
      table.getEntry(title).setValue(value);
    } else {
      tab.add(title, value).buildInto(table, table.getSubTable("robot-debug-meta"));
    }
  }
}
