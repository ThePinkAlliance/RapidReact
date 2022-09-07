package frc.robot;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class BooleanEntry {
  private NetworkTableEntry entry;
  private boolean defaultValue;

  public BooleanEntry(String table, String entry) {
    this.entry = NetworkTableInstance.getDefault().getTable(table).getEntry(entry);
    this.defaultValue = false;
  }

  public BooleanEntry(String table, String entry, boolean defaultValue) {
    this.entry = NetworkTableInstance.getDefault().getTable(table).getEntry(entry);
    this.defaultValue = defaultValue;
  }

  public boolean get() {
    return this.entry.getBoolean(defaultValue);
  }

  public boolean get(boolean defaultValue) {
    return this.entry.getBoolean(defaultValue);
  }
}
