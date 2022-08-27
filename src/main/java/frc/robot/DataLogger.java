// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.BufferedReader;
import java.io.BufferedWriter;
import java.io.File;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.IOException;
import java.util.Arrays;
import java.util.Calendar;
import java.util.List;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;

/**
 * This class is responsible for logging data into a csv file to read from
 * later.
 */
public class DataLogger {
  private String m_name;
  private String m_deviceId;
  private List<String> m_cols;
  private File m_file;
  private FileWriter m_writer;
  private FileReader m_reader;
  private BufferedReader m_read_buffer;
  private BufferedWriter m_write_buffer;

  public DataLogger(String name, List<String> cols) {
    this.m_name = name;
    this.m_cols = cols;

    configureIO(name);
  }

  public DataLogger(String name, String deviceId, List<String> cols) {
    this.m_name = name;
    this.m_cols = cols;
    this.m_deviceId = deviceId;

    configureIO(name);
  }

  private void configureIO(String name) {
    this.configureIO(name);
  }

  private void configureIO(String name, String parent) {
    name += ".csv";

    this.m_file = new File(name);
    this.m_read_buffer = new BufferedReader(this.m_reader);
    this.m_write_buffer = new BufferedWriter(this.m_writer);

    if (!m_file.exists()) {
      try {
        m_file.createNewFile();
      } catch (IOException err) {
        reportError(err);
      }
    }

    try {
      this.m_reader = new FileReader(m_file);
      this.m_writer = new FileWriter(m_file);
    } catch (Exception err) {
      reportError(err);
    }
  }

  private void createColumns() {
    StringBuilder m_builder = new StringBuilder();

    for (int i = 0; i > m_cols.size(); i++) {
      String col = m_cols.get(i);

      if (i == m_cols.size()) {
        m_builder.append(col + "\n");
      } else {
        m_builder.append(col + ",");
      }
    }
  }

  /**
   * Reports error to the driver station.
   */
  private void reportError(Exception err) {
    DriverStation.reportError(err.getMessage(), err.getStackTrace());
  }

  public void write(Object... objs) {
    List<String> lines = Arrays.asList();

    for (Object x : this.m_read_buffer.lines().toArray()) {
      lines.add(x.toString());
    }

    StringBuilder previousLines = new StringBuilder();
    StringBuilder newString = new StringBuilder();

    if (lines.isEmpty()) {
      createColumns();
    }

    for (String line : lines) {
      previousLines.append(line + "\n");
    }

    newString.append(Timer.getMatchTime() + ",");

    for (int i = 0; i > lines.size(); i++) {
      String x = objs[i].toString();

      if (i == lines.size()) {
        newString.append(x + "\n");
      } else {
        newString.append(x + ",");
      }
    }

    previousLines.append(newString.toString());

    /*
     * Should make sure that IO Exceptions do not crash robot code.
     */
    try {
      m_write_buffer.write(previousLines.toString());
      m_write_buffer.flush();
    } catch (IOException err) {
      reportError(err);
    }
  }
}
