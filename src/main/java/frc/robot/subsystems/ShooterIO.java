// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

/** Add your docs here. */
public class ShooterIO implements LoggableInputs {
  public static double rpm = 0;
  public static double kP = 0;
  public static double kF = 0;
  public static double kD = 0;

  @Override
  public void toLog(LogTable table) {
    table.put("shooter-kP", kP);
    table.put("shooter-kF", kF);
    table.put("shooter-kD", kD);
    table.put("shooter-rpm", rpm);
  }

  @Override
  public void fromLog(LogTable table) {
    ShooterIO.rpm = table.getDouble("shooter-rpm", 0);
    ShooterIO.kP = table.getDouble("shooter-kP", 0);
    ShooterIO.kF = table.getDouble("shooter-kF", 0);
    ShooterIO.kD = table.getDouble("shooter-kD", 0);
  }
}
