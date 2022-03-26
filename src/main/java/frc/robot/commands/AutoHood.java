// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Hood;

public class AutoHood extends CommandBase {

  Hood m_hood;
  Joystick joystick;
  double position;
  int buttonId;
  public static double HUB_SHOT_TICK_COUNT = -22000;
  public static double TARMAC_SHOT_TICK_COUNT = -55000; 


  /** Creates a new CommandHood. */
  public AutoHood(
    Hood m_hood,
    double position
  ) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_hood = m_hood;
    this.position = position;

    addRequirements(m_hood);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    double p = SmartDashboard.getNumber("hood p", 1);
    double i = SmartDashboard.getNumber("hood i", 0);
    double d = SmartDashboard.getNumber("hood d", 0);
    double ff = SmartDashboard.getNumber("hood ff", 0);

    System.out.println("p: " + p + ", i: " + i + ", d: " + d + ", ff: " + ff);

    this.m_hood.setPID(p, i, d, ff);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double ticks = SmartDashboard.getNumber("hood ticks", position);
    SmartDashboard.putNumber("hood output", this.m_hood.getHoodPower());
    SmartDashboard.putNumber("hood draw", this.m_hood.getCurrentDraw());

    this.m_hood.setPosition(ticks);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    SmartDashboard.putNumber("hood output", this.m_hood.getHoodPower());
    SmartDashboard.putNumber("hood draw", this.m_hood.getCurrentDraw());
    m_hood.resetPID();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true; //NEED CONDITION
  }
}
