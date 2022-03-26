// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Dashboard;
import frc.robot.subsystems.Hood;

public class CommandHood extends CommandBase {

  Hood m_hood;
  Joystick joystick;
  double position;
  int buttonId;
  public static double HUB_SHOT_TICK_COUNT = -22000;
  public static double TARMAC_SHOT_TICK_COUNT = -55000; 


  /** Creates a new CommandHood. */
  public CommandHood(
    Hood m_hood,
    double position,
    int buttonId,
    Joystick joystick
  ) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_hood = m_hood;
    this.position = position;
    this.buttonId = buttonId;
    this.joystick = joystick;

    addRequirements(m_hood);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //Get dashboard value if overriden otherwise use defaults.
    double p = SmartDashboard.getNumber(Dashboard.DASH_HOOD_P, Hood.HOOD_Kp);
    double i = SmartDashboard.getNumber(Dashboard.DASH_HOOD_I, Hood.HOOD_Ki);
    double d = SmartDashboard.getNumber(Dashboard.DASH_HOOD_D, Hood.HOOD_Kd);
    double ff = SmartDashboard.getNumber(Dashboard.DASH_HOOD_FF, Hood.HOOD_FF);

    System.out.println("p: " + p + ", i: " + i + ", d: " + d + ", ff: " + ff);

    this.m_hood.setPID(p, i, d, ff);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double ticks = SmartDashboard.getNumber(Dashboard.DASH_HOOD_TICKS, position);
    SmartDashboard.putNumber(Dashboard.DASH_HOOD_OUTPUT, this.m_hood.getHoodPower());
    SmartDashboard.putNumber(Dashboard.DASH_HOOD_DRAW, this.m_hood.getCurrentDraw());
    this.m_hood.setPosition(ticks);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    SmartDashboard.putNumber(Dashboard.DASH_HOOD_OUTPUT, this.m_hood.getHoodPower());
    SmartDashboard.putNumber(Dashboard.DASH_HOOD_DRAW, this.m_hood.getCurrentDraw());
    this.m_hood.disableCloseLoopControl();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return this.joystick.getRawButtonReleased(buttonId);
  }
}
