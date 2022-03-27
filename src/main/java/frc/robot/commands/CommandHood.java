// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.HoodConstants;
import frc.robot.TargetPackage;
import frc.robot.subsystems.Dashboard;
import frc.robot.subsystems.Hood;

public class CommandHood extends CommandBase {

  Hood m_hood;
  Joystick joystick;

  private TargetPackage highPackage;
  private TargetPackage tarmacPackage;
  private TargetPackage lowPackage;
  private TargetPackage defualtPackage;
  private TargetPackage currentPackage;

  int buttonId;
  public static double HUB_SHOT_TICK_COUNT = -22000;
  public static double TARMAC_SHOT_TICK_COUNT = -55000;

  /** Creates a new CommandHood. */
  public CommandHood(
    Hood m_hood,
    Joystick joystick,
    TargetPackage highPackage,
    TargetPackage tarmacPackage,
    TargetPackage lowPackage,
    TargetPackage defualtPackage,
    int buttonId
  ) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_hood = m_hood;

    this.lowPackage = lowPackage;
    this.highPackage = highPackage;
    this.tarmacPackage = tarmacPackage;
    this.defualtPackage = defualtPackage;

    this.buttonId = buttonId;
    this.joystick = joystick;

    addRequirements(m_hood);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //Get dashboard value if overriden otherwise use defaults.
    double p = SmartDashboard.getNumber(
      Dashboard.DASH_HOOD_P,
      HoodConstants.kGains.kP
    );
    double i = SmartDashboard.getNumber(
      Dashboard.DASH_HOOD_I,
      HoodConstants.kGains.kI
    );
    double d = SmartDashboard.getNumber(
      Dashboard.DASH_HOOD_D,
      HoodConstants.kGains.kD
    );

    System.out.println("p: " + p + ", i: " + i + ", d: " + d);

    this.m_hood.setPID(p, i, d);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    boolean low = joystick.getPOV() == Constants.JOYSTICK_POV_LEFT;
    boolean tarmac = joystick.getPOV() == Constants.JOYSTICK_POV_DOWN;
    boolean high = joystick.getPOV() == Constants.JOYSTICK_POV_UP;

    if (low) {
      currentPackage = lowPackage;
    } else if (tarmac) {
      currentPackage = tarmacPackage;
    } else if (high) {
      currentPackage = highPackage;
    } else {
      currentPackage = defualtPackage;
    }

    double ticks = SmartDashboard.getNumber(
      Dashboard.DASH_HOOD_TICKS,
      currentPackage.hoodPosition
    );
    SmartDashboard.putNumber(
      Dashboard.DASH_HOOD_OUTPUT,
      this.m_hood.getHoodPower()
    );
    SmartDashboard.putNumber(
      Dashboard.DASH_HOOD_DRAW,
      this.m_hood.getCurrentDraw()
    );
    this.m_hood.setPosition(ticks);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    SmartDashboard.putNumber(
      Dashboard.DASH_HOOD_OUTPUT,
      this.m_hood.getHoodPower()
    );
    SmartDashboard.putNumber(
      Dashboard.DASH_HOOD_DRAW,
      this.m_hood.getCurrentDraw()
    );
    this.m_hood.disableCloseLoopControl();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return !this.joystick.getRawButton(buttonId);
  }
}
