// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.util.datalog.StringLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.HoodConstants;
import frc.robot.TargetPackage;
import frc.robot.TargetPackageFactory;
import frc.robot.debugInfo.DebugInfo;
import frc.robot.subsystems.Dashboard;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;

public class PrimitiveShooterTuning extends CommandBase {

  private Shooter m_shooter;
  private Limelight m_limelight;
  private Hood m_hood;
  private Joystick joystick;
  private TargetPackage currentPackage;
  private int button_id;
  private DoubleLogEntry distanceEntry;
  private DoubleLogEntry rpmEntry;
  private DoubleLogEntry kpEntry;
  private DoubleLogEntry kfEntry;
  private DoubleLogEntry distanceRawEntry;
  private StringLogEntry targetTypeEntry;

  /** Creates a new PrimimitveShooter. */
  public PrimitiveShooterTuning(
      Shooter m_shooter,
      Limelight m_limeLight,
      Hood m_hood,
      Joystick joystick,
      int button_id) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_shooter = m_shooter;
    this.m_limelight = m_limeLight;
    this.m_hood = m_hood;
    this.button_id = button_id;
    this.joystick = joystick;

    DataLog log = DataLogManager.getLog();

    this.distanceEntry = new DoubleLogEntry(log, "/shooter/distance");
    this.rpmEntry = new DoubleLogEntry(log, "/shooter/rpm");
    this.kpEntry = new DoubleLogEntry(log, "/shooter/kp");
    this.kfEntry = new DoubleLogEntry(log, "/shooter/kf");
    this.distanceRawEntry = new DoubleLogEntry(log, "/shooter/distance-raw");
    this.targetTypeEntry = new StringLogEntry(log, "/shooter/target-type");

    addRequirements(m_shooter, m_hood);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    DataLogManager.start("logs");

    m_hood.setPID(
        HoodConstants.kGains.kP,
        HoodConstants.kGains.kI,
        HoodConstants.kGains.kD);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    boolean low = joystick.getPOV() == Constants.JOYSTICK_POV_DOWN;
    boolean tarmac = joystick.getPOV() == Constants.JOYSTICK_POV_LEFT;
    boolean high = joystick.getPOV() == Constants.JOYSTICK_POV_UP;

    double distance = m_limelight.calculateDistanceHypot();
    double unmodifiedDistance = m_limelight.calculateUnmodifiedDistance();

    String type = "";

    if (low) {
      currentPackage = TargetPackageFactory.getLowHubPackage();
      System.out.println("Low Hub Package");

      type = "low";
    } else if (tarmac) {
      currentPackage = TargetPackageFactory.getTarmacPackage();
      System.out.println("Tarmac Package");

      type = "tarmac";
    } else if (high) {
      currentPackage = TargetPackageFactory.getHighHubPackage();
      System.out.println("High Hub Package");

      type = "high";
    } else {
      System.out.println("Custom Package Distance: " + distance);
      currentPackage = TargetPackageFactory.getCustomPackage(distance);

      type = "custom";
    }

    log(distance, unmodifiedDistance, type, currentPackage);

    m_hood.setPosition(currentPackage.hoodPosition);

    boolean ready = m_shooter.readyToShoot(currentPackage.rpm, 100);
    SmartDashboard.putBoolean(Dashboard.DASH_SHOOTER_READY, ready);

    this.m_shooter.configKp(currentPackage.Kp);
    this.m_shooter.configFeedForward(currentPackage.Kf);
    this.m_shooter.commandRpm(currentPackage.rpm);

    DebugInfo.send("kp", currentPackage.Kp);
    DebugInfo.send("kf", currentPackage.Kf);
    DebugInfo.send("rpm", currentPackage.rpm);
    DebugInfo.send("hoodPosition", currentPackage.hoodPosition);
  }

  private void log(double distance, double unmodifiedDistance, String type, TargetPackage currentPackage) {
    this.distanceEntry.append(distance);
    this.distanceRawEntry.append(unmodifiedDistance);

    this.rpmEntry.append(currentPackage.rpm);
    this.kpEntry.append(currentPackage.Kp);
    this.kfEntry.append(currentPackage.Kf);

    this.targetTypeEntry.append(type);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_shooter.command(0);
    m_hood.disableCloseLoopControl();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return !joystick.getRawButton(button_id);
  }
}
