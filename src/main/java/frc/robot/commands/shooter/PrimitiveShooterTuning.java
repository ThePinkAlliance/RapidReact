// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.util.datalog.StringLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.HoodConstants;
import frc.robot.LinearInterpolationTable;
import frc.robot.TargetPackage;
import frc.robot.TargetPackageFactory;
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
  private LinearInterpolationTable distanceTable;
  private DoubleLogEntry distanceEntry;
  private DoubleLogEntry rpmEntry;
  private DoubleLogEntry kpEntry;
  private DoubleLogEntry kfEntry;
  private DoubleLogEntry distanceRawEntry;
  private StringLogEntry targetTypeEntry;
  private DataLog log;

  /** Creates a new PrimimitveShooter. */
  public PrimitiveShooterTuning(
      Shooter m_shooter,
      Limelight m_limeLight,
      Hood m_hood,
      Joystick joystick,
      LinearInterpolationTable table,
      int button_id) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_shooter = m_shooter;
    this.m_limelight = m_limeLight;
    this.m_hood = m_hood;
    this.button_id = button_id;
    this.joystick = joystick;
    this.distanceTable = table;

    DataLogManager.start("logs");

    this.log = DataLogManager.getLog();

    this.distanceEntry = new DoubleLogEntry(log, "/shooter/distance");
    this.rpmEntry = new DoubleLogEntry(log, "/shooter/rpm");
    this.kpEntry = new DoubleLogEntry(log, "/shooter/kp");
    this.kfEntry = new DoubleLogEntry(log, "/shooter/kf");
    this.distanceRawEntry = new DoubleLogEntry(log, "/shooter/distance-raw");
    this.targetTypeEntry = new StringLogEntry(log, "/shooter/target-type");

    this.log.start("shooter-logs", "subsystem");

    addRequirements(m_shooter, m_hood);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

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
    boolean ceilingShot = joystick.getPOV() == Constants.JOYSTICK_POV_RIGHT;

    double unmodifiedDistance = m_limelight.calculateUnmodifiedDistance();
    double distance = distanceTable.interp(unmodifiedDistance);

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
    } else if (ceilingShot) {
      currentPackage = TargetPackageFactory.getCustomPackage(300);

      currentPackage.hoodPosition = 0;
      type = "ceiling";
    } else {
      System.out.println("Custom Package Distance: " + distance);

      /*
       * If the linear interpolation table cannot find two points to interpolate
       * between then it will return NaN and in this case we handle it by making no
       * changes to currentPackage.
       */
      if (distance != Double.NaN) {
        currentPackage = TargetPackageFactory.getCustomPackage(distance);
      }

      // if (currentPackage == null) {
      // currentPackage = TargetPackageFactory.getTarmacPackage();
      // distanceEntry.setMetadata("currentPackage=null");
      // }

      type = "custom";
    }

    /*
     * Appending data to the wpilib datalogger.
     */
    this.distanceEntry.append(distance);
    this.distanceRawEntry.append(unmodifiedDistance);
    this.rpmEntry.append(currentPackage.rpm);
    this.kpEntry.append(currentPackage.Kp);
    this.kfEntry.append(currentPackage.Kf);
    this.targetTypeEntry.append(type);

    boolean ready = m_shooter.readyToShoot(currentPackage.rpm, 100);

    if ((!Double.isNaN(currentPackage.Kf) || !Double.isNaN(currentPackage.rpm)) && !ceilingShot) {
      m_hood.setPosition(currentPackage.hoodPosition);
      this.m_shooter.configKp(currentPackage.Kp);
      this.m_shooter.configFeedForward(currentPackage.Kf);
      this.m_shooter.commandRpm(currentPackage.rpm);
    } else if (ceilingShot) {
      m_hood.setPosition(0);
      m_shooter.command(1);
    }

    System.out.println("unmod: " + unmodifiedDistance);
    System.out.println("rpm: " + currentPackage.rpm + ", kp: " + currentPackage.Kp + ", kf: " + currentPackage.Kf);

    // Shooter ready is relevant to the drivers.
    SmartDashboard.putBoolean(Dashboard.DASH_SHOOTER_READY, ready);

    /*
     * The rest of these SmartDashboard queries are relevant to developers.
     */
    if (!DriverStation.isFMSAttached()) {
      SmartDashboard.putNumber("hood", currentPackage.hoodPosition);
      SmartDashboard.putNumber(
          Dashboard.DASH_SHOOTER_VELOCITY,
          this.m_shooter.getMotorOutputPercent());
      SmartDashboard.putString("type", type);

      SmartDashboard.putNumber(
          Dashboard.DASH_SHOOTER_RPMS,
          this.m_shooter.getMotorRpms());
      SmartDashboard.putNumber(Dashboard.DASH_SHOOTER_P, currentPackage.Kp);
      SmartDashboard.putNumber(Dashboard.DASH_SHOOTER_FF, currentPackage.Kf);
    }
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
