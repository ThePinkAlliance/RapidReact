// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.HoodConstants;
import frc.robot.TargetPackage;
import frc.robot.TargetPackageFactory;
import frc.robot.subsystems.Dashboard;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;

public class ShooterCalibration extends CommandBase {
  private Shooter m_shooter;
  private Limelight m_limelight;
  private Hood m_hood;
  private Joystick joystick;
  private TargetPackage currentPackage;
  private int button_id;

  /** Creates a new ShooterCalibration. */
  public ShooterCalibration(Shooter shooter, Limelight limelight, Hood hood, Joystick joystick,
      TargetPackage defaultTargetPackage, int button_id) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.button_id = button_id;
    this.currentPackage = defaultTargetPackage;
    this.joystick = joystick;
    this.m_hood = hood;
    this.m_limelight = limelight;
    this.m_shooter = shooter;

    addRequirements(shooter, limelight, hood);
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

    m_hood.setPosition(currentPackage.hoodPosition);

    boolean ready = m_shooter.readyToShoot(currentPackage.rpm, 100);
    double estHoodAngle = Hood.getHoodAngle(currentPackage.hoodPosition);
    SmartDashboard.putBoolean(Dashboard.DASH_SHOOTER_READY, ready);

    this.m_shooter.configKp(currentPackage.Kp);
    this.m_shooter.configFeedForward(currentPackage.Kf);
    this.m_shooter.commandRpm(currentPackage.rpm);
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
    return joystick.getRawButtonPressed(button_id);
  }
}
