// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.debugInfo.DebugInfo;
import frc.robot.subsystems.Collector;
import frc.robot.subsystems.Dashboard;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Shooter;

public class AutoShootHood extends CommandBase {

  private double MAX_TIMER_SECS = 3;

  private Shooter m_shooter;
  private Collector m_collector;
  private Hood m_hood;

  private double distance;

  private int ballsShot = 0;
  private boolean shotBefore = false;

  private Timer timer;

  public AutoShootHood(
      Shooter m_shooter,
      Collector m_collector,
      Hood m_hood,
      double distance) {
    // Use addRequirements() here to declare subsystem dependencies.

    this.m_collector = m_collector;
    this.m_shooter = m_shooter;
    this.m_hood = m_hood;
    this.distance = distance;

    this.timer = new Timer();

    addRequirements(this.m_shooter, this.m_collector);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double angle = Math.atan(
        Math.toRadians(
            (Math.tan(Math.toRadians(Shooter.CARGO_INCOMMING_ANGLE)) *
                distance -
                2 *
                    Shooter.SHOOTER_FROM_GROUND)
                /
                -distance));
    double velocity = Math.sqrt(
        (Math.pow(9.8 * distance, 2) *
            (1 + Math.pow(Math.tan(Math.toRadians(angle)), 2)) /
            2 *
            Shooter.SHOOTER_FROM_GROUND -
            2 *
                distance *
                Math.tan(Math.toRadians(angle))));

    // ? this might need the gear ratio added however I don't know that right now
    double rpm = velocity / Shooter.SHOOTER_FLYWHEEL_CIRCUMFRENCE * 60;

    DebugInfo.send("encoder ticks", m_hood.getHoodTicks());
    DebugInfo.send("shooter trajectory velocity", velocity);
    DebugInfo.send("shooter trajectory angle", angle);

    boolean ready = m_shooter.readyToShoot(rpm, 100);

    if (ready) {
      this.m_collector.enableTowerOverride();
    } else {
      this.m_collector.disableTowerOverride();
    }

    this.m_collector.SetSpeedTowerForOverride(Collector.TOWER_MOTOR_FULL_SPEED);
    this.m_shooter.commandRpm(rpm);
    DebugInfo.send(
        Dashboard.DASH_SHOOTER_VELOCITY,
        this.m_shooter.getMotorOutputPercent());
    DebugInfo.send(
        Dashboard.DASH_SHOOTER_RPMS,
        this.m_shooter.getMotorRpms());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    this.m_shooter.command(0);
    this.m_collector.disableTowerOverride();
    this.m_collector.SetSpeedTowerForOverride(0);
    this.timer.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timer.get() > MAX_TIMER_SECS;
  }
}
