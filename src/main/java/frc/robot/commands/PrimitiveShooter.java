// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.ShooterConstants;
import frc.robot.TargetPackage;
import frc.robot.subsystems.Dashboard;
import frc.robot.subsystems.Shooter;

public class PrimitiveShooter extends CommandBase {

  private Shooter m_shooter;
  private Joystick joystick;
  private TargetPackage highPackage;
  private TargetPackage tarmacPackage;
  private TargetPackage lowPackage;
  private TargetPackage defualtPackage;
  private TargetPackage currentPackage;

  private int button_id;

  /** Creates a new PrimimitveShooter. */
  public PrimitiveShooter(
    Shooter m_shooter,
    Joystick joystick,
    TargetPackage highPackage,
    TargetPackage tarmacPackage,
    TargetPackage lowPackage,
    TargetPackage defualtPackage,
    int button_id
  ) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_shooter = m_shooter;
    this.button_id = button_id;

    this.lowPackage = lowPackage;
    this.highPackage = highPackage;
    this.tarmacPackage = tarmacPackage;
    this.defualtPackage = defualtPackage;

    this.joystick = joystick;

    addRequirements(m_shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double currentDistance = 108;
    boolean low = joystick.getPOV() == 90;
    boolean tarmac = joystick.getPOV() == 315;
    boolean high = joystick.getPOV() == 0;

    double angle = Math.atan(
      Math.toRadians(
        (
          Math.tan(Math.toRadians(Shooter.CARGO_INCOMMING_ANGLE)) *
          currentDistance -
          2 *
          Shooter.SHOOTER_FROM_GROUND
        ) /
        -currentDistance
      )
    );
    double velocity = Math.sqrt(
      (
        Math.pow(9.8 * currentDistance, 2) *
        (1 + Math.pow(Math.tan(Math.toRadians(angle)), 2)) /
        2 *
        Shooter.SHOOTER_FROM_GROUND -
        2 *
        currentDistance *
        Math.tan(Math.toRadians(angle))
      )
    );

    SmartDashboard.putNumber("shooter trajectory velocity", velocity);
    SmartDashboard.putNumber("shooter trajectory angle", angle);

    if (low) {
      currentPackage = lowPackage;
    } else if (tarmac) {
      currentPackage = tarmacPackage;
    } else if (high) {
      currentPackage = highPackage;
    } else {
      currentPackage = defualtPackage;
    }

    double shooterKp = SmartDashboard.getNumber(
      Dashboard.DASH_SHOOTER_P,
      currentPackage.Kp
    );
    double shooterFf = SmartDashboard.getNumber(
      Dashboard.DASH_SHOOTER_FF,
      currentPackage.Kf
    );
    double rpm = SmartDashboard.getNumber(
      Dashboard.DASH_SHOOTER_TARGET_RPMS,
      currentPackage.rpm
    );
    boolean ready = m_shooter.readyToShoot(rpm, 100);

    SmartDashboard.putBoolean(Dashboard.DASH_SHOOTER_READY, ready);

    this.m_shooter.configKp(shooterKp);
    this.m_shooter.configFeedForward(shooterFf);
    this.m_shooter.commandRpm(rpm);

    SmartDashboard.putNumber(
      Dashboard.DASH_SHOOTER_VELOCITY,
      this.m_shooter.getMotorOutputPercent()
    );

    SmartDashboard.putNumber(
      Dashboard.DASH_SHOOTER_RPMS,
      this.m_shooter.getMotorRpms()
    );
    SmartDashboard.putNumber(Dashboard.DASH_SHOOTER_P, shooterKp);
    SmartDashboard.putNumber(Dashboard.DASH_SHOOTER_FF, shooterFf);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_shooter.command(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return joystick.getRawButtonReleased(button_id);
  }
}
