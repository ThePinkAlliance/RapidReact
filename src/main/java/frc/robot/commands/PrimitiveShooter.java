// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Dashboard;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Shooter;

public class PrimitiveShooter extends CommandBase {

  private Shooter m_shooter;
  private Hood m_hood;
  private Joystick joystick;
  private double rpm;

  private int button_id;

  /** Creates a new PrimimitveShooter. */
  public PrimitiveShooter(
    Shooter m_shooter,
    Hood m_hood,
    Joystick joystick,
    double rpm,
    int button_id
  ) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_shooter = m_shooter;
    this.m_hood = m_hood;
    this.button_id = button_id;
    this.rpm = rpm;
    this.joystick = joystick;

    addRequirements(m_shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // double pwr = joystick.getRawAxis(5);

    double cargoIncommingAngle = -69;

    // remeasure this
    double shooterFromGround = 22.27;
    double currentDistance = 108;

    SmartDashboard.putNumber("hood angle", m_hood.getHoodAngle());

    double angle = Math.atan(
      Math.toRadians(
        (
          Math.tan(Math.toRadians(cargoIncommingAngle)) *
          currentDistance -
          2 *
          shooterFromGround
        ) /
        -currentDistance
      )
    );
    double velocity = Math.sqrt(
      (
        Math.pow(9.8 * currentDistance, 2) *
        (1 + Math.pow(Math.tan(Math.toRadians(angle)), 2)) /
        2 *
        shooterFromGround -
        2 *
        currentDistance *
        Math.tan(Math.toRadians(angle))
      )
    );

    SmartDashboard.putNumber("encoder ticks", m_hood.getHoodTicks());
    SmartDashboard.putNumber("shooter trajectory velocity", velocity);
    SmartDashboard.putNumber("shooter trajectory angle", angle);

    rpm = SmartDashboard.getNumber(Dashboard.DASH_SHOOTER_TARGET_RPMS, rpm);
    boolean ready = m_shooter.readyToShoot(rpm, 100);
    SmartDashboard.putBoolean(Dashboard.DASH_SHOOTER_READY, ready);
    this.m_shooter.commandRpm(rpm);
    SmartDashboard.putNumber(
      Dashboard.DASH_SHOOTER_VELOCITY,
      this.m_shooter.getMotorOutputPercent()
    );
    SmartDashboard.putNumber(
      Dashboard.DASH_SHOOTER_RPMS,
      this.m_shooter.getMotorRpms()
    );
    // hood.commandHood(MathUtil.clamp(pwr, -0.2, 0.2));
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
