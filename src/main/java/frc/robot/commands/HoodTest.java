// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Gains;
import frc.robot.subsystems.Shooter;

public class HoodTest extends CommandBase {

  private Shooter shooter;
  private double angle;

  Gains gains = new Gains(0.0, 0.0, 0.0, 0.0, 0, 0.0);
  PIDController pid;

  /** Creates a new HoodTest. */
  public HoodTest(Shooter shooter, double angle) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.shooter = shooter;
    this.angle = angle;

    addRequirements(shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    pid = new PIDController(gains.kP, gains.kI, gains.kD);
    pid.enableContinuousInput(-1, 1);
    pid.setTolerance(5);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double ticks = shooter.hoodDesiredTicks(angle);

    if (
      ticks <
      (shooter.MAX_HOOD_HEIGHT_INCHES / shooter.HOOD_WHEEL_CIRCUMFERENCE) *
      shooter.REV_TICKS_PER_REV
    ) {
      shooter.commandHood(pid.calculate(shooter.getHoodTicks(), ticks));
    } else {
      shooter.commandHood(0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.commandHood(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return pid.atSetpoint();
  }
}
